#include "MotionMatch.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>

namespace MotionMatch
{
	namespace
	{
		struct Quat
		{
			float x, y, z, w;
		};

		Quat Mul(const Quat& a, const Quat& b)
		{
			return { a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
				a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z };
		}

		Quat Conj(const Quat& a) { return { -a.x, -a.y, -a.z, a.w }; }

		Quat Norm(Quat a)
		{
			const float n = 1.0f / std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w + 1e-12f);
			return { a.x * n, a.y * n, a.z * n, a.w * n };
		}

		Quat Lerp(const Quat& a, Quat b, float u)
		{
			if (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w < 0.0f) {
				b = { -b.x, -b.y, -b.z, -b.w };
			}
			return Norm({ a.x + (b.x - a.x) * u, a.y + (b.y - a.y) * u, a.z + (b.z - a.z) * u, a.w + (b.w - a.w) * u });
		}

		Quat Db16(const Db& d, std::size_t f, std::size_t b)
		{
			const std::int16_t* p = &d.q[(f * d.nb + b) * 4];
			return Norm({ p[0] / 32767.0f, p[1] / 32767.0f, p[2] / 32767.0f, p[3] / 32767.0f });
		}

		Quat Pose(const Db& d, float f, std::size_t b)
		{
			const std::size_t f0 = static_cast<std::size_t>(f), f1 = (std::min)(f0 + 1, static_cast<std::size_t>(d.nf - 1));
			return Lerp(Db16(d, f0, b), Db16(d, f1, b), f - static_cast<float>(f0));
		}

		float Cost(const float* a_q, const float* a_f, float a_best)
		{
			float c = 0;
			for (int i = 0; i < 12; ++i) {
				const float e = a_q[i] - a_f[i];
				c += e * e;
			}
			if (c >= a_best) {
				return c;
			}
			for (int i = 12; i < kFeat; ++i) {
				const float e = a_q[i] - a_f[i];
				c += e * e;
			}
			return c;
		}

		void Predict(const Input& in, float (&q)[kFeat])
		{
			const float speed = std::hypot(in.vx, in.vy);
			const float phi0 = std::atan2(in.vx, in.vy);
			constexpr float T = 0.5f;
			float           x = 0, y = 0;
			constexpr int   steps = 30;
			constexpr float h = 1.0f / steps;
			int             k = 0;
			for (int i = 1; i <= steps; ++i) {
				const float tm = (i - 0.5f) * h;
				const float th = in.omega * T * (1.0f - std::exp(-tm / T));
				float       vx = speed * std::sin(phi0 + th), vy = speed * std::cos(phi0 + th);
				if (in.haveGoal) {
					const float k = 1.0f - std::exp(-tm / in.goalT);
					vx += (in.gx - vx) * k;
					vy += (in.gy - vy) * k;
				}
				x += vx * h;
				y += vy * h;
				if (i % 10 == 0) {
					q[15 + k * 2] = x;
					q[16 + k * 2] = y;
					q[21 + k * 2] = std::sin(in.face[k]);
					q[22 + k * 2] = std::cos(in.face[k]);
					++k;
				}
			}
		}
	}

	const Db* Get()
	{
		static std::unique_ptr<Db> db;
		static bool                tried = false;
		if (tried) {
			return db.get();
		}
		tried = true;
		const char*   path = "Data\\SKSE\\Plugins\\FluidMotion_mm.bin";
		std::ifstream in(path, std::ios::binary);
		auto          rd = [&](void* p, std::size_t n) { in.read(static_cast<char*>(p), static_cast<std::streamsize>(n)); };
		char          magic[4]{};
		std::uint32_t hdr[4]{};
		float         dt = 0;
		rd(magic, 4);
		rd(hdr, 16);
		rd(&dt, 4);
		if (!in || std::string_view(magic, 4) != "FMMM" || (hdr[0] != 1 && hdr[0] != 2) || hdr[1] == 0 || hdr[1] > 128 || hdr[2] < 100 || hdr[3] != kFeat) {
			logger::warn("MotionMatch: {} missing or invalid - motion matching is off", path);
			return nullptr;
		}
		auto d = std::make_unique<Db>();
		d->nb = hdr[1];
		d->nf = hdr[2];
		d->dt = dt;
		for (std::uint32_t i = 0; i < d->nb; ++i) {
			std::uint8_t n = 0;
			rd(&n, 1);
			std::string s(n, '\0');
			rd(s.data(), n);
			d->bones.push_back(std::move(s));
		}
		rd(d->mean, sizeof(d->mean));
		rd(d->scale, sizeof(d->scale));
		rd(&d->ratio, 4);
		rd(&d->ankleH, 4);
		d->feat.resize(static_cast<std::size_t>(d->nf) * kFeat);
		d->flags.resize(d->nf);
		d->comZ.resize(d->nf);
		d->q.resize(static_cast<std::size_t>(d->nf) * d->nb * 4);
		rd(d->feat.data(), d->feat.size() * 4);
		rd(d->flags.data(), d->nf);
		rd(d->comZ.data(), d->nf * 4);
		d->yawRate.assign(d->nf, 0.0f);
		if (hdr[0] >= 2) {
			rd(d->yawRate.data(), d->nf * 4);
			for (float& w : d->yawRate) {
				w = std::isfinite(w) ? std::clamp(w, -12.0f, 12.0f) : 0.0f;
			}
		}
		rd(d->q.data(), d->q.size() * 2);
		if (!in) {
			logger::warn("MotionMatch: {} is truncated - motion matching is off", path);
			return nullptr;
		}
		d->speed.resize(d->nf);
		std::vector<float> hs(d->nf);
		for (std::uint32_t f = 0; f < d->nf; ++f) {
			const float* r = &d->feat[static_cast<std::size_t>(f) * kFeat];
			const float  vx = r[12] / d->scale[12] + d->mean[12], vy = r[13] / d->scale[13] + d->mean[13];
			hs[f] = (d->flags[f] & 1) ? std::hypot(vx, vy) : -1.0f;
		}
		for (std::uint32_t f = 0; f < d->nf; ++f) {
			float sum = 0;
			int   n = 0;
			for (int k = -15; k <= 15; ++k) {
				const std::int64_t g = static_cast<std::int64_t>(f) + k;
				if (g >= 0 && g < d->nf && hs[g] >= 0.0f) {
					sum += hs[g];
					++n;
				}
			}
			d->speed[f] = n ? sum / n : 0.0f;
		}
		logger::info("MotionMatch: {} frames, {} bones, scale {:.3f}, root turn data {}", d->nf, d->nb, d->ratio, hdr[0] >= 2);
		db = std::move(d);
		return db.get();
	}

	void Step(const Db& d, State& s, const Input& in, const Params& p, float dt)
	{
		float q[kFeat]{};
		Predict(in, q);
		const float speed = std::hypot(in.vx, in.vy);
		auto        norm = [&](int i, float v) { q[i] = (v - d.mean[i]) * d.scale[i]; };
		for (int i = 15; i < kFeat; ++i) {
			norm(i, q[i]);
		}
		norm(12, in.vx);
		norm(13, in.vy);
		norm(14, 0.0f);

		auto row = [&](std::uint32_t f) { return &d.feat[static_cast<std::size_t>(f) * kFeat]; };
		auto search = [&](bool a_start, bool a_force, bool a_now = false) {
			const std::uint32_t cur = static_cast<std::uint32_t>(s.f + 0.5f) < d.nf ? static_cast<std::uint32_t>(s.f + 0.5f) : d.nf - 1;
			const bool          curOk = !a_start && (d.flags[cur] & 1);
			if (curOk) {
				for (int i = 0; i < 12; ++i) {
					q[i] = row(cur)[i];
				}
			} else {
				for (int k = 0; k < 2; ++k) {
					for (int c = 0; c < 3; ++c) {
						q[k * 3 + c] = ((in.haveFeet ? in.foot[k][c] : (c == 0 ? (k ? 8.0f : -8.0f) : c == 2 ? d.ankleH : 0.0f)) - d.mean[k * 3 + c]) * d.scale[k * 3 + c];
						q[6 + k * 3 + c] = (0.0f - d.mean[6 + k * 3 + c]) * d.scale[6 + k * 3 + c];
					}
				}
			}
			float         best = 1e30f;
			std::uint32_t bi = 0;
			for (std::uint32_t f = 0; f < d.nf; ++f) {
				if (!(d.flags[f] & 1) || (a_force && std::fabs(static_cast<float>(f) - s.f) < 8.0f)) {
					continue;
				}
				const float k = in.preferVanilla && (d.flags[f] & 4) ? p.vanillaBias : 1.0f;
				const float c = Cost(q, row(f), best / k) * k;
				if (c < best) {
					best = c;
					bi = f;
				}
			}
			++s.searches;
			s.lastCost = best;
			s.curCost = curOk ? Cost(q, row(cur), 1e30f) * (in.preferVanilla && (d.flags[cur] & 4) ? p.vanillaBias : 1.0f) : 1e30f;
			if (a_start || a_force || !curOk || ((s.sinceSwitch > 0.3f || (a_now && s.sinceSwitch > 0.1f)) && best < s.curCost * (1.0f - p.margin))) {
				if (std::fabs(static_cast<float>(bi) - s.f) > 2.0f || a_start || !curOk) {
					s.sinceSwitch = 0;
					if (!a_start) {
						++s.switches;
						if (!s.out.empty()) {
							for (std::uint32_t b = 0; b < d.nb; ++b) {
								const Quat o = { s.out[b * 4], s.out[b * 4 + 1], s.out[b * 4 + 2], s.out[b * 4 + 3] };
								const Quat n = Mul(o, Conj(Db16(d, bi, b)));
								s.off[b * 4] = n.x, s.off[b * 4 + 1] = n.y, s.off[b * 4 + 2] = n.z, s.off[b * 4 + 3] = n.w;
							}
							s.offZ = s.outZ - d.comZ[bi];
						}
					}
					s.f = static_cast<float>(bi);
					s.frame = static_cast<int>(bi);
				}
			}
			s.searchT = 0;
		};

		s.off.resize(d.nb * 4, 0.0f);
		s.out.resize(d.nb * 4, 0.0f);
		if (!s.playing) {
			s.f = 0;
			search(true, false);
			s.playing = true;
			for (std::uint32_t b = 0; b < d.nb; ++b) {
				Quat o = { 0, 0, 0, 1 };
				if (in.graphQ) {
					const Quat g = Norm({ in.graphQ[b * 4], in.graphQ[b * 4 + 1], in.graphQ[b * 4 + 2], in.graphQ[b * 4 + 3] });
					o = Mul(g, Conj(Db16(d, static_cast<std::size_t>(s.f), b)));
				}
				s.off[b * 4] = o.x, s.off[b * 4 + 1] = o.y, s.off[b * 4 + 2] = o.z, s.off[b * 4 + 3] = o.w;
			}
			s.offZ = in.graphQ ? in.graphZ - d.comZ[static_cast<std::size_t>(s.f)] : 0.0f;
			std::copy(s.off.begin(), s.off.end(), s.out.begin());
		} else {
			const std::uint32_t cur = (std::min)(static_cast<std::uint32_t>(s.f), d.nf - 1);
			const float         clip = d.speed[cur];
			const float k = (clip > 25.0f && speed > 35.0f) ? std::clamp(speed / clip, 0.5f, 2.0f) : 1.0f;
			const float target = std::clamp(std::pow(k, p.cadence), p.rateMin, p.rateMax);
			s.rate += (target - s.rate) * (1.0f - std::exp(-dt / 0.15f));
			s.stride += (std::clamp(k / s.rate, 0.75f, 1.4f) - s.stride) * (1.0f - std::exp(-dt / 0.15f));
			s.sinceSwitch += dt;
			s.f += dt / d.dt * s.rate;
			s.searchT += dt;
			const std::uint32_t nxt = static_cast<std::uint32_t>(s.f);
			const bool          end = nxt + 1 >= d.nf || (d.flags[(std::min)(nxt, d.nf - 1)] & 2) || (nxt > cur && (d.flags[cur] & 2));
			if (end) {
				s.f = static_cast<float>(cur);
				search(false, true);
				if (!(d.flags[(std::min)(static_cast<std::uint32_t>(s.f), d.nf - 1)] & 1)) {
					s.playing = false;
				}
			} else if (s.searchT >= 1.0f / p.searchHz || (in.force && s.searchT > 0.05f)) {
				search(false, false, in.force);
			}
		}

		const float k = std::exp(-dt * 0.6931f / (std::max)(p.halflife, 0.01f));
		s.offZ *= k;
		for (std::uint32_t b = 0; b < d.nb; ++b) {
			const Quat o = Lerp({ 0, 0, 0, 1 }, { s.off[b * 4], s.off[b * 4 + 1], s.off[b * 4 + 2], s.off[b * 4 + 3] }, k);
			s.off[b * 4] = o.x, s.off[b * 4 + 1] = o.y, s.off[b * 4 + 2] = o.z, s.off[b * 4 + 3] = o.w;
			const Quat r = Norm(Mul(o, Pose(d, s.f, b)));
			s.out[b * 4] = r.x, s.out[b * 4 + 1] = r.y, s.out[b * 4 + 2] = r.z, s.out[b * 4 + 3] = r.w;
		}
		const std::size_t f0 = (std::min)(static_cast<std::size_t>(s.f), static_cast<std::size_t>(d.nf - 1)), f1 = (std::min)(f0 + 1, static_cast<std::size_t>(d.nf - 1));
		const float       u = s.f - static_cast<float>(f0);
		s.outZ = d.comZ[f0] + (d.comZ[f1] - d.comZ[f0]) * u + s.offZ;
		s.yawRate = (d.yawRate[f0] + (d.yawRate[f1] - d.yawRate[f0]) * u) * s.rate;
		s.vanilla = (d.flags[f0] & 4) != 0;
	}
}
