#include "Mantle.h"
#include "ClimbClip.h"
#include "Locomotion.h"
#include "Tuning.h"

namespace Mantle
{
	namespace
	{
		using V = RE::NiPoint3;

		constexpr float kInset = 5.0f;
		constexpr float kWristUp = 7.0f;

		struct Climb
		{
			RE::ActorHandle handle;
			RE::FormID      id = 0;
			int             kind = 0;
			V               p0, O0, D, corr;
			float h0 = 0, want = 0, t = 0, tail = -1.0f, tailLen = 0.35f, kz = 1.0f, Ta = 0.1f;
		};
		std::vector<Climb>                     g_climbs;
		std::unordered_map<RE::FormID, double> g_cool;
		double                                 g_time = 0;
		double                                 g_nextPrune = 30;

		float Smooth01(float x)
		{
			x = std::clamp(x, 0.0f, 1.0f);
			return x * x * (3.0f - 2.0f * x);
		}
		float WrapPi(float a)
		{
			while (a > 3.14159265f) a -= 6.2831853f;
			while (a < -3.14159265f) a += 6.2831853f;
			return a;
		}
		V Lerp(const V& a, const V& b, float t) { return a + (b - a) * t; }
		V ToWorld(float h, const V& c, float kz)
		{
			return V{ std::cos(h), -std::sin(h), 0.0f } * c.x + V{ std::sin(h), std::cos(h), 0.0f } * c.y + V{ 0, 0, c.z * kz };
		}

		void End(const Climb& c, RE::Actor* a)
		{
			g_cool[c.id] = g_time + 1.2;
			if (a) {
				if (auto* cc = a->GetCharController()) {
					cc->SetLinearVelocityImpl(RE::hkVector4(0, 0, 0, 0));
				}
			}
		}
	}

	bool Active(RE::FormID a_id)
	{
		return std::ranges::any_of(g_climbs, [&](const Climb& c) { return c.id == a_id && c.tail < 0.0f; });
	}

	void Reset()
	{
		g_climbs.clear();
		g_cool.clear();
	}

	bool PlanClimb(const Raycast::Caster& ray, const V& P, const V& dir, float h, float maxWall, bool air, Plan& o, float window)
	{
		const auto& cfg = Config::Get().anim;
		const auto* clip = ClimbClip::Get(air ? ClimbClip::kHang : ClimbClip::kVault);
		if (!clip) {
			return false;
		}
		Raycast::Hit wall;
		float        best = 1e9f;
		for (float k : { 0.3f, 0.5f, 0.7f }) {
			Raycast::Hit hit;
			const V      from = P + V{ 0, 0, h * k };
			if (ray.Cast(from, from + dir * maxWall, hit) && !Raycast::IsActorRef(hit.ref) && std::fabs(hit.normal.z) < 0.5f) {
				const float d = (hit.point - from).Length();
				if (d < best) {
					best = d;
					wall = hit;
				}
			}
		}
		if (best > 1e8f) {
			return false;
		}
		V           n{ wall.normal.x, wall.normal.y, 0.0f };
		const float nl = n.Length();
		if (nl < 0.1f) {
			return false;
		}
		n = n * (1.0f / nl);
		if (n.x * -dir.x + n.y * -dir.y < 0.35f) {
			return false;
		}
		const V      in = wall.point - n * 14.0f;
		Raycast::Hit top;
		if (!ray.Cast({ in.x, in.y, P.z + h * 1.6f }, { in.x, in.y, P.z + 20.0f }, top) || top.normal.z < 0.7f || Raycast::IsActorRef(top.ref)) {
			return false;
		}
		const float H = top.point.z - P.z;
		const V     fwd = n * -1.0f;
		if (air) {
			if (std::fabs(top.point.z + kWristUp - clip->mid0.z - P.z) > window) {
				return false;
			}
			Raycast::Hit tmp;
			const V      f = wall.point - fwd * 30.0f;
			const V      f0{ f.x, f.y, top.point.z - 20.0f };
			if (!ray.Cast(f0, f0 + fwd * 60.0f, tmp) || Raycast::IsActorRef(tmp.ref)) {
				return false;
			}
		} else if (H < cfg.climbMin || H > cfg.climbMaxFrac * h) {
			return false;
		}
		const float S = (std::max)(clip->mid0.y - kInset, 16.0f);
		const float nat = clip->frames.back().root.y - S;
		const float dmax = (std::max)(nat, 26.0f);
		bool        found = false;
		for (float d : { dmax, dmax - 8.0f, 26.0f }) {
			const V      s = wall.point + fwd * d;
			Raycast::Hit tmp;
			if (ray.Cast({ s.x, s.y, top.point.z + 2.0f }, { s.x, s.y, top.point.z + h * (air ? 0.55f : 0.9f) }, tmp) && !Raycast::IsActorRef(tmp.ref)) {
				continue;
			}
			float standZ = top.point.z;
			if (!ray.Cast({ s.x, s.y, top.point.z + 30.0f }, { s.x, s.y, top.point.z - 12.0f }, tmp) ||
				(!Raycast::IsActorRef(tmp.ref) && (std::fabs(tmp.point.z - top.point.z) > 10.0f || tmp.normal.z < 0.7f))) {
				continue;
			}
			if (!Raycast::IsActorRef(tmp.ref)) {
				standZ = tmp.point.z;
			}
			o.stand = { s.x, s.y, standZ };
			found = true;
			break;
		}
		if (!found) {
			return false;
		}
		o.edge = { wall.point.x, wall.point.y, top.point.z };
		o.fwd = fwd;
		o.topZ = top.point.z;
		o.H = H;
		o.hang = air;
		return true;
	}

	bool CanClimb(RE::Actor* a)
	{
		static std::unordered_map<RE::TESRace*, bool> cache;
		if (a->IsPlayerRef()) {
			return true;
		}
		auto* race = a->GetRace();
		if (!race || a->GetHeight() >= Config::Get().jump.giantHeight) {
			return false;
		}
		if (auto it = cache.find(race); it != cache.end()) {
			return it->second;
		}
		bool ok = race->HasKeywordString("ActorTypeNPC");
		std::string edid = race->formEditorID.c_str();
		std::ranges::transform(edid, edid.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		for (auto p : Config::Get().jump.climbRaces) {
			std::ranges::transform(p, p.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
			ok = ok || (!p.empty() && edid.find(p) != std::string::npos);
		}
		logger::info("CLIMBRACE '{}' ({}) human anims={}", race->GetName(), race->formEditorID.c_str(), ok);
		return cache[race] = ok;
	}

	bool Start(RE::Actor* a, const Plan& pl)
	{
		if (!CanClimb(a)) {
			return false;
		}
		const auto& cfg = Config::Get().anim;
		const int   kind = pl.hang ? ClimbClip::kHang : ClimbClip::kVault;
		const auto* clip = ClimbClip::Get(kind);
		if (!clip || Active(a->GetFormID())) {
			return false;
		}
		Climb c;
		c.handle = a->GetHandle();
		c.id = a->GetFormID();
		c.kind = kind;
		c.p0 = a->GetPosition();
		c.h0 = a->GetHeading(false);
		c.want = std::atan2(pl.fwd.x, pl.fwd.y);
		c.tailLen = pl.hang ? cfg.hangTail : cfg.climbTail;
		const float S = (std::max)(clip->mid0.y - kInset, 16.0f);
		c.kz = pl.hang ? 1.0f : (pl.H + kWristUp) / clip->mid0.z;
		c.O0 = pl.edge - pl.fwd * S - ToWorld(c.want, { clip->mid0.x, 0, 0 }, 1.0f);
		c.O0.z = pl.hang ? pl.topZ + kWristUp - clip->mid0.z : c.p0.z;
		const V mid = c.O0 + ToWorld(c.want, clip->mid0, c.kz);
		c.corr = pl.edge + pl.fwd * kInset + V{ 0, 0, kWristUp } - mid;
		c.D = pl.stand - (c.O0 + ToWorld(c.want, clip->frames.back().root, c.kz));
		c.Ta = (std::max)(cfg.climbApproach, 0.02f) + (std::max)(std::fabs(c.O0.z - c.p0.z) - 30.0f, 0.0f) / 350.0f;
		g_cool[c.id] = g_time + 3600.0;
		if (cfg.log) {
			const auto* nm = a->GetName();
			logger::info("ANIM {} {:08X} '{}' H={:.0f} {:.2f}s", pl.hang ? "hang climb" : "climb", c.id, nm ? nm : "", pl.H,
				c.Ta + clip->Duration() / cfg.climbRate);
		}
		g_climbs.push_back(std::move(c));
		return true;
	}

	bool TryStart(RE::Actor* a, bool a_air, const V* a_dir, float a_maxWall, float a_window)
	{
		const auto& cfg = Config::Get().anim;
		if (!Locomotion::Settings().enabled || !cfg.climb || (a->IsPlayerRef() && !(Locomotion::Settings().playerClimb && Locomotion::Settings().playerMotion && Locomotion::Settings().player)) || (a_air && !cfg.hang) || Active(a->GetFormID()) || !CanClimb(a)) {
			return false;
		}
		if (auto it = g_cool.find(a->GetFormID()); it != g_cool.end() && g_time < it->second) {
			return false;
		}
		g_cool[a->GetFormID()] = g_time + 0.12;
		Raycast::Caster ray;
		if (!ray.Init(a)) {
			return false;
		}
		float h = a->GetHeight();
		if (!(h > 60.0f && h < 400.0f)) {
			h = 128.0f;
		}
		const float ang = a->data.angle.z;
		const V     dir = a_dir ? *a_dir : V{ std::sin(ang), std::cos(ang), 0.0f };
		Plan        plan;
		if (!PlanClimb(ray, a->GetPosition(), dir, h, a_maxWall, a_air, plan, a_window)) {
			return false;
		}
		return Start(a, plan);
	}

	void OnFrame(float a_dt)
	{
		g_time += a_dt;
		if (g_time >= g_nextPrune) {
			g_nextPrune = g_time + 30.0;
			std::erase_if(g_cool, [&](const auto& kv) { return kv.second < g_time; });
		}
		const auto& cfg = Config::Get().anim;
		for (auto it = g_climbs.begin(); it != g_climbs.end();) {
			auto& c = *it;
			auto  ptr = c.handle.get();
			auto* a = ptr.get();
			const auto* clip = ClimbClip::Get(c.kind);
			const auto& ls = Locomotion::Settings();
			if (!a || !clip || a->IsDead() || !a->Is3DLoaded() || a->IsInRagdollState() || Tuning::GetMode() != Tuning::Mode::kFluid || !ls.enabled ||
				(a->IsPlayerRef() && !(ls.player && ls.playerMotion))) {
				Locomotion::SetClimb(c.id, nullptr);
				End(c, a);
				it = g_climbs.erase(it);
				continue;
			}
			c.t += a_dt;
			const float           Ta = c.Ta, rate = (std::max)(cfg.climbRate, 0.2f);
			const float           Tc = clip->Duration() / rate;
			Locomotion::ClimbPose pose;
			pose.clip = c.kind;
			if (c.tail >= 0.0f) {
				c.tail += a_dt;
				if (c.tail >= c.tailLen) {
					Locomotion::SetClimb(c.id, nullptr);
					it = g_climbs.erase(it);
					continue;
				}
				pose.t = clip->Duration();
				pose.w = 1.0f - Smooth01(c.tail / (std::max)(c.tailLen, 0.02f));
				Locomotion::SetClimb(c.id, &pose);
				++it;
				continue;
			}
			if (c.t >= Ta + Tc) {
				a->SetPosition(c.O0 + ToWorld(c.want, clip->frames.back().root, c.kz) + c.D, true);
				a->IsPlayerRef() ? Locomotion::SetPlayerHeading(c.want) : a->SetHeading(c.want);
				End(c, a);
				c.tail = 0.0f;
				pose.w = 1.0f;
				pose.t = clip->Duration();
				Locomotion::SetClimb(c.id, &pose);
				++it;
				continue;
			}
			V     pos;
			float heading = c.want;
			if (c.t < Ta) {
				const float k = Smooth01(c.t / Ta);
				pos = Lerp(c.p0, c.O0, k);
				pos.z = c.p0.z + (c.O0.z - c.p0.z) * (1.0f - (1.0f - k) * (1.0f - k));
				heading = c.h0 + WrapPi(c.want - c.h0) * k;
				pose.w = k;
				pose.contact = k;
			} else {
				const float tc = (c.t - Ta) * rate;
				const float rel = clip->release;
				pos = c.O0 + ToWorld(c.want, clip->Sample(tc).root, c.kz) + c.D * Smooth01((tc - rel) / (clip->Duration() - rel));
				pose.w = 1.0f;
				pose.t = tc;
				pose.contact = 1.0f - Smooth01((tc - rel) / 0.25f);
			}
			const auto fr = clip->Sample(pose.t);
			for (int i = 0; i < 2; ++i) {
				pose.hand[i] = c.O0 + ToWorld(c.want, fr.hand[i], c.kz) + c.corr;
			}
			a->SetPosition(pos, true);
			a->IsPlayerRef() ? Locomotion::SetPlayerHeading(heading) : a->SetHeading(heading);
			if (auto* cc = a->GetCharController()) {
				cc->SetLinearVelocityImpl(RE::hkVector4(0, 0, 0, 0));
				cc->fallStartHeight = pos.z;
				cc->fallTime = 0.0f;
			}
			Locomotion::NoteDriven(c.id, 0.0f);
			Locomotion::SetClimb(c.id, &pose);
			++it;
		}
	}
}
