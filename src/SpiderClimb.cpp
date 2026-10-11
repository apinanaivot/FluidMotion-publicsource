#include "SpiderClimb.h"
#include "AnimMath.h"
#include "Config.h"
#include "CrashLog.h"
#include "JumpNav.h"
#include "Locomotion.h"
#include "Mantle.h"
#include "Raycast.h"
#include "SpiderLegs.h"
#include "Tuning.h"

namespace SpiderClimb
{
	using namespace AnimMath;

	namespace
	{
		constexpr float kDeg = kPi / 180.0f;

		struct Crawler
		{
			RE::ActorHandle handle, target;
			RE::FormID      id = 0;
			std::string     name;
			V3              goal{};
			bool            pursue = false, hold = false, arrived = false;
			V3              A{}, Av{}, n{ 0, 0, 1 }, nT{ 0, 0, 1 }, fwd{ 0, 1, 0 };
			float           offset = 0, speed = 0, scale = 1, t = 0, stuck = 0, progT = 0, progDist = 1e9f, logT = 0, yaw = 0;
			bool            radiusLogged = false;
			float           slide = 1.0f;
			V3              goodA{}, goodN{ 0, 0, 1 };
			bool            haveGood = false;
			float           flip = 0, goodT = 0, recovT = -9.0f;
			int             recov = 0;
		};

		std::vector<Crawler>                  g_c;
		struct Other
		{
			V3         pos;
			float      r;
			RE::FormID id;
		};
		std::vector<Other> g_others;
		struct Grace
		{
			RE::FormID id;
			float      until;
		};
		std::vector<Grace>                    g_grace;
		std::unordered_map<RE::FormID, float> g_cool;
		struct Memo
		{
			RE::FormID target = 0;
			float      best = 1e9f, bestT = 0, offT = -1.0f;
		};
		std::unordered_map<RE::FormID, Memo>  g_memo;
		float                                 g_time = 0, g_scanT = 0, g_statT = 0;
		int                                   g_endLogs = 0;

		bool On() { return Config::Get().climb.enabled && Config::Get().spider.enabled && Locomotion::Settings().enabled && Tuning::GetMode() == Tuning::Mode::kFluid; }

		Crawler* FindC(RE::FormID a_id)
		{
			for (auto& c : g_c) {
				if (c.id == a_id) {
					return &c;
				}
			}
			return nullptr;
		}

		V3 Face(const V3& a_n, const V3& a_toward) { return Norm(a_n.Dot(a_toward) >= 0.0f ? a_n : a_n * -1.0f, a_toward); }

		V3 OnPlane(const V3& a_v, const V3& a_n, const V3& a_fallback)
		{
			const V3 p = a_v - a_n * a_v.Dot(a_n);
			return Norm(p, a_fallback);
		}

		V3 DesiredDir(const V3& a_d, const V3& a_n, const V3& a_fwd)
		{
			const float g = a_d.Dot(a_n);
			const V3    ds = a_d - a_n * g;
			const float dl = ds.Length();
			V3          m = dl > 4.0f ? ds * (1.0f / dl) : OnPlane(a_fwd, a_n, a_fwd);
			const V3    up = OnPlane({ 0, 0, 1 }, a_n, { 0, 0, 0 });
			const float upl = up.Length();
			if (upl > 0.3f) {
				const V3 upD = up * (1.0f / upl);
				if (g < 0.0f) {
					const float al = Smooth(-g, 0.0f, 60.0f);
					m = Norm(m * (1.0f - al) + upD * al, m);
				} else if (dl < 6.0f) {
					m = upD * -1.0f;
				}
			}
			return m;
		}

		float Hor(const V3& a_n) { return std::sqrt((std::max)(0.0f, 1.0f - a_n.z * a_n.z)); }

		struct Step
		{
			V3 A, n;
			int kind = 0;
		};

		int g_backface = 0, g_inside = 0, g_hidden = 0, g_probes = 0, g_tunnel = 0;

		bool Advance(const Raycast::Caster& r, const Crawler& c, const V3& a_m, float a_s, Step& a_out)
		{
			const auto& cfg = Config::Get().climb;
			const float sc = c.scale, H = cfg.hover * sc, body = 22.0f * sc;
			const V3&   A = c.A;
			const V3&   n = c.nT;
			const float minNz = std::cos(cfg.maxAngleDeg * kDeg);
			auto cast = [&](const V3& from, const V3& to, Raycast::Hit& hit) {
				if (!r.CastSolid(from, to, hit) || Raycast::IsActorRef(hit.ref)) {
					return false;
				}
				++g_probes;
				const V3 dir = Norm(to - from, { 0, 0, -1 });
				if ((hit.point - from).Length() < 1.5f) {
					++g_inside;
					return false;
				}
				if (Norm(hit.normal, dir * -1.0f).Dot(dir) > 0.15f) {
					++g_backface;
					return false;
				}
				return true;
			};
			auto exposed = [&](const V3& P, const V3& N) {
				Raycast::Hit h;
				if (!r.CastSolid(P + N * (120.0f * sc), P - N * 2.0f, h) || Raycast::IsActorRef(h.ref)) {
					return true;
				}
				const bool ok = (h.point - P).Length() < 8.0f;
				g_hidden += ok ? 0 : 1;
				return ok;
			};
			Raycast::Hit hit;

			{
				const V3 o = A + n * (H * 0.5f);
				if (cast(o, o + a_m * (a_s + body), hit)) {
					const V3 n2 = Norm(hit.normal, a_m * -1.0f);
					if (n2.Dot(n) < 0.9f) {
						if (n2.z < minNz) {
							return false;
						}
						if (exposed(hit.point, n2)) {
							a_out = { hit.point, n2, 1 };
							return true;
						}
					}
				}
			}
			const V3 p1 = A + a_m * a_s;
			{
				const V3 from = p1 + n * H;
				if (cast(from, p1 - n * (20.0f * sc), hit)) {
					const V3 nh = Norm(hit.normal, n);
					if (nh.Dot(n) >= 0.5f && nh.z >= minNz && (nh.Dot(n) > 0.95f || exposed(hit.point, nh))) {
						a_out = { hit.point, nh, 2 };
						return true;
					}
				}
			}
			for (float back : { 8.0f, 24.0f }) {
				const V3 from = p1 - n * (back * sc);
				if (cast(from, from - a_m * (H + 24.0f * sc), hit)) {
					const V3 nh = Norm(hit.normal, from - hit.point);
					if (nh.Dot(n) < 0.7f && nh.z >= minNz && exposed(hit.point, nh)) {
						a_out = { hit.point, nh, 3 };
						return true;
					}
				}
			}
			return false;
		}

		void ReleaseController(RE::Actor* a, bool a_air)
		{
			if (auto* cc = a->GetCharController()) {
				const float s = RE::bhkWorld::GetWorldScale();
				cc->context.currentState = a_air ? RE::hkpCharacterStateType::kInAir : RE::hkpCharacterStateType::kOnGround;
				cc->SetLinearVelocityImpl(RE::hkVector4(0, 0, 0, 0));
				(void)s;
				cc->fallStartHeight = a->GetPosition().z;
				cc->fallTime = 0.0f;
			}
		}

		void End(Crawler& c, RE::Actor* a, const char* a_reason)
		{
			if (a && c.n.z < 0.75f && !a->IsDead()) {
				g_grace.push_back({ c.id, g_time + 2.5f });
				ReleaseController(a, true);
			}
			const bool bad = std::strcmp(a_reason, "arrived") != 0 && std::strcmp(a_reason, "done") != 0;
			g_cool[c.id] = g_time + (bad ? Config::Get().climb.cooldown : 1.5f);
			if (Config::Get().climb.log && g_endLogs++ < 200) {
				logger::info("CLIMB end {:08X} '{}' {} after {:.1f}s n.z={:.2f} pos=({:.0f},{:.0f},{:.0f}) goal=({:.0f},{:.0f},{:.0f})", c.id, c.name, a_reason, c.t, c.n.z,
					c.A.x, c.A.y, c.A.z, c.goal.x, c.goal.y, c.goal.z);
			}
		}

		void Drive(RE::Actor* a, Crawler& c)
		{
			const V3 pos = c.Av + c.n * c.offset;
			a->SetPosition(pos, true);
			if (Hor(c.fwd) > 0.3f) {
				c.yaw = std::atan2(c.fwd.x, c.fwd.y);
			}
			a->SetHeading(c.yaw);
			if (auto* cc = a->GetCharController()) {
				cc->context.currentState = RE::hkpCharacterStateType::kOnGround;
				cc->SetLinearVelocityImpl(RE::hkVector4(0, 0, 0, 0));
				cc->fallStartHeight = pos.z;
				cc->fallTime = 0.0f;
				if (!c.radiusLogged && Config::Get().climb.log) {
					c.radiusLogged = true;
					logger::info("CLIMB capsule {:08X} radius={:.2f} (x70 = {:.1f}) height={:.1f} scale={:.2f} stand={:.1f}", c.id, cc->radius, cc->radius * 70.0f, cc->actorHeight,
						c.scale, Config::Get().climb.stand * c.scale);
				}
			}
			Locomotion::NoteDriven(c.id, c.speed);
		}

		bool UpdateOne(Crawler& c, float dt)
		{
			const auto& cfg = Config::Get().climb;
			auto        ptr = c.handle.get();
			RE::Actor*  a = ptr.get();
			const char* drop = nullptr;
			if (!a || a->IsDead() || !a->Is3DLoaded() || a->IsInRagdollState()) {
				drop = "lost";
			} else if (a->AsActorState() && a->AsActorState()->GetKnockState() != RE::KNOCK_STATE_ENUM::kNormal) {
				drop = "knocked";
			} else if (c.t > cfg.maxSeconds) {
				drop = "timeout";
			} else if (JumpNav::InFlight(c.id) || Mantle::Active(c.id)) {
				drop = "other";
			}
			RE::Actor* tgt = nullptr;
			if (!drop) {
				if (auto tp = c.target.get(); tp.get()) {
					tgt = tp.get();
					if (tgt->IsDead() || !tgt->Is3DLoaded()) {
						drop = "targetgone";
					}
				} else if (c.pursue) {
					drop = "targetgone";
				}
				if (!drop && c.pursue && !a->IsInCombat()) {
					drop = "calm";
				}
			}
			Raycast::Caster r;
			if (!drop && !r.Init(a)) {
				drop = "noworld";
			}
			if (drop) {
				End(c, a, drop);
				return false;
			}
			c.t += dt;
			c.scale = (a->Get3D(false) && a->Get3D(false)->world.scale > 0.05f) ? a->Get3D(false)->world.scale : 1.0f;
			const float sc = c.scale;
			if (c.pursue && tgt) {
				c.goal = tgt->GetPosition() + V3{ 0, 0, 0.3f * (std::max)(tgt->GetHeight(), 40.0f) };
			}

			const V3    d = c.goal - c.A;
			const float dist3 = d.Length();
			const float stop = c.pursue ? cfg.stopDist * sc : 18.0f * sc;
			c.arrived = dist3 <= stop;
			if (c.arrived && !c.hold && (!c.pursue || c.n.z > 0.85f)) {
				End(c, a, c.pursue ? "arrived" : "done");
				return false;
			}
			V3       m = DesiredDir(d, c.nT, c.fwd);
			V3 corr{};
			{
				V3          sep{};
				const float myR = cfg.sepRadius * sc;
				for (const auto& o : g_others) {
					if (o.id == c.id) {
						continue;
					}
					const V3    dd = c.A - o.pos;
					const float dist = dd.Length(), minD = (myR + o.r) * 0.9f;
					if (dist >= minD * 1.6f || dist < 0.01f) {
						continue;
					}
					const V3 dir = OnPlane(dd, c.nT, { 0, 0, 0 });
					const V3 dn = dir.Length() > 0.05f ? Norm(dir, dir) : OnPlane(c.fwd, c.nT, c.fwd) * -1.0f;
					const float k = 1.0f - dist / (minD * 1.6f);
					sep = sep + dn * (k * k);
					if (dist < minD) {
						corr = corr + dn * ((minD - dist) * 0.5f);
					}
				}
				if (sep.Length() > 0.01f && !c.arrived) {
					m = Norm(m + sep * 1.5f, m);
				}
			}

			V3 faceDir = m;
			if (tgt) {
				faceDir = OnPlane(tgt->GetPosition() - c.A, c.n, m);
			}
			{
				const float ang = std::atan2(c.n.Dot(c.fwd.Cross(faceDir)), c.fwd.Dot(faceDir));
				float       rate = cfg.turnDeg;
				if (const float cap = Config::Get().spider.maxTurnDeg; cap > 0.0f) {
					rate = (std::min)(rate, cap);
				}
				const float lim = rate * kDeg * dt;
				c.fwd = OnPlane(AxisAngle(c.n, Clamp(ang, -lim, lim)) * c.fwd, c.n, m);
			}

			const bool  struggling = c.flip > 3.0f || c.stuck > 0.5f;
			c.flip = (std::max)(0.0f, c.flip - 1.5f * dt);
			const float cs = c.fwd.Dot(m);
			const float factor = cs >= 0.0f ? cfg.sideFrac + (1.0f - cfg.sideFrac) * cs : cfg.sideFrac + (cfg.backFrac - cfg.sideFrac) * -cs;
			float       want = c.arrived ? 0.0f : cfg.speed * factor * (c.pursue ? Smooth(dist3, stop, stop + 120.0f * sc) : Smooth(dist3, 18.0f * sc, 120.0f * sc)) * (struggling ? 0.4f : 1.0f);
			c.speed = want > c.speed ? (std::min)(want, c.speed + cfg.accel * dt) : (std::max)(want, c.speed - 2.0f * cfg.accel * dt);

			if (c.speed > 1.0f) {
				Step        st;
				const float s = c.speed * dt;
				auto crosses = [&](const Step& t) {
					const float    hv = cfg.hover * c.scale * 0.7f;
					Raycast::Hit   h;
					const bool     x = r.CastSolid(c.A + c.nT * hv, t.A + t.n * hv, h) && !Raycast::IsActorRef(h.ref);
					g_tunnel += x ? 1 : 0;
					return x;
				};
				bool        ok = Advance(r, c, m, s, st) && !crosses(st);
				for (int k = 0; k < 6 && !ok; ++k) {
					const float deg = (k < 2 ? 35.0f : k < 4 ? 70.0f : 105.0f) * ((k & 1) ? -1.0f : 1.0f) * c.slide;
					const V3    m2 = Norm(AxisAngle(c.nT, deg * kDeg) * m, m);
					if (Advance(r, c, m2, s, st) && !crosses(st)) {
						ok = true;
						if (deg < 0.0f) {
							c.slide = -c.slide;
						}
					}
				}
				if (ok) {
					if (st.n.Dot(c.nT) < 0.85f) {
						c.flip += 1.0f;
					}
					c.A = st.A;
					c.nT = st.n;
					c.stuck = (std::max)(0.0f, c.stuck - dt);
					if (!struggling && c.t - c.goodT > 0.3f) {
						c.goodT = c.t;
						c.goodA = c.A;
						c.goodN = c.nT;
						c.haveGood = true;
					}
				} else {
					c.stuck += dt;
					c.speed *= 0.5f;
				}
			}
			if (corr.Length() > 0.01f) {
				const float lim = 150.0f * sc * dt;
				c.A = c.A + corr * ((std::min)(corr.Length(), lim) / corr.Length());
			}
			if (struggling && c.haveGood && c.flip + c.stuck * 4.0f > 5.0f && c.t - c.recovT > 1.0f) {
				c.recovT = c.t;
				if (++c.recov >= 3) {
					End(c, a, "wedged");
					return false;
				}
				c.A = c.goodA;
				c.nT = c.goodN;
				c.n = c.goodN;
				c.flip = 0.0f;
				c.stuck = 0.0f;
				c.slide = -c.slide;
				c.speed = 0.0f;
			}
			c.n = Norm(c.n + (c.nT - c.n) * (1.0f - std::exp(-dt / (struggling ? 0.3f : 0.08f))), c.nT);
			c.fwd = OnPlane(c.fwd, c.n, m);
			const float k = 1.0f - std::exp(-dt / (struggling ? 0.22f : 0.05f));
			c.Av = (c.Av - c.A).Length() > 80.0f * sc ? c.A : c.Av + (c.A - c.Av) * k;
			c.offset = cfg.stand * sc * Hor(c.n);

			if (!c.arrived && want > 20.0f) {
				c.progT += dt;
				if (c.progT >= 1.5f) {
					if (dist3 > c.progDist - 12.0f * sc) {
						c.stuck += 1.0f;
					}
					c.progDist = dist3;
					c.progT = 0.0f;
				}
			}
			if (c.stuck > 1.2f) {
				End(c, a, "blocked");
				return false;
			}
			Drive(a, c);
			if (cfg.log && c.t - c.logT > 2.0f) {
				c.logT = c.t;
				logger::info("CLIMB {:08X} '{}' t={:.1f} dist={:.0f} speed={:.0f} n=({:.2f},{:.2f},{:.2f}) fwd.m={:.2f} stuck={:.2f}", c.id, c.name, c.t, dist3, c.speed, c.n.x, c.n.y, c.n.z, cs, c.stuck);
			}
			return true;
		}

		bool Begin(RE::Actor* a, const V3& a_goal, RE::Actor* a_target, bool a_pursue, bool a_hold)
		{
			if (!On() || !a || !a->Is3DLoaded() || a->IsDead() || FindC(a->GetFormID()) || !SpiderLegs::Has(a->GetFormID())) {
				return false;
			}
			Raycast::Caster r;
			if (!r.Init(a)) {
				return false;
			}
			Crawler c;
			c.handle = a->GetHandle();
			c.id = a->GetFormID();
			c.name = a->GetName() ? a->GetName() : "";
			c.pursue = a_pursue;
			c.hold = a_hold;
			c.goal = a_goal;
			if (a_target) {
				c.target = a_target->GetHandle();
			}
			c.A = c.Av = a->GetPosition();
			const float yaw = a->data.angle.z;
			c.fwd = { std::sin(yaw), std::cos(yaw), 0.0f };
			c.yaw = yaw;
			c.scale = (a->Get3D(false) && a->Get3D(false)->world.scale > 0.05f) ? a->Get3D(false)->world.scale : 1.0f;
			Raycast::Hit hit;
			if (r.CastSolid(c.A + V3{ 0, 0, 30.0f * c.scale }, c.A - V3{ 0, 0, 60.0f * c.scale }, hit) && !Raycast::IsActorRef(hit.ref) && hit.normal.z > 0.3f) {
				c.n = c.nT = Norm(hit.normal, { 0, 0, 1 });
				c.A = c.Av = hit.point;
			}
			c.fwd = OnPlane(c.fwd, c.n, { 0, 1, 0 });
			if (Config::Get().climb.log) {
				logger::info("CLIMB start {:08X} '{}' {} goal=({:.0f},{:.0f},{:.0f}) dist={:.0f}", c.id, c.name, a_pursue ? "pursue" : "goto", a_goal.x, a_goal.y, a_goal.z,
					(a_goal - c.A).Length());
			}
			g_c.push_back(std::move(c));
			return true;
		}

		void Scan(float a_now)
		{
			const auto& cfg = Config::Get().climb;
			auto*       player = RE::PlayerCharacter::GetSingleton();
			auto*       lists = RE::ProcessLists::GetSingleton();
			if (!player || !lists) {
				return;
			}
			const V3 pp = player->GetPosition();
			for (auto& h : lists->highActorHandles) {
				auto  ptr = h.get();
				auto* a = ptr.get();
				if (!a || a == player || a->IsDead() || !a->Is3DLoaded() || !a->IsInCombat()) {
					continue;
				}
				const RE::FormID id = a->GetFormID();
				if (FindC(id) || !SpiderLegs::Has(id)) {
					continue;
				}
				if (auto it = g_cool.find(id); it != g_cool.end() && a_now < it->second) {
					continue;
				}
				auto* tgt = a->GetActorRuntimeData().currentCombatTarget.get().get();
				if (!tgt || tgt->IsDead() || !tgt->Is3DLoaded() || (a->GetPosition() - pp).Length() > 2500.0f) {
					continue;
				}
				const V3    d = tgt->GetPosition() - a->GetPosition();
				const float d3 = d.Length(), dh = std::hypot(d.x, d.y);
				auto&       memo = g_memo[id];
				if (memo.target != tgt->GetFormID()) {
					memo = { tgt->GetFormID(), d3, a_now };
				}
				if (d3 < memo.best - 40.0f) {
					memo.best = d3;
					memo.bestT = a_now;
				}
				bool        off = false;
				if (d3 > 250.0f && !JumpNav::OnNavmesh(a, tgt)) {
					if (memo.offT < 0.0f) {
						memo.offT = a_now;
					}
					off = a_now - memo.offT > 0.6f;
				} else {
					memo.offT = -1.0f;
				}
				const bool high = d.z >= cfg.minRise && d3 < cfg.engage;
				const bool stuck = a_now - memo.bestT > cfg.stuckSec && d3 < cfg.engage && dh > cfg.stopDist * 1.5f && !JumpNav::Routing(id);
				if ((high || stuck || off) && d3 > cfg.stopDist * 1.2f && dh > cfg.stopDist * 0.8f) {
					if (cfg.log) {
						logger::info("CLIMB trigger {:08X} '{}' {} dz={:.0f} d3={:.0f} noProgress={:.1f}s", id, a->GetName(), high ? "target above" : off ? "off navmesh" : "stuck", d.z, d3, a_now - memo.bestT);
					}
					Begin(a, tgt->GetPosition(), tgt, true, false);
					memo.bestT = a_now;
				}
			}
		}
	}

	bool Enabled() { return On(); }

	bool Active(RE::FormID a_id) { return FindC(a_id) != nullptr; }

	bool Get(RE::FormID a_id, Pose& a_out)
	{
		const Crawler* c = FindC(a_id);
		if (!c) {
			return false;
		}
		a_out.normal = c->n;
		a_out.fwd = c->fwd;
		a_out.anchor = c->Av;
		a_out.offset = c->offset;
		return true;
	}

	bool GoTo(RE::Actor* a_actor, const RE::NiPoint3& a_goal, bool a_hold) { return Begin(a_actor, a_goal, nullptr, false, a_hold); }

	bool Pursue(RE::Actor* a_actor, RE::Actor* a_target) { return a_target && Begin(a_actor, a_target->GetPosition(), a_target, true, false); }

	void Stop(RE::FormID a_id)
	{
		for (auto it = g_c.begin(); it != g_c.end(); ++it) {
			if (it->id == a_id) {
				auto ptr = it->handle.get();
				End(*it, ptr.get(), "stopped");
				g_c.erase(it);
				return;
			}
		}
	}

	void Reset()
	{
		g_c.clear();
		g_grace.clear();
		g_cool.clear();
		g_memo.clear();
		g_endLogs = 0;
	}

	void OnFrame(float a_delta)
	{
		if (a_delta <= 0.0f || a_delta > 0.5f) {
			return;
		}
		g_time += a_delta;
		if (!On()) {
			for (auto& c : g_c) {
				auto ptr = c.handle.get();
				End(c, ptr.get(), "disabled");
			}
			g_c.clear();
			return;
		}
		CrashLog::Crumb("SpiderClimb");
		{
			g_others.clear();
			auto* player = RE::PlayerCharacter::GetSingleton();
			auto* lists = RE::ProcessLists::GetSingleton();
			if (player && lists && !g_c.empty()) {
				g_others.push_back({ player->GetPosition(), (std::max)(player->GetBoundRadius(), 25.0f), player->GetFormID() });
				for (auto& h : lists->highActorHandles) {
					auto  ptr = h.get();
					auto* o = ptr.get();
					if (!o || o == player || o->IsDead() || !o->Is3DLoaded()) {
						continue;
					}
					g_others.push_back({ o->GetPosition(), (std::max)(o->GetBoundRadius(), 25.0f), o->GetFormID() });
				}
			}
			if (Config::Get().climb.log && !g_c.empty() && g_time - g_statT > 5.0f) {
				g_statT = g_time;
				logger::info("CLIMBSTAT crawlers={} probes={} rejected: backface={} insideStart={} notExposed={} tunnel={}", g_c.size(), g_probes, g_backface, g_inside, g_hidden, g_tunnel);
				g_probes = g_backface = g_inside = g_hidden = g_tunnel = 0;
				for (int l = 0; l < 64; ++l) {
					if (Raycast::g_skipped[l]) {
						logger::info("CLIMBSKIP layer {} ({}) skipped {} ray hits (not a surface)", l, RE::CollisionLayerToString(static_cast<RE::COL_LAYER>(l)), Raycast::g_skipped[l]);
						Raycast::g_skipped[l] = 0;
					}
				}
			}
		}
		for (auto it = g_c.begin(); it != g_c.end();) {
			if (UpdateOne(*it, a_delta)) {
				++it;
			} else {
				it = g_c.erase(it);
			}
		}
		for (auto it = g_grace.begin(); it != g_grace.end();) {
			auto* ref = RE::TESForm::LookupByID<RE::Actor>(it->id);
			if (g_time > it->until || !ref || ref->IsDead()) {
				it = g_grace.erase(it);
				continue;
			}
			if (auto* cc = ref->GetCharController()) {
				cc->fallStartHeight = ref->GetPosition().z;
				cc->fallTime = 0.0f;
			}
			++it;
		}
		if (g_time - g_scanT > 0.3f) {
			g_scanT = g_time;
			Scan(g_time);
		}
	}
}
