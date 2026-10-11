#include "SpiderLegs.h"
#include "AnimMath.h"
#include "Config.h"
#include "CrashLog.h"
#include "JumpNav.h"
#include "Locomotion.h"
#include "Mantle.h"
#include "Raycast.h"
#include "SpiderClimb.h"
#include "SpiderRig.h"
#include "Tuning.h"

namespace SpiderLegs
{
	using namespace AnimMath;

	namespace
	{
		constexpr int kMaxLegs = 8;
		constexpr const SpiderRig::Rig* kRigs[] = { &SpiderRig::kFrostbite, &SpiderRig::kDwarven };

		struct LegState
		{
			float cool = 0, lastStart = -9.0f;
			V3    plant{}, nrm{ 0, 0, 1 };
			bool  swing = false;
			float t = 0, dur = 0.15f, apex = 0;
			V3    from{}, to{}, toN{ 0, 0, 1 };
		};

		struct Pub
		{
			V3            foot[kMaxLegs]{};
			V3            nrm[kMaxLegs]{};
			V3            n{ 0, 0, 1 };
			V3            fwd{ 0, 1, 0 };
			V3            orel{};
			float         w = 0, off = 0;
			std::uint32_t frame = 0;
		};

		struct Spider
		{
			std::mutex                    m;
			RE::FormID                    id = 0;
			const SpiderRig::Rig*         rig = nullptr;
			RE::NiPointer<RE::NiAVObject> root;
			RE::NiAVObject*               body = nullptr;
			RE::NiAVObject*               bone[kMaxLegs][5]{};
			std::uint32_t                 seen = 0, postFrame = 0;

			bool      init = false, havePrev = false;
			V3        prevPos{}, vel{};
			float     prevHeading = 0, omega = 0, w = 0, offZ = 0, scale = 1.0f, logT = 0, airT = 0;
			V3        prevFoot[kMaxLegs]{};
			bool      havePrevFoot = false, prevOk = true;
			V3        nS{ 0, 0, 1 }, prevFwd{ 0, 1, 0 };
			LegState  leg[kMaxLegs];
			int       steps = 0;
			float     speedMax = 0, turnMax = 0;
			bool      mtLogged = false;
			std::atomic<float> errMax{ 0.0f };

			float            wrote[kMaxLegs][5][4]{};
			V3               wroteTip[kMaxLegs]{};
			bool             haveWrote = false;
			std::atomic<int> posts[2]{};
			std::string      seq;
			char             cur[8]{};
			int              ovR = 0, ovRW = 0, renders = 0;
			float            worldMax = 0;
			int              pf = 0, pfPosts[2]{}, ovF = 0, ovP = 0, noPost = 0, multi = 0, recommits = 0, probeLogs = 0;
			float            probeT = 0, diffMax = 0;

			struct Redirect
			{
				RE::BShkbAnimationGraph* graph;
				int                      idx;
				RE::NiNode*              real;
				RE::NiNode*              dummy;
			};
			std::vector<Redirect>                                     redir;
			std::vector<RE::BSTSmartPointer<RE::BShkbAnimationGraph>> graphRefs;
			std::vector<RE::NiPointer<RE::NiNode>>                    dummies;
			RE::NiAVObject*                                           src[kMaxLegs][5]{};
			RE::NiAVObject*                                           srcBody = nullptr;
			bool                                                      hooked = false, hookFailed = false;

			Pub pub;
		};

		std::mutex                                           g_mutex;
		std::unordered_map<RE::FormID, std::shared_ptr<Spider>> g_table;
		std::atomic<std::uint32_t>                           g_frame{ 0 };
		float                                                g_time = 0;
		int                                                  g_restLogs = 0;

		std::shared_ptr<Spider> Find(RE::FormID a_id)
		{
			std::lock_guard lk(g_mutex);
			const auto      it = g_table.find(a_id);
			return it == g_table.end() ? nullptr : it->second;
		}

		bool Active() { return Config::Get().spider.enabled && Locomotion::Settings().enabled && Tuning::GetMode() == Tuning::Mode::kFluid; }

		float AngleBetween(const float (&a)[4], const float (&b)[4])
		{
			const float d = std::fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
			return 2.0f * std::acos((std::min)(d, 1.0f)) * 180.0f / kPi;
		}

		bool Resolve(Spider& s, RE::NiAVObject* a_root, const SpiderRig::Rig& a_rig)
		{
			auto get = [&](const char* a_name) { return a_root->GetObjectByName(RE::BSFixedString(a_name)); };
			if (!get(a_rig.probe)) {
				return false;
			}
			s.body = get(a_rig.body);
			if (!s.body || !s.body->parent) {
				return false;
			}
			for (int i = 0; i < a_rig.count; ++i) {
				const auto& l = a_rig.legs[i];
				for (int k = 0; k < 5; ++k) {
					s.bone[i][k] = l.bone[k] ? get(l.bone[k]) : nullptr;
					if (l.bone[k] && !s.bone[i][k]) {
						return false;
					}
				}
				if (s.bone[i][1]->parent != s.bone[i][0] || s.bone[i][2]->parent != s.bone[i][1]) {
					return false;
				}
			}
			return true;
		}

		int Eligible(RE::Actor* a)
		{
			auto* cc = a->GetCharController();
			if (!cc) {
				return 2;
			}
			const bool air = !SpiderClimb::Active(a->GetFormID()) && cc->context.currentState != RE::hkpCharacterStateType::kOnGround;
			if (a->IsDead() || a->IsInRagdollState() || a->IsInKillMove() || a->IsOnMount()) {
				return 2;
			}
			const auto* st = a->AsActorState();
			if (!st || st->IsFlying() || st->IsSwimming() || st->IsBleedingOut() || st->GetSitSleepState() != RE::SIT_SLEEP_STATE::kNormal) {
				return 2;
			}
			if (JumpNav::InFlight(a->GetFormID()) || Mantle::Active(a->GetFormID())) {
				return 2;
			}
			return air ? 1 : 0;
		}

		void CopyGraphPose(Spider& s)
		{
			if (!s.hooked) {
				return;
			}
			for (const auto& r : s.redir) {
				r.real->local = r.dummy->local;
			}
			Refresh(s.body);
		}

		void Unhook(Spider& s)
		{
			if (!s.hooked) {
				return;
			}
			for (const auto& r : s.redir) {
				auto& arr = r.graph->boneNodes;
				if (static_cast<std::uint32_t>(r.idx) < arr.size() && arr[r.idx].node == r.dummy) {
					r.real->local = r.dummy->local;
					arr[r.idx].node = r.real;
				}
			}
			s.redir.clear();
			s.graphRefs.clear();
			s.dummies.clear();
			s.hooked = false;
			s.srcBody = s.body;
			std::memcpy(s.src, s.bone, sizeof(s.src));
			if (Config::Get().spider.log) {
				logger::info("SPIDERHOOK {:08X} unhooked", s.id);
			}
		}

		void Hook(RE::Actor* a, Spider& s)
		{
			if (s.hooked || s.hookFailed) {
				return;
			}
			RE::BSTSmartPointer<RE::BSAnimationGraphManager> mgr;
			if (!a->GetAnimationGraphManager(mgr) || !mgr) {
				return;
			}
			std::vector<RE::NiAVObject*> want{ s.body };
			for (int i = 0; i < s.rig->count; ++i) {
				for (int k = 0; k < 5; ++k) {
					if (s.bone[i][k]) {
						want.push_back(s.bone[i][k]);
					}
				}
			}
			std::vector<Spider::Redirect> found;
			std::vector<bool>             hit(want.size(), false);
			for (std::uint32_t gi = 0; gi < mgr->graphs.size(); ++gi) {
				auto* g = mgr->graphs[gi].get();
				if (!g) {
					continue;
				}
				auto& arr = g->boneNodes;
				if (arr.size() == 0 || arr.size() > 1024 || !arr.data()) {
					continue;
				}
				for (std::uint32_t j = 0; j < arr.size(); ++j) {
					for (std::size_t w = 0; w < want.size(); ++w) {
						if (arr[j].node && arr[j].node == want[w]) {
							found.push_back({ g, static_cast<int>(j), arr[j].node, nullptr });
							hit[w] = true;
						}
					}
				}
			}
			const auto missing = std::count(hit.begin(), hit.end(), false);
			if (missing) {
				s.hookFailed = true;
				logger::info("SPIDERHOOK {:08X} not redirected: {} of {} bones not in the graph bone table ({} graphs)", s.id, missing, want.size(), mgr->graphs.size());
				return;
			}
			std::unordered_map<RE::NiNode*, RE::NiNode*> made;
			for (auto& r : found) {
				auto& d = made[r.real];
				if (!d) {
					RE::NiPointer<RE::NiNode> n(RE::NiNode::Create(0));
					if (!n) {
						s.hookFailed = true;
						s.dummies.clear();
						return;
					}
					n->local = r.real->local;
					s.dummies.push_back(n);
					d = n.get();
				}
				r.dummy = d;
			}
			for (std::uint32_t gi = 0; gi < mgr->graphs.size(); ++gi) {
				if (mgr->graphs[gi]) {
					s.graphRefs.push_back(mgr->graphs[gi]);
				}
			}
			for (auto& r : found) {
				r.graph->boneNodes[r.idx].node = r.dummy;
			}
			s.redir = std::move(found);
			s.hooked = true;
			auto dummyOf = [&](RE::NiAVObject* a_n) -> RE::NiAVObject* {
				const auto it = made.find(static_cast<RE::NiNode*>(a_n));
				return it == made.end() ? a_n : it->second;
			};
			s.srcBody = dummyOf(s.body);
			for (int i = 0; i < s.rig->count; ++i) {
				for (int k = 0; k < 5; ++k) {
					s.src[i][k] = s.bone[i][k] ? dummyOf(s.bone[i][k]) : nullptr;
				}
			}
			if (Config::Get().spider.log) {
				logger::info("SPIDERHOOK {:08X} redirected {} graph bone entries ({} nodes, {} graphs)", s.id, s.redir.size(), made.size(), s.graphRefs.size());
			}
		}

		struct World
		{
			Raycast::Caster ray;
			float           upR = 45.0f, downR = 110.0f;

			bool Ground(const V3& a_p, const V3& a_up, V3& a_pt, V3& a_n) const
			{
				Raycast::Hit hit;
				if (ray.CastSolid(a_p + a_up * upR, a_p - a_up * downR, hit) && !Raycast::IsActorRef(hit.ref)) {
					const V3 nh = Norm(hit.normal, a_up);
					if (nh.Dot(a_up) < -0.2f) {
						a_pt = a_p;
						a_n = a_up;
						return false;
					}
					a_pt = hit.point;
					a_n = nh.Dot(a_up) > 0.3f ? nh : a_up;
					return true;
				}
				a_pt = a_p;
				a_n = a_up;
				return false;
			}
		};

		bool PatchSpeeds(RE::Movement::MaxSpeeds& a_d)
		{
			using namespace RE::Movement;
			const auto& c = Config::Get().spider;
			const float cap = c.maxTurnDeg * kPi / 180.0f;
			bool        changed = false;
			auto        set = [&](float& a_v, float a_new) {
				if (a_v != a_new) {
					a_v = a_new;
					changed = true;
				}
			};
			auto& sp = a_d.speeds;
			for (int m = 0; m < MaxSpeeds::kTotal; ++m) {
				const float f = sp[SPEED_DIRECTIONS::kForward][m];
				if (f > 0.0f) {
					for (int d : { SPEED_DIRECTIONS::kLeft, SPEED_DIRECTIONS::kRight }) {
						if (!(sp[d][m] > 0.0f)) {
							set(sp[d][m], f * c.sideSpeed);
						}
					}
					if (!(sp[SPEED_DIRECTIONS::kBack][m] > 0.0f)) {
						set(sp[SPEED_DIRECTIONS::kBack][m], f * c.backSpeed);
					}
				}
				if (cap > 0.0f && sp[SPEED_DIRECTIONS::kRotations][m] > cap) {
					set(sp[SPEED_DIRECTIONS::kRotations][m], cap);
				}
			}
			if (cap > 0.0f && a_d.rotateWhileMovingRun > cap) {
				set(a_d.rotateWhileMovingRun, cap);
			}
			return changed;
		}

		void PatchActorMovement(RE::Actor* a, Spider& s)
		{
			auto* proc = a->GetActorRuntimeData().currentProcess;
			auto* hi = proc ? proc->high : nullptr;
			if (!hi) {
				return;
			}
			auto&      d = hi->currentMovementType.defaultData;
			const auto before = d;
			if (PatchSpeeds(d) && Config::Get().spider.log && !s.mtLogged) {
				s.mtLogged = true;
				logger::info("SPIDERMT {:08X} '{}' actor movement data patched: L/R/F/B/Rot walk {:.2f}/{:.2f}/{:.2f}/{:.2f}/{:.2f} run {:.2f}/{:.2f}/{:.2f}/{:.2f}/{:.2f} rotMoving {:.2f} "
							 "(now rot {:.2f}/{:.2f} rotMoving {:.2f})",
					s.id, hi->currentMovementType.typeName.c_str(), before.speeds[0][0], before.speeds[1][0], before.speeds[2][0], before.speeds[3][0], before.speeds[4][0],
					before.speeds[0][1], before.speeds[1][1], before.speeds[2][1], before.speeds[3][1], before.speeds[4][1], before.rotateWhileMovingRun, d.speeds[4][0],
					d.speeds[4][1], d.rotateWhileMovingRun);
			}
		}

		V3 HomeWorld(const Spider& s, int i, const V3& a_anchor, const V3& a_right, const V3& a_fwd)
		{
			const auto& hm = s.rig->legs[i].home;
			return a_anchor + (a_right * hm[0] + a_fwd * hm[1]) * s.scale;
		}

		V3 InPlane(const V3& a_v, const V3& a_up) { return a_v - a_up * a_v.Dot(a_up); }

		void Update(RE::Actor* a, Spider& s, const World& a_w, float dt, float now, float a_playerDist)
		{
			const auto& c = Config::Get().spider;
			const auto& rig = *s.rig;
			const V3    pos = a->GetPosition();
			s.scale = s.root->world.scale > 0.05f ? s.root->world.scale : 1.0f;
			const float sc = s.scale;
			if (c.omni) {
				PatchActorMovement(a, s);
			}

			SpiderClimb::Pose cp;
			const bool        crawl = SpiderClimb::Get(s.id, cp);
			const V3          baseUp = crawl ? Norm(cp.normal, { 0, 0, 1 }) : V3{ 0, 0, 1 };
			const V3          anchor = crawl ? V3(cp.anchor) : pos;
			const float       heading = a->data.angle.z;
			const V3          fwdIn = crawl ? V3(cp.fwd) : V3{ std::sin(heading), std::cos(heading), 0.0f };
			const V3          fwdB = Norm(InPlane(fwdIn, baseUp), fwdIn);
			const V3          rightB = fwdB.Cross(baseUp);

			const int  el = Eligible(a);
			s.airT = el == 1 ? s.airT + dt : 0.0f;
			const bool ok = el == 0 || (el == 1 && s.airT < 0.3f);
			if (ok != s.prevOk && c.log) {
				static int fadeLogs = 0;
				if (fadeLogs++ < 40) {
					logger::info("SPIDERFADE {:08X} ok={} reason={} airT={:.2f} w={:.2f}", s.id, ok, el, s.airT, s.w);
				}
			}
			s.prevOk = ok;
			if (!s.init) {
				s.nS = baseUp;
			}
			if (s.havePrev) {
				const V3 d = anchor - s.prevPos;
				if (d.Length() > 150.0f * sc) {
					s.init = false;
					s.vel = {};
					s.nS = baseUp;
				} else {
					s.vel = s.vel + (d * (1.0f / dt) - s.vel) * (1.0f - std::exp(-dt / 0.15f));
					const float om = -std::atan2(baseUp.Dot(s.prevFwd.Cross(fwdB)), s.prevFwd.Dot(fwdB)) / dt;
					s.omega = s.omega + (om - s.omega) * (1.0f - std::exp(-dt / 0.08f));
					s.turnMax = (std::max)(s.turnMax, std::fabs(s.omega));
				}
			}
			s.prevPos = anchor;
			s.prevFwd = fwdB;
			s.havePrev = true;

			const float step = dt / (std::max)(c.fade, 0.02f);
			s.w = ok ? (std::min)(1.0f, s.w + step) : (std::max)(0.0f, s.w - step);
			{
				std::lock_guard lk(s.m);
				if (a->IsDead() || a->IsInRagdollState() || (!ok && s.w <= 0.0f)) {
					Unhook(s);
				} else if (ok) {
					Hook(a, s);
				}
			}
			if (!ok) {
				if (s.w <= 0.0f) {
					s.init = false;
				}
			} else {
				World w = a_w;
				w.upR = 30.0f * sc;
				w.downR = 90.0f * sc;
				const V3    bodyUp = Norm(s.nS, baseUp);
				const V3    fwdL = Norm(InPlane(fwdB, bodyUp), fwdB);
				const V3    rightL = fwdL.Cross(bodyUp);
				const V3    velS = InPlane(s.vel, baseUp);
				const float speed = velS.Length();
				s.speedMax = (std::max)(s.speedMax, speed);
				const V3    dir = speed > 8.0f ? velS * (1.0f / speed) : fwdL;
				const float lat = speed > 8.0f ? std::fabs(dir.Dot(rightL)) : 0.0f;
				const float thr = (std::min)(c.stepDist + c.stepDistSpeed * speed, c.stepDistMax) * (1.0f - (1.0f - c.sideStep) * lat) * sc;
				const float stride = thr * (1.0f + c.overshoot);
				const float swingFrac = c.swingFracSlow + (c.swingFrac - c.swingFracSlow) * Smooth(speed, 40.0f, 300.0f);
				const float dur = Clamp(swingFrac * stride / (std::max)(speed, c.swingSlow), c.swingMin, c.swingMax);

				auto target = [&](int i, float tAhead, V3& a_pt, V3& a_n) {
					const auto& hm = rig.legs[i].home;
					const float om = std::fabs(s.omega) > 0.2f ? s.omega : 0.0f, r = (std::max)(std::hypot(hm[0], hm[1]) * sc, 1.0f);
					const float lead = Clamp(om * 0.25f, -c.overshoot * thr / r, c.overshoot * thr / r);
					const M3    R = AxisAngle(bodyUp, -(om * tAhead + lead));
					const V3    home = HomeWorld(s, i, anchor + velS * tAhead, R * rightL, R * fwdL);
					const V3    p = home + dir * (c.overshoot * thr * Smooth(speed, 15.0f, 80.0f));
					return w.Ground(p, bodyUp, a_pt, a_n);
				};

				if (!s.init) {
					for (int i = 0; i < rig.count; ++i) {
						auto&       L = s.leg[i];
						const V3    h = HomeWorld(s, i, anchor, rightL, fwdL);
						const float ph = ((rig.legs[i].row + (rig.legs[i].right ? 1 : 0)) & 1) ? -1.0f : 1.0f;
						w.Ground(h + fwdL * (ph * 0.35f * c.stepDist * sc), bodyUp, L.plant, L.nrm);
						L.swing = false;
					}
					s.init = true;
					s.offZ = 0.0f;
				}

				int swinging = 0;
				for (int i = 0; i < rig.count; ++i) {
					auto& L = s.leg[i];
					if (!L.swing) {
						continue;
					}
					L.t += dt / L.dur;
					if (L.t >= 1.0f) {
						L.swing = false;
						L.plant = L.to;
						L.nrm = L.toN;
						L.cool = 0.1f;
						++s.steps;
					} else {
						++swinging;
					}
				}

				struct Want
				{
					int   i;
					float u;
				};
				Want want[kMaxLegs];
				int  nWant = 0;
				for (int i = 0; i < rig.count; ++i) {
					auto& L = s.leg[i];
					L.cool = (std::max)(0.0f, L.cool - dt);
					if (L.swing || L.cool > 0.0f) {
						continue;
					}
					const V3    h = HomeWorld(s, i, anchor, rightL, fwdL);
					const float d = InPlane(L.plant - h, bodyUp).Length();
					if (d > thr) {
						want[nWant++] = { i, d / thr };
					}
				}
				std::sort(want, want + nWant, [](const Want& x, const Want& y) { return x.u > y.u; });
				const bool fast = speed > 250.0f;
				const int  maxSwing = (std::max)(1, rig.count / 2) + (fast ? 1 : 0);
				int       started = 0;
				for (int k = 0; k < nWant && started < c.maxPerFrame; ++k) {
					const int i = want[k].i;
					const bool force = want[k].u > 1.25f;
					if (force) {
						if (swinging >= (want[k].u > 2.0f ? rig.count - 2 : rig.count * 3 / 4)) {
							continue;
						}
					} else {
						if (swinging >= maxSwing || s.leg[i ^ 1].swing || (!fast && ((i >= 2 && s.leg[i - 2].swing) || (i + 2 < rig.count && s.leg[i + 2].swing)))) {
							continue;
						}
					}
					auto& L = s.leg[i];
					V3    pt, pn;
					const bool  hit = target(i, dur, pt, pn);
					const float disp = InPlane(pt - L.plant, bodyUp).Length();
					if (disp < 0.25f * thr || (!hit && !force)) {
						L.cool = 0.3f;
						continue;
					}
					if (c.log && now - L.lastStart < 0.35f && speed < 120.0f) {
						static int rapidLogs = 0;
						if (rapidLogs++ < 40) {
							logger::info("SPIDERRAPID {:08X} leg={} restep after {:.0f}ms speed={:.0f} u={:.2f} thr={:.1f} disp={:.1f} hit={}", s.id, i,
								(now - L.lastStart) * 1000.0f, speed, want[k].u, thr, disp, hit);
						}
					}
					L.lastStart = now;
					L.swing = true;
					L.t = 0.0f;
					L.dur = c.footSpeed > 0.0f ? Clamp(disp / (c.footSpeed * sc + 2.0f * speed), dur, (std::max)(dur, c.swingMax)) : dur;
					L.from = L.plant;
					L.to = pt;
					L.toN = pn;
					L.apex = c.stepHeight * sc + (std::max)(0.0f, (L.to - L.from).Dot(bodyUp));
					++swinging;
					++started;
				}
			}

			{
				float wsum = 0, su = 0, sv = 0, sh = 0;
				V3    gp[kMaxLegs];
				float gw[kMaxLegs], gu[kMaxLegs], gv[kMaxLegs], gh[kMaxLegs];
				for (int i = 0; i < rig.count; ++i) {
					const auto& L = s.leg[i];
					gp[i] = L.swing ? L.to : L.plant;
					gw[i] = L.swing ? 0.25f : 1.0f;
					const V3 q = gp[i] - anchor;
					gu[i] = q.Dot(rightB);
					gv[i] = q.Dot(fwdB);
					gh[i] = q.Dot(baseUp);
					wsum += gw[i];
					su += gu[i] * gw[i];
					sv += gv[i] * gw[i];
					sh += gh[i] * gw[i];
				}
				if (s.init && wsum > 0.1f) {
					const float ub = su / wsum, vb = sv / wsum, hb = sh / wsum;
					float       cuu = 0, cuv = 0, cvv = 0, cuh = 0, cvh = 0;
					for (int i = 0; i < rig.count; ++i) {
						const float x = gu[i] - ub, y = gv[i] - vb, z = gh[i] - hb;
						cuu += gw[i] * x * x;
						cuv += gw[i] * x * y;
						cvv += gw[i] * y * y;
						cuh += gw[i] * x * z;
						cvh += gw[i] * y * z;
					}
					const float det = cuu * cvv - cuv * cuv;
					float       ga = 0, gb = 0;
					if (det > 1.0f) {
						ga = (cuh * cvv - cvh * cuv) / det;
						gb = (cvh * cuu - cuh * cuv) / det;
					}
					const float maxG = std::tan(c.maxTiltDeg * kPi / 180.0f), gl = std::hypot(ga, gb);
					if (gl > maxG) {
						ga *= maxG / gl;
						gb *= maxG / gl;
					}
					const V3    n = Norm(baseUp - rightB * ga - fwdB * gb, baseUp);
					const float h0 = hb - ga * ub - gb * vb;
					const float offT = Clamp(h0, -30.0f * sc, 30.0f * sc);
					const float k = 1.0f - std::exp(-dt / (std::max)(c.bodyTime, 0.01f));
					s.offZ += (offT - s.offZ) * k;
					s.nS = Norm(s.nS + (n - s.nS) * (k * 0.7f), baseUp);
				}
			}

			{
				std::lock_guard lk(s.m);
				auto&           p = s.pub;
				const V3        bodyUp = Norm(s.nS, baseUp);
				p.frame = g_frame.load();
				p.w = s.w;
				p.off = s.offZ;
				p.n = bodyUp;
				p.fwd = fwdB;
				p.orel = (anchor - pos) + baseUp * s.offZ;
				for (int i = 0; i < rig.count; ++i) {
					const auto& L = s.leg[i];
					if (L.swing) {
						const float e = Smooth(L.t, 0.0f, 1.0f);
						p.foot[i] = L.from + (L.to - L.from) * e + bodyUp * (L.apex * std::sin(kPi * Clamp(L.t, 0.0f, 1.0f)));
						p.nrm[i] = Norm(bodyUp + (L.toN - bodyUp) * e, bodyUp);
					} else {
						p.foot[i] = L.plant;
						p.nrm[i] = L.nrm;
					}
				}
			}

			if (c.log) {
				static int jumpLogs = 0;
				for (int i = 0; i < rig.count; ++i) {
					const V3 f = s.pub.foot[i];
					if (s.havePrevFoot && s.w > 0.5f && jumpLogs < 60) {
						const float d = (f - s.prevFoot[i]).Length();
						if (d > 40.0f * sc + 3.5f * s.vel.Length() * dt) {
							++jumpLogs;
							const auto& L = s.leg[i];
							logger::info("SPIDERJUMP {:08X} leg={} moved {:.1f} in {:.0f}ms swing={} t={:.2f} dur={:.2f} speed={:.0f} from=({:.0f},{:.0f},{:.0f}) to=({:.0f},{:.0f},{:.0f}) foot=({:.0f},{:.0f},{:.0f})",
								s.id, i, d, dt * 1000.0f, L.swing, L.t, L.dur, s.vel.Length(), L.from.x, L.from.y, L.from.z, L.to.x, L.to.y, L.to.z, f.x, f.y, f.z);
						}
					}
					s.prevFoot[i] = f;
				}
				s.havePrevFoot = true;
			}

			if (c.log && a_playerDist < 1200.0f && now - s.logT > 5.0f) {
				const float span = s.logT > 0.0f ? now - s.logT : 5.0f;
				s.logT = now;
				int sw = 0;
				for (int i = 0; i < rig.count; ++i) {
					sw += s.leg[i].swing ? 1 : 0;
				}
				logger::info("SPIDER {:08X} {} ok={} w={:.2f} speed={:.0f} (max {:.0f}) swinging={}/{} steps/s={:.1f} offZ={:.1f} tilt={:.1f}deg tipMiss(max)={:.1f}u turnMax={:.0f}deg/s", s.id, rig.name, ok, s.w,
					s.vel.Length(), s.speedMax, sw, rig.count, static_cast<float>(s.steps) / span, s.offZ,
					std::acos(Clamp(s.nS.z, -1.0f, 1.0f)) * 180.0f / kPi, s.errMax.exchange(0.0f), s.turnMax * 180.0f / kPi);
				s.steps = 0;
				s.turnMax = 0;
				s.speedMax = 0;
			}
		}

		std::shared_ptr<Spider> Bind(RE::Actor* a, std::uint32_t a_frame)
		{
			RE::NiAVObject* root = a->Get3D(false);
			if (!root) {
				return nullptr;
			}
			auto s = Find(a->GetFormID());
			if (s && s->root.get() == root) {
				s->seen = a_frame;
				return s->rig ? s : nullptr;
			}
			if (s && s->rig) {
				std::lock_guard lk(s->m);
				Unhook(*s);
			}
			auto n = std::make_shared<Spider>();
			n->id = a->GetFormID();
			n->root.reset(root);
			n->seen = a_frame;
			for (const auto* r : kRigs) {
				if (Resolve(*n, root, *r)) {
					n->rig = r;
					n->srcBody = n->body;
					std::memcpy(n->src, n->bone, sizeof(n->src));
					break;
				}
			}
			{
				std::lock_guard lk(g_mutex);
				g_table[n->id] = n;
			}
			if (n->rig && Config::Get().spider.log) {
				float sum = 0, mx = 0;
				int   cnt = 0;
				for (int i = 0; i < n->rig->count; ++i) {
					for (int k = 0; k < 5; ++k) {
						if (!n->bone[i][k]) {
							continue;
						}
						float q[4];
						ToQuat(n->bone[i][k]->local.rotate, q);
						const float ang = AngleBetween(q, n->rig->legs[i].rest[k]);
						sum += ang;
						mx = (std::max)(mx, ang);
						++cnt;
					}
				}
				if (g_restLogs++ < 6) {
					logger::info("SPIDER bound {:08X} '{}' rig={} legs={} restcheck: mean {:.1f} max {:.1f} deg (graph pose vs skeleton rest; ~0 when idle)",
						n->id, a->GetName(), n->rig->name, n->rig->count, cnt ? sum / static_cast<float>(cnt) : 0.0f, mx);
				}
			}
			return n->rig ? n : nullptr;
		}
	}

	bool Has(RE::FormID a_id)
	{
		if (!Active()) {
			return false;
		}
		const auto s = Find(a_id);
		return s && s->rig;
	}

	void Reset()
	{
		std::lock_guard lk(g_mutex);
		for (auto& [id, s] : g_table) {
			if (s->rig) {
				std::lock_guard sl(s->m);
				Unhook(*s);
			}
		}
		g_table.clear();
		g_restLogs = 0;
	}

	namespace
	{
		struct Chain
		{
			int             n = 0;
			RE::NiAVObject* node[5]{};
			M3              lr[5], wr[5];
			V3              lt[5], wt[5];
			M3              pr;
			V3              pt{};
			float           sc = 1.0f;

			void Fk(int from)
			{
				for (int k = from; k < n; ++k) {
					const M3& Mp = k ? wr[k - 1] : pr;
					const V3& tp = k ? wt[k - 1] : pt;
					wr[k] = Mp * lr[k];
					wt[k] = tp + (Mp * lt[k]) * sc;
				}
			}
			void Rot(int k, const M3& a_dR)
			{
				const M3& P = k ? wr[k - 1] : pr;
				lr[k] = P.Transpose() * a_dR * P * lr[k];
			}
		};

		void Solve2(Chain& C, int a, int b, int c, const V3& a_target, const V3& a_pole)
		{
			const V3    A = C.wt[a], B = C.wt[b], Cc = C.wt[c];
			const float l1 = (B - A).Length(), l2 = (Cc - B).Length();
			const V3    AT = a_target - A;
			const float d = AT.Length();
			if (l1 < 1e-3f || l2 < 1e-3f || d < 1e-3f) {
				return;
			}
			const V3    dir = AT * (1.0f / d);
			const float dc = Clamp(d, std::fabs(l1 - l2) + 0.5f, l1 + l2 - 0.2f);
			const float along = (l1 * l1 - l2 * l2 + dc * dc) / (2.0f * dc);
			const float h = std::sqrt((std::max)(l1 * l1 - along * along, 0.0f));
			const V3    pole = Norm(a_pole - dir * a_pole.Dot(dir), Norm((B - A) - dir * (B - A).Dot(dir), { 0, 0, 1 }));
			const V3    newB = A + dir * along + pole * h;
			C.Rot(a, RotBetween(Norm(B - A, dir), Norm(newB - A, dir)));
			C.Fk(a);
			const V3 B2 = C.wt[b], C2 = C.wt[c];
			C.Rot(b, RotBetween(Norm(C2 - B2, dir), Norm(A + dir * dc - B2, dir)));
			C.Fk(b);
		}

		bool Finite(const V3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

		float PoseDiff(const Spider& s)
		{
			float mx = 0;
			for (int i = 0; i < s.rig->count; ++i) {
				int ci = 0;
				for (int k = 0; k < 5; ++k) {
					if (!s.bone[i][k]) {
						continue;
					}
					float q[4];
					ToQuat(s.bone[i][k]->local.rotate, q);
					mx = (std::max)(mx, AngleBetween(q, s.wrote[i][ci++]));
				}
			}
			return mx;
		}

		void ProbeFrame(Spider& s)
		{
			std::lock_guard lk(s.m);
			if (!s.haveWrote) {
				return;
			}
			const int   j = s.posts[0].exchange(0), u = s.posts[1].exchange(0);
			if (g_frame.load() - s.pub.frame > 4) {
				return;
			}
			const float d = PoseDiff(s);
			s.pfPosts[0] += j;
			s.pfPosts[1] += u;
			s.noPost += (j + u) == 0;
			s.multi += (j + u) > 1;
			s.ovF += d > 3.0f;
			s.diffMax = (std::max)(s.diffMax, d);
			s.cur[0] = static_cast<char>('0' + (std::min)(j + u, 9));
			s.cur[1] = d > 3.0f ? 'V' : 'o';
			if (d > 3.0f && Config::Get().spider.recommit && s.pub.w > 0.99f) {
				for (int i = 0; i < s.rig->count; ++i) {
					int ci = 0;
					for (int k = 0; k < 5; ++k) {
						if (s.bone[i][k]) {
							s.bone[i][k]->local.rotate = FromQuat(s.wrote[i][ci++]);
						}
					}
				}
				Refresh(s.body);
				++s.recommits;
			}
		}
	}

	namespace
	{
		struct SavedType
		{
			RE::TESRace*        race;
			int                 slot;
			RE::BGSMovementType* orig;
			RE::BGSMovementType* copy;
		};
		std::vector<SavedType> g_saved;
	}

	void ApplyMovement(bool a_fluid)
	{
		using namespace RE::Movement;
		const auto& c = Config::Get().spider;
		if (!a_fluid) {
			for (auto& sv : g_saved) {
				sv.race->baseMoveTypes[sv.slot] = sv.orig;
			}
			if (!g_saved.empty()) {
				logger::info("SPIDERMOVE restored {} movement types", g_saved.size());
			}
			g_saved.clear();
			return;
		}
		if (!g_saved.empty()) {
			return;
		}
		auto* dh = RE::TESDataHandler::GetSingleton();
		if (!dh) {
			return;
		}
		for (auto* race : dh->GetFormArray<RE::TESRace>()) {
			if (!race) {
				continue;
			}
			std::string edid = race->formEditorID.c_str();
			std::ranges::transform(edid, edid.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
			bool match = false;
			for (auto pat : c.races) {
				std::ranges::transform(pat, pat.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
				match = match || (!pat.empty() && edid.find(pat) != std::string::npos);
			}
			if (!match) {
				continue;
			}
			for (int slot : { RE::TESRace::MovementTypes::kWalk, RE::TESRace::MovementTypes::kRun, RE::TESRace::MovementTypes::kSneak,
					 RE::TESRace::MovementTypes::kSprint }) {
				auto* orig = race->baseMoveTypes[slot];
				if (!orig) {
					continue;
				}
				auto* copy = RE::IFormFactory::Create<RE::BGSMovementType>();
				if (!copy) {
					continue;
				}
				copy->movementTypeData = orig->movementTypeData;
				const auto before = orig->movementTypeData.defaultData;
				PatchSpeeds(copy->movementTypeData.defaultData);
				race->baseMoveTypes[slot] = copy;
				g_saved.push_back({ race, slot, orig, copy });
				if (c.log) {
					const auto& a = copy->movementTypeData.defaultData;
					logger::info("SPIDERMOVE {} slot={} '{}' (shared form {:08X}) L/R/F/B/Rot walk {:.2f}/{:.2f}/{:.2f}/{:.2f}/{:.2f} run {:.2f}/{:.2f}/{:.2f}/{:.2f}/{:.2f} "
								 "(was walk {:.2f}/{:.2f}/{:.2f}/{:.2f}/{:.2f} run {:.2f}/{:.2f}/{:.2f}/{:.2f}/{:.2f}) rotWhileMoving {:.2f} (was {:.2f})",
						race->formEditorID.c_str(), slot, orig->movementTypeData.typeName.c_str(), orig->GetFormID(), a.speeds[0][0], a.speeds[1][0],
						a.speeds[2][0], a.speeds[3][0], a.speeds[4][0], a.speeds[0][1], a.speeds[1][1], a.speeds[2][1], a.speeds[3][1], a.speeds[4][1],
						before.speeds[0][0], before.speeds[1][0], before.speeds[2][0], before.speeds[3][0], before.speeds[4][0], before.speeds[0][1],
						before.speeds[1][1], before.speeds[2][1], before.speeds[3][1], before.speeds[4][1], a.rotateWhileMovingRun, before.rotateWhileMovingRun);
				}
			}
		}
		logger::info("SPIDERMOVE {} movement types replaced (omni={}, max turn {:.0f} deg/s)", g_saved.size(), c.omni, c.maxTurnDeg);
	}

	void Post(RE::Actor* a, int a_src)
	{
		if (!Active()) {
			return;
		}
		const auto sp = Find(a->GetFormID());
		if (!sp || !sp->rig) {
			return;
		}
		CrashLog::Crumb("Spider post");
		auto&           s = *sp;
		std::lock_guard lk(s.m);
		const auto      frame = g_frame.load();
		RE::NiAVObject* root = a->Get3D(false);
		if (!root || root != s.root.get()) {
			return;
		}
		if (a->IsDead() || a->IsInRagdollState() || frame - s.pub.frame > 30) {
			Unhook(s);
			return;
		}
		if (frame - s.pub.frame > 4 || s.pub.w < 0.003f) {
			CopyGraphPose(s);
			return;
		}
		const auto& rig = *s.rig;
		const auto& p = s.pub;
		const float w = p.w, sc = root->world.scale > 0.05f ? root->world.scale : 1.0f;
		const V3 upB = Norm(p.n, { 0, 0, 1 });
		const V3 fwdB = Norm(p.fwd - upB * p.fwd.Dot(upB), { 0, 1, 0 });
		const V3 rightB = fwdB.Cross(upB);
		M3       Rb;
		for (int r = 0; r < 3; ++r) {
			Rb.entry[r][0] = rightB[r];
			Rb.entry[r][1] = fwdB[r];
			Rb.entry[r][2] = upB[r];
		}
		const M3 R0 = root->world.rotate;
		const M3 Rtilt = Rb * R0.Transpose();

		RE::NiAVObject* body = s.body;
		if (s.srcBody && s.srcBody != body) {
			body->local = s.srcBody->local;
		}
		{
			const RE::NiAVObject* bp = body->parent;
			const M3              PR = bp->world.rotate;
			const float           PS = bp->world.scale > 0.05f ? bp->world.scale : 1.0f;
			const V3              lt = body->local.translate;
			const V3              animW = V3(bp->world.translate) + (PR * lt) * PS;
			const V3              O = V3(root->world.translate) + p.orel;
			const V3              tgt = O + (Rtilt * (PR * V3{ lt.x, lt.y, rig.bodyZ })) * PS;
			float                 q[4], r[4], len = 0;
			ToQuat(Rtilt, q);
			const float sg = q[3] < 0.0f ? -1.0f : 1.0f;
			for (int k = 0; k < 4; ++k) {
				r[k] = q[k] * sg * w + (k == 3 ? 1.0f - w : 0.0f);
				len += r[k] * r[k];
			}
			if (std::isfinite(len) && len > 1e-6f) {
				for (float& v : r) {
					v /= std::sqrt(len);
				}
				RotateWorld(body, FromQuat(r));
				MoveWorld(body, animW + (tgt - animW) * w);
			}
			Refresh(body);
		}
		const V3 bodyPos = body->world.translate;

		bool any = false;
		for (int i = 0; i < rig.count; ++i) {
			const auto& L = rig.legs[i];
			auto*       b = s.bone[i];
			Chain       C;
			float       aq[5][4];
			int         tipC = 0;
			for (int k = 0; k < 5; ++k) {
				if (!b[k]) {
					continue;
				}
				const int ci = C.n++;
				const auto* sn = s.src[i][k] ? s.src[i][k] : b[k];
				if (sn != b[k]) {
					b[k]->local.translate = sn->local.translate;
					b[k]->local.scale = sn->local.scale;
				}
				C.node[ci] = b[k];
				C.lt[ci] = sn->local.translate;
				ToQuat(sn->local.rotate, aq[ci]);
				C.lr[ci] = FromQuat(L.rest[k]);
				tipC = ci;
			}
			const RE::NiAVObject* par = b[0]->parent;
			C.pr = par->world.rotate;
			C.pt = par->world.translate;
			C.sc = par->world.scale > 0.05f ? par->world.scale : 1.0f;
			Chain V = C;
			for (int ci = 0; ci < V.n; ++ci) {
				V.lr[ci] = FromQuat(aq[ci]);
			}
			V.Fk(0);
			const V3 tipAnim = V.wt[tipC];
			const V3 tipT = p.foot[i] + p.nrm[i] * (rig.tipH * sc);
			const V3 goal = tipAnim + (tipT - tipAnim) * w;
			C.Fk(0);

			const V3  P = C.wt[0], restTip = C.wt[tipC];
			const V3  vr = restTip - P, vg = goal - P;
			const float az0 = std::atan2(vr.Dot(rightB), vr.Dot(fwdB)), az1 = std::atan2(vg.Dot(rightB), vg.Dot(fwdB));
			C.Rot(0, AxisAngle(upB, Clamp(Wrap(az0 - az1), -1.2f, 1.2f)));
			C.Fk(0);

			const V3 ob = V3(P) - V3(bodyPos);
			const V3 out = Norm(ob - upB * ob.Dot(upB), fwdB);
			const V3 pole = Norm(upB + out * 0.3f, upB);
			if (L.hasLow) {
				const V3    off = (Rtilt * (R0 * V3{ L.ankle[0], L.ankle[1], L.ankle[2] })) * sc;
				V3          ankle = goal + off;
				const V3    P1 = C.wt[1];
				const float reach = ((C.wt[2] - P1).Length() + (C.wt[3] - C.wt[2]).Length()) * 0.97f;
				if ((ankle - P1).Length() > reach) {
					const V3    hz = Norm(InPlane(P1 - goal, upB), fwdB);
					const float len = off.Length();
					for (int deg = 2; deg < 80; deg += 2) {
						const float ph = static_cast<float>(deg) * kPi / 180.0f;
						ankle = goal + (upB * std::cos(ph) + hz * std::sin(ph)) * len;
						if ((ankle - P1).Length() <= reach) {
							break;
						}
					}
				}
				Solve2(C, 1, 2, 3, ankle, pole);
				const V3 lo = C.wt[3];
				C.Rot(3, RotBetween(Norm(C.wt[tipC] - lo, upB * -1.0f), Norm(goal - lo, upB * -1.0f)));
				C.Fk(3);
			} else {
				Solve2(C, 1, 2, tipC, goal, pole);
			}

			bool ok = true;
			for (int ci = 0; ci < C.n && ok; ++ci) {
				ok = Finite(C.wt[ci]) && (C.wt[ci] - C.pt).Length() < 400.0f * sc;
			}
			float fin[5][4];
			for (int ci = 0; ci < C.n && ok; ++ci) {
				float ik[4], len = 0;
				ToQuat(C.lr[ci], ik);
				const float sg = (ik[0] * aq[ci][0] + ik[1] * aq[ci][1] + ik[2] * aq[ci][2] + ik[3] * aq[ci][3]) < 0.0f ? -1.0f : 1.0f;
				for (int c = 0; c < 4; ++c) {
					fin[ci][c] = aq[ci][c] + (ik[c] * sg - aq[ci][c]) * w;
					len += fin[ci][c] * fin[ci][c];
				}
				ok = std::isfinite(len) && len > 1e-6f;
				if (ok) {
					for (float& v : fin[ci]) {
						v /= std::sqrt(len);
					}
				}
			}
			if (!ok) {
				static std::atomic_int badLogs{ 0 };
				if (Config::Get().spider.log && badLogs.fetch_add(1) < 30) {
					logger::info("SPIDERBAD {:08X} leg={} rejected: goal=({:.0f},{:.0f},{:.0f}) tipT=({:.0f},{:.0f},{:.0f}) tipAnim=({:.0f},{:.0f},{:.0f}) w={:.2f}", s.id, i,
						goal.x, goal.y, goal.z, tipT.x, tipT.y, tipT.z, tipAnim.x, tipAnim.y, tipAnim.z, w);
				}
				for (int ci = 0; ci < C.n; ++ci) {
					C.node[ci]->local.rotate = FromQuat(aq[ci]);
				}
				continue;
			}
			if (w > 0.99f) {
				const float e = (C.wt[tipC] - tipT).Length();
				if (e > s.errMax.load(std::memory_order_relaxed)) {
					s.errMax.store(e, std::memory_order_relaxed);
				}
			}
			for (int ci = 0; ci < C.n; ++ci) {
				C.node[ci]->local.rotate = FromQuat(fin[ci]);
				std::memcpy(s.wrote[i][ci], fin[ci], sizeof(fin[ci]));
			}
			s.wroteTip[i] = V3{ 0, 0, 0 };
			any = true;
		}
		if (any) {
			Refresh(body);
			const V3 rp = root->world.translate;
			for (int i = 0; i < rig.count; ++i) {
				s.wroteTip[i] = V3(s.bone[i][4]->world.translate) - rp;
			}
			s.haveWrote = true;
			s.posts[a_src ? 1 : 0].fetch_add(1);
		}
	}

	void OnFrame(float a_delta)
	{
		if (!Active()) {
			std::lock_guard lk(g_mutex);
			for (auto& [id, s] : g_table) {
				if (s->rig && s->hooked) {
					std::lock_guard sl(s->m);
					Unhook(*s);
				}
			}
			return;
		}
		if (a_delta <= 0.0f || a_delta > 0.5f) {
			return;
		}
		const auto frame = g_frame.fetch_add(1) + 1;
		g_time += a_delta;
		auto* player = RE::PlayerCharacter::GetSingleton();
		auto* lists = RE::ProcessLists::GetSingleton();
		if (!player || !lists || !player->Get3D(false)) {
			return;
		}
		const auto& c = Config::Get().spider;
		struct Cand
		{
			float                   d;
			std::shared_ptr<Spider> s;
			RE::ActorHandle         h;
		};
		static std::vector<Cand> cands;
		cands.clear();
		const V3    pp = player->GetPosition();
		const float r2 = c.radius * c.radius;
		for (auto& h : lists->highActorHandles) {
			auto  ptr = h.get();
			auto* a = ptr.get();
			if (!a || a == player || !a->Is3DLoaded()) {
				continue;
			}
			const V3    d = a->GetPosition() - pp;
			const float d2 = d.x * d.x + d.y * d.y + d.z * d.z;
			if (d2 > r2) {
				continue;
			}
			if (auto s = Bind(a, frame)) {
				cands.push_back({ std::sqrt(d2), s, h });
			}
		}
		std::sort(cands.begin(), cands.end(), [](const Cand& x, const Cand& y) { return x.d < y.d; });
		if (cands.size() > static_cast<std::size_t>(c.maxSpiders)) {
			for (std::size_t k = c.maxSpiders; k < cands.size(); ++k) {
				auto&           sp = *cands[k].s;
				std::lock_guard lk(sp.m);
				sp.pub.w = 0.0f;
				sp.w = 0.0f;
				sp.init = false;
				sp.haveWrote = false;
				Unhook(sp);
			}
			cands.resize(c.maxSpiders);
		}
		for (auto& cd : cands) {
			auto  ptr = cd.h.get();
			auto* a = ptr.get();
			if (!a) {
				continue;
			}
			World w;
			if (!w.ray.Init(a)) {
				continue;
			}
			CrashLog::Crumb("Spider update");
			ProbeFrame(*cd.s);
			Update(a, *cd.s, w, a_delta, g_time, cd.d);
		}
		if ((frame & 63) == 0) {
			std::lock_guard lk(g_mutex);
			for (auto it = g_table.begin(); it != g_table.end();) {
				if (frame - it->second->seen > 600) {
					if (it->second->rig) {
						std::lock_guard sl(it->second->m);
						Unhook(*it->second);
					}
					it = g_table.erase(it);
				} else {
					++it;
				}
			}
		}
	}

	void OnPresent()
	{
		if (!Active() || !Config::Get().spider.log) {
			return;
		}
		const auto frame = g_frame.load();
		std::vector<std::shared_ptr<Spider>> list;
		{
			std::lock_guard lk(g_mutex);
			for (auto& [id, s] : g_table) {
				if (s->rig && frame - s->seen < 3) {
					list.push_back(s);
				}
			}
		}
		for (auto& sp : list) {
			auto&           s = *sp;
			std::lock_guard lk(s.m);
			if (!s.haveWrote || !s.cur[0]) {
				continue;
			}
			const int   n = s.posts[0].load() + s.posts[1].load();
			const float d = PoseDiff(s);
			s.ovP += d > 3.0f;
			s.diffMax = (std::max)(s.diffMax, d);
			s.seq += std::format("{}{}{}{}{} ", s.cur[0], s.cur[1], static_cast<char>('0' + (std::min)(n, 9)), s.cur[2] ? s.cur[2] : '-', d > 3.0f ? 'V' : 'o');
			s.cur[2] = 0;
			s.cur[0] = 0;
			if (++s.pf >= 40) {
				if (g_time - s.probeT > 2.0f && s.probeLogs < 20) {
					++s.probeLogs;
					s.probeT = g_time;
					logger::info("SPIDERPROBE {:08X} frames={} posts job={} ua={} noPost={} multiPost={} overwritten@frame={} overwritten@render={} (world only {}, renders {}) overwritten@present={} recommits={} maxDiff={:.0f}deg maxWorld={:.1f}u seq(postsBefore,frame,postsAfter,render,present)={}",
						s.id, s.pf, s.pfPosts[0], s.pfPosts[1], s.noPost, s.multi, s.ovF, s.ovR, s.ovRW, s.renders, s.ovP, s.recommits, s.diffMax, s.worldMax, s.seq);
				}
				s.seq.clear();
				s.pf = s.pfPosts[0] = s.pfPosts[1] = s.noPost = s.multi = s.ovF = s.ovP = s.recommits = s.ovR = s.ovRW = s.renders = 0;
				s.diffMax = s.worldMax = 0;
			}
		}
	}

	namespace
	{
		void PreRender()
		{
			if (!Active()) {
				return;
			}
			const auto frame = g_frame.load();
			std::vector<std::shared_ptr<Spider>> list;
			{
				std::lock_guard lk(g_mutex);
				for (auto& [id, s] : g_table) {
					if (s->rig && frame - s->seen < 3) {
						list.push_back(s);
					}
				}
			}
			for (auto& sp : list) {
				auto&           s = *sp;
				std::lock_guard lk(s.m);
				if (!s.haveWrote || !s.root || s.pub.w < 0.99f || frame - s.pub.frame > 4) {
					continue;
				}
				const float d = PoseDiff(s);
				const V3    rp = s.root->world.translate;
				const float sc = s.root->world.scale > 0.05f ? s.root->world.scale : 1.0f;
				float       wd = 0;
				for (int i = 0; i < s.rig->count; ++i) {
					wd = (std::max)(wd, (V3(s.bone[i][4]->world.translate) - rp - s.wroteTip[i]).Length());
				}
				++s.renders;
				s.worldMax = (std::max)(s.worldMax, wd);
				const bool badL = d > 3.0f, badW = !badL && wd > 3.0f * sc;
				s.ovR += badL || badW;
				s.ovRW += badW;
				s.cur[2] = badL ? 'V' : badW ? 'W' : 'o';
				if ((badL || badW) && Config::Get().spider.recommit) {
					for (int i = 0; i < s.rig->count; ++i) {
						int ci = 0;
						for (int k = 0; k < 5; ++k) {
							if (s.bone[i][k]) {
								s.bone[i][k]->local.rotate = FromQuat(s.wrote[i][ci++]);
							}
						}
					}
					Refresh(s.body);
					++s.recommits;
				}
			}
		}

		struct RenderWorldHook
		{
			static void thunk(bool a_unk)
			{
				PreRender();
				Locomotion::PreRender();
				func(a_unk);
				Locomotion::PostRender();
			}
			static inline REL::Relocation<decltype(thunk)> func;
		};
	}

	void Install()
	{
		const auto site = REL::ID{ 36559 }.address() + 0x85E;
		const auto* p = reinterpret_cast<const std::uint8_t*>(site);
		if (p[0] != 0xE8 || site + 5 + *reinterpret_cast<const std::int32_t*>(p + 1) != REL::ID{ 107142 }.address()) {
			logger::error("SpiderLegs: RenderWorld call site not as expected - pre-render pose check NOT installed");
			return;
		}
		RenderWorldHook::func = SKSE::GetTrampoline().write_call<5>(site, RenderWorldHook::thunk);
		logger::info("SpiderLegs: pre-render hook installed");
	}
}
