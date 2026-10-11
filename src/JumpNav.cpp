#include "JumpNav.h"
#include "SpiderClimb.h"
#include "SpiderLegs.h"
#include "Locomotion.h"
#include "ClimbClip.h"
#include "Mantle.h"

#include "Config.h"
#include "PathProbe.h"
#include "Raycast.h"
#include "Tuning.h"
#include <queue>
#include <unordered_set>

namespace JumpNav
{
	namespace
	{
		using Vec = RE::NiPoint3;
		using Raycast::Caster;
		using Raycast::Hit;
		using Raycast::IsActorRef;

		constexpr float kEvalInterval = 0.25f;
		constexpr int   kPlansPerFrame = 2;
		constexpr float kPostLandCheck = 0.4f;

		const char* TriggerName(Trigger a_t)
		{
			switch (a_t) {
			case Trigger::kShortcut: return "shortcut";
			case Trigger::kPursuit:  return "pursuit";
			case Trigger::kFollow:   return "follow";
				case Trigger::kRoute:    return "route";
			default:                 return "none";
			}
		}

		float Len2D(const Vec& a_v) { return std::hypot(a_v.x, a_v.y); }

		struct Caps
		{
			float height = 0, radius = 0;
			float up = 0, down = 0, dist = 0;
			bool  npc = false;
			float steep = 0;
			int   rig = 0;
			bool  jumps = true;
		};

		std::unordered_map<RE::TESRace*, int> g_raceExcluded;

		int RaceFlags(RE::TESRace* a_race)
		{
			if (!a_race) {
				return 3;
			}
			if (auto it = g_raceExcluded.find(a_race); it != g_raceExcluded.end()) {
				return it->second;
			}
			std::string edid = a_race->formEditorID.c_str();
			std::ranges::transform(edid, edid.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
			auto match = [&](const std::vector<std::string>& a_list) {
				for (auto pattern : a_list) {
					std::ranges::transform(pattern, pattern.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
					if (!pattern.empty() && edid.find(pattern) != std::string::npos) {
						return true;
					}
				}
				return false;
			};
			const int flags = (match(Config::Get().jump.excludeRaces) ? 1 : 0) | (match(Config::Get().jump.noRouteRaces) ? 3 : 0) | (match(Config::Get().jump.steepRaces) ? 4 : 0);
			g_raceExcluded[a_race] = flags;
			return flags;
		}

		bool CanJump(RE::Actor* a_actor, Caps& a_caps)
		{
			if (!a_actor || a_actor->IsPlayerRef() || a_actor->IsDead() || a_actor->IsDisabled() || !a_actor->Is3DLoaded() ||
				a_actor->IsBeingRidden() || a_actor->IsInMidair() || a_actor->IsInRagdollState() ||
				a_actor->IsInKillMove() || a_actor->IsStaggering() || a_actor->IsAttacking() || !a_actor->GetCharController()) {
				return false;
			}
			if (RE::NiPointer<RE::Actor> mount; a_actor->GetMount(mount) && mount) {
				return false;
			}
			const auto* st = a_actor->AsActorState();
			if (!st || st->IsFlying() || st->IsSwimming() || st->IsSitting() || st->IsUnconscious() || st->IsBleedingOut() ||
				st->GetKnockState() != RE::KNOCK_STATE_ENUM::kNormal) {
				return false;
			}
			auto* race = a_actor->GetRace();
			const int raceFlags = RaceFlags(race);
			if (raceFlags & 2) {
				return false;
			}
			const auto& cfg = Config::Get().jump;
			a_caps.height = a_actor->GetHeight();
			if (a_caps.height < 20.0f) {
				return false;
			}
			a_caps.radius = std::clamp(a_actor->GetBoundRadius(), 10.0f, 150.0f);
			if (a_caps.height >= cfg.giantHeight) {
				if (auto* root = a_actor->Get3D(false); root && root->GetObjectByName(RE::BSFixedString("NPC L Thigh [RThg]"))) {
					a_caps.rig = 1;
				}
			}
			a_caps.steep = (raceFlags & 4) ? cfg.steepRise : 0.0f;
			a_caps.jumps = !(raceFlags & 1) && a_caps.height <= cfg.maxActorHeight;
			if (!a_caps.jumps) {
				a_caps.up = a_caps.down = a_caps.dist = 0.0f;
			} else if (a_caps.height >= cfg.giantHeight) {
				a_caps.up = a_caps.height * cfg.giantUpPerHeight;
				a_caps.down = cfg.downMax;
				a_caps.dist = std::clamp(a_caps.height * cfg.giantDistPerHeight, cfg.distMin, cfg.giantDistMax);
			} else {
				a_caps.up = std::clamp(a_caps.height * cfg.upPerHeight, cfg.upMin, cfg.upMax);
				a_caps.down = cfg.downMax;
				a_caps.dist = std::clamp(a_caps.height * cfg.distPerHeight, cfg.distMin, cfg.distMax);
			}
			a_caps.npc = Mantle::CanClimb(a_actor);
			return true;
		}

		bool IsRanged(RE::Actor* a_actor)
		{
			for (bool left : { false, true }) {
				auto* obj = a_actor->GetEquippedObject(left);
				if (!obj) {
					continue;
				}
				if (obj->Is(RE::FormType::Spell)) {
					return true;
				}
				if (auto* w = obj->As<RE::TESObjectWEAP>(); w && (w->IsBow() || w->IsCrossbow() || w->IsStaff())) {
					return true;
				}
			}
			return false;
		}

		struct Flight
		{
			RE::ActorHandle handle;
			RE::FormID      id = 0;
			std::string     name;
			Trigger         trigger = Trigger::kNone;
			Vec             start, end;
			Vec             vel;
			float           T = 0, t = 0;
			float           g = 900.0f;
			float           yaw = 0;
			float           maxErr = 0;
			bool            npc = false, pose = false;
			int             rig = 0;
			std::uintptr_t  path = 0;
			int             node = -1;
			float           progress = 0;

			Vec At(float a_t) const
			{
				return { start.x + vel.x * a_t, start.y + vel.y * a_t, start.z + vel.z * a_t - 0.5f * g * a_t * a_t };
			}
		};

		struct Memo
		{
			double     nextEval = 0;
			double     cooldownUntil = 0;
			RE::FormID target = 0;
			float      best = (std::numeric_limits<float>::max)();
			double     bestTime = 0;
			double     nextRejectLog = 0;
			double     postCheckAt = 0;
			Flight     last;
				RE::FormID rTarget = 0;
				float      rBest = (std::numeric_limits<float>::max)();
				double     rBestTime = 0;
				double     routeNext = 0;
			double     nextPfLog = 0;
			bool       forceStuck = false;
		};

		std::vector<Flight>                   g_flights;
		std::unordered_map<RE::FormID, Memo>  g_memo;
		double                                g_time = 0;
		double                                g_nextRefresh = 0;
		double                                g_nextPrune = 30;
		double                                g_nextStatsLog = 60;
		std::size_t                           g_rr = 0;

		struct DebugEntry
		{
			std::vector<Vec> points;
			double           time = 0;
			bool             launched = false;
			std::string      label;
		};
		std::deque<DebugEntry> g_debug;

		void RecordDebug(std::vector<Vec> a_points, bool a_launched, std::string a_label)
		{
			g_debug.push_back({ std::move(a_points), g_time, a_launched, std::move(a_label) });
			while (g_debug.size() > 80) {
				g_debug.pop_front();
			}
		}

		std::mutex g_statsLock;
		Stats      g_stats;
		Stats      g_loggedStats;

		template <class F>
		void Count(F&& a_f)
		{
			std::scoped_lock l(g_statsLock);
			a_f(g_stats);
		}

		enum class Reject
		{
			kOk,
			kRange,
			kProfile,
			kRise,
			kSweep,
			kLanding,
			kOccupied,
			kNoWorld,
		};

		const char* RejectName(Reject a_r)
		{
			switch (a_r) {
			case Reject::kOk:       return "ok";
			case Reject::kRange:    return "range";
			case Reject::kProfile:  return "profile";
			case Reject::kRise:     return "rise";
			case Reject::kSweep:    return "sweep";
			case Reject::kLanding:  return "landing";
			case Reject::kOccupied: return "occupied";
			default:                return "no-world";
			}
		}

		void CountReject(Reject a_r)
		{
			Count([&](Stats& s) {
				switch (a_r) {
				case Reject::kRange:    ++s.rejRange; break;
				case Reject::kProfile:  ++s.rejProfile; break;
				case Reject::kRise:     ++s.rejRise; break;
				case Reject::kSweep:    ++s.rejSweep; break;
				case Reject::kLanding:
				case Reject::kNoWorld:  ++s.rejLanding; break;
				case Reject::kOccupied: ++s.rejOccupied; break;
				default:                break;
				}
			});
		}

		struct Plan
		{
			Vec   start, end;
			float T = 0, v0 = 0, rise = 0;
		};

		bool ActorNear(RE::Actor* a_self, RE::TESObjectREFR* a_ignore, const Vec& a_p, float a_r)
		{
			auto* lists = RE::ProcessLists::GetSingleton();
			for (auto& h : lists->highActorHandles) {
				auto  ptr = h.get();
				auto* a = ptr.get();
				if (!a || a == a_self || a == a_ignore || a->IsDead()) {
					continue;
				}
				const auto d = a->GetPosition() - a_p;
				if (std::fabs(d.z) < 150.0f && Len2D(d) < a_r + a->GetBoundRadius()) {
					return true;
				}
			}
			if (auto* player = RE::PlayerCharacter::GetSingleton(); player && player != a_ignore) {
				const auto d = player->GetPosition() - a_p;
				if (std::fabs(d.z) < 150.0f && Len2D(d) < a_r + 30.0f) {
					return true;
				}
			}
			return false;
		}

		Reject PlanJump(RE::Actor* a_actor, const Caps& a_caps, const Vec& a_start, Vec a_end, RE::TESObjectREFR* a_ignore, Plan& a_out)
		{
			const auto& cfg = Config::Get().jump;
			Count([](Stats& s) { ++s.evals; });
			Caster ray;
			if (!ray.Init(a_actor)) {
				return Reject::kNoWorld;
			}

			Hit hit;
			if (!ray.Cast(a_end + Vec{ 0, 0, 60 }, a_end - Vec{ 0, 0, 120 }, hit)) {
				return Reject::kLanding;
			}
			if (IsActorRef(hit.ref)) {
				return Reject::kOccupied;
			}
			a_end.z = hit.point.z;
			if (ActorNear(a_actor, a_ignore, a_end, a_caps.radius)) {
				return Reject::kOccupied;
			}

			const Vec   d = a_end - a_start;
			const float D = Len2D(d);
			const float dz = d.z;
			if (D > a_caps.dist || dz > a_caps.up || -dz > a_caps.down || D < 20.0f) {
				return Reject::kRange;
			}

			const int   n = std::clamp(static_cast<int>(D / 35.0f), 4, 14);
			const float zHigh = (std::max)(a_start.z, a_end.z) + a_caps.up + a_caps.height;
			const float zLow = (std::min)(a_start.z, a_end.z) - 40.0f;
			std::array<float, 16> us{}, tops{};
			for (int i = 0; i < n; ++i) {
				const float u = (i + 1.0f) / (n + 1.0f);
				Vec         from{ a_start.x + d.x * u, a_start.y + d.y * u, zHigh };
				const Vec   to{ from.x, from.y, zLow };
				float       top = zLow;
				for (int tries = 0; tries < 3; ++tries) {
					if (!ray.Cast(from, to, hit)) {
						break;
					}
					if (IsActorRef(hit.ref)) {
						if (hit.ref != a_ignore) {
							return Reject::kOccupied;
						}
						from.z = hit.point.z - 2.0f;
						continue;
					}
					top = hit.point.z;
					break;
				}
				us[i] = u;
				tops[i] = top;
			}

			float speed = 0.0f;
			{
				Vec v;
				a_actor->GetLinearVelocity(v);
				speed = Len2D(v);
			}
			const float g = cfg.gravity;
			const float vmax = std::clamp(speed * 1.15f, cfg.minSpeed, cfg.maxSpeed);
			const float tMin = (std::max)(D / vmax, 0.25f);
			const float tMax = (std::max)(tMin, 1.6f);
			const float h = a_caps.height;
			Reject      why = Reject::kRise;
			int         sweeps = 0;
			for (float T = tMin; T <= tMax; T += 0.04f) {
				const float v0 = (dz + 0.5f * g * T * T) / T;
				if (v0 - g * T > -20.0f) {
					continue;
				}
				const float rise = v0 > 0 ? v0 * v0 / (2.0f * g) : 0.0f;
				if (rise > a_caps.up + 5.0f) {
					break;
				}
				bool clear = true;
				for (int i = 0; i < n && clear; ++i) {
					const float u = us[i];
					const float t = u * T;
					const float z = a_start.z + v0 * t - 0.5f * g * t * t;
					const float margin = cfg.clearance * (std::min)(1.0f, (std::min)(u, 1.0f - u) / 0.15f);
					clear = z >= tops[i] + margin;
				}
				if (!clear) {
					why = Reject::kProfile;
					continue;
				}

				Flight f;
				f.start = a_start;
				f.vel = { d.x / T, d.y / T, v0 };
				f.g = g;
				const int   k = std::clamp(static_cast<int>(T / 0.05f), 6, 16);
				const Vec   dir{ d.x / D, d.y / D, 0.0f };
				const Vec   side{ -dir.y * a_caps.radius * 0.8f, dir.x * a_caps.radius * 0.8f, 0.0f };
				const float heights[3] = { (std::min)(18.0f, h * 0.25f), h * 0.5f, (std::max)(h - 8.0f, h * 0.6f) };
				bool        swept = true;
				Vec         prev = a_start;
				for (int s = 1; s <= k && swept; ++s) {
					const Vec cur = s == k ? a_end : f.At(T * s / k);
					auto      blocked = [&](const Vec& a, const Vec& b) {
                        return ray.Cast(a, b, hit) && !(a_ignore && hit.ref == a_ignore);
					};
					for (float hh : heights) {
						if (blocked(prev + Vec{ 0, 0, hh }, cur + Vec{ 0, 0, hh })) {
							swept = false;
							break;
						}
					}
					const Vec mid{ 0, 0, h * 0.5f };
					if (swept && (blocked(prev + mid + side, cur + mid + side) || blocked(prev + mid - side, cur + mid - side))) {
						swept = false;
					}
					prev = cur;
				}
				if (!swept) {
					why = Reject::kSweep;
					if (++sweeps >= 3) {
						return why;
					}
					continue;
				}

				a_out = { a_start, a_end, T, v0, rise };
				return Reject::kOk;
			}
			return why;
		}

		void SendEvent(RE::Actor* a_actor, const std::string& a_event, bool& a_accepted)
		{
			a_accepted = false;
			if (!a_event.empty()) {
				a_accepted = a_actor->NotifyAnimationGraph(RE::BSFixedString(a_event.c_str()));
			}
		}

		void Launch(RE::Actor* a_actor, const Caps& a_caps, const Plan& a_plan, Trigger a_trigger, float a_saving,
			std::uintptr_t a_path = 0, int a_node = -1, float a_progress = 0)
		{
			const auto& cfg = Config::Get().jump;
			Flight      f;
			f.handle = a_actor->GetHandle();
			f.id = a_actor->GetFormID();
			auto* n = a_actor->GetName();
			f.name = n && *n ? n : "<unnamed>";
			f.trigger = a_trigger;
			f.start = a_plan.start;
			f.end = a_plan.end;
			const Vec d = a_plan.end - a_plan.start;
			f.vel = { d.x / a_plan.T, d.y / a_plan.T, a_plan.v0 };
			f.T = a_plan.T;
			f.g = cfg.gravity;
			f.yaw = std::atan2(d.x, d.y);
			f.npc = a_caps.npc;
			f.path = a_path;
			f.node = a_node;
			f.progress = a_progress;

			{
				std::vector<Vec> pts;
				for (int i = 0; i <= 20; ++i) {
					pts.push_back(f.At(f.T * i / 20.0f));
				}
				RecordDebug(std::move(pts), true, std::format("{} {:.0f}u dz{:+.0f}", TriggerName(a_trigger), Len2D(d), d.z));
			}
			bool anim = false;
			SendEvent(a_actor, a_caps.npc ? cfg.npcStartEvent : cfg.creatureStartEvent, anim);
			f.rig = a_caps.rig;
			f.pose = !anim && (a_caps.npc || a_caps.rig == 1) && Locomotion::Settings().enabled && Config::Get().anim.jumpPose && Config::Get().anim.climb;
			if (cfg.log) {
				logger::info("[{}] JUMP {:08X} '{}' {} D={:.0f} dz={:+.0f} T={:.2f}s rise={:.0f} v=({:.0f},{:.0f}) saving={:.0f} "
							 "caps(h={:.0f} up={:.0f} dist={:.0f}) node={} progress={:.2f} anim={}",
					Tuning::ModeLabel(), f.id, f.name, TriggerName(a_trigger), Len2D(d), d.z, f.T, a_plan.rise,
					Len2D(f.vel), f.vel.z, a_saving, a_caps.height, a_caps.up, a_caps.dist, a_node, a_progress,
					anim ? "yes" : "no");
			}
			auto& m = g_memo[f.id];
			m.cooldownUntil = g_time + f.T + cfg.cooldown;
			m.best = (std::numeric_limits<float>::max)();
			m.bestTime = g_time;
			Count([&](Stats& s) { ++s.launched[static_cast<int>(a_trigger)]; });
			g_flights.push_back(std::move(f));
		}

		float g_dt = 0.016f;

		float WrapPi(float a)
		{
			while (a > 3.14159265f) a -= 6.2831853f;
			while (a < -3.14159265f) a += 6.2831853f;
			return a;
		}

		float TurnToward(RE::Actor* a_actor, float a_yaw, float a_rateDeg = -1.0f)
		{
			const float cur = a_actor->GetHeading(false);
			float       rate = a_rateDeg > 0.0f ? a_rateDeg : Config::Get().jump.turnRateDeg;
			if (const float sp = Config::Get().spider.maxTurnDeg; sp > 0.0f && SpiderLegs::Has(a_actor->GetFormID())) {
				rate = (std::min)(rate, sp);
			}
			const float lim = rate * 0.0174533f * g_dt;
			return cur + (std::clamp)(WrapPi(a_yaw - cur), -lim, lim);
		}

		struct Motion
		{
			float run, rotStand, rotMove;
		};

		Motion RaceMotion(RE::Actor* a_actor)
		{
			const auto& cfg = Config::Get().jump;
			Motion      m{ cfg.glideSpeed, cfg.turnRateDeg, cfg.turnRateDeg };
			if (!cfg.raceMotion) {
				return m;
			}
			auto* proc = a_actor->GetActorRuntimeData().currentProcess;
			auto* hi = proc ? proc->high : nullptr;
			if (!hi) {
				return m;
			}
			using namespace RE::Movement;
			const auto& d = hi->currentMovementType.defaultData;
			constexpr float kDeg = 57.29578f;
			const float     run = d.speeds[SPEED_DIRECTIONS::kForward][MaxSpeeds::kRun];
			const float     stand = d.speeds[SPEED_DIRECTIONS::kRotations][MaxSpeeds::kRun] * kDeg * cfg.driveTurnScale;
			const float     move = d.rotateWhileMovingRun * kDeg * cfg.driveTurnScale;
			if (run > 50.0f && run < 1500.0f) {
				m.run = run;
			}
			if (stand > 20.0f) {
				m.rotStand = stand;
			}
			if (move > 20.0f) {
				m.rotMove = (std::min)(move, m.rotStand);
			}
			return m;
		}

		void Drive(RE::Actor* a_actor, float a_yaw, const Vec& a_pos, const Vec& a_vel, bool a_inAir)
		{
			a_actor->SetPosition(a_pos, true);
			a_actor->SetHeading(TurnToward(a_actor, a_yaw));
			if (Config::Get().jump.driveMode != 0) {
				return;
			}
			if (auto* cc = a_actor->GetCharController()) {
				const float s = RE::bhkWorld::GetWorldScale();
				if (a_inAir) {
					cc->context.currentState = RE::hkpCharacterStateType::kInAir;
				}
				cc->SetLinearVelocityImpl(RE::hkVector4(a_vel.x * s, a_vel.y * s, a_vel.z * s, 0.0f));
				cc->fallStartHeight = a_pos.z;
				cc->fallTime = 0.0f;
			}
		}

		struct LandPose
		{
			RE::FormID id;
			double     start;
			int        rig;
		};
		std::vector<LandPose> g_landPoses;

		void UpdateLandPoses()
		{
			constexpr float kDur = 0.8f, kFade = 0.3f;
			for (auto it = g_landPoses.begin(); it != g_landPoses.end();) {
				const float t = static_cast<float>(g_time - it->start);
				auto*       actor = RE::TESForm::LookupByID<RE::Actor>(it->id);
				if (t >= kDur || !actor || !actor->Is3DLoaded() || Mantle::Active(it->id) || std::ranges::any_of(g_flights, [&](const Flight& f) { return f.id == it->id; })) {
					it = g_landPoses.erase(it);
					continue;
				}
				Locomotion::ClimbPose cp;
				cp.clip = ClimbClip::kJumpLand;
				cp.rig = it->rig;
				cp.t = t;
				cp.w = t < kDur - kFade ? 1.0f : (kDur - t) / kFade;
				Locomotion::SetClimb(it->id, &cp);
				++it;
			}
		}

		void UpdateFlights(float a_dt)
		{
			const auto& cfg = Config::Get().jump;
			for (auto it = g_flights.begin(); it != g_flights.end();) {
				auto& f = *it;
				auto  ptr = f.handle.get();
				auto* actor = ptr.get();
				const bool lost = !actor || actor->IsDead() || !actor->Is3DLoaded() || actor->IsInRagdollState() ||
				                  (actor->AsActorState() && actor->AsActorState()->GetKnockState() != RE::KNOCK_STATE_ENUM::kNormal);
				if (lost) {
					if (cfg.log) {
						logger::info("[{}] JUMPABORT {:08X} '{}' at t={:.2f}/{:.2f}s", Tuning::ModeLabel(), f.id, f.name, f.t, f.T);
					}
					Count([](Stats& s) { ++s.aborted; });
					it = g_flights.erase(it);
					continue;
				}

				const float next = (std::min)(f.t + a_dt, f.T);
				f.maxErr = (std::max)(f.maxErr, actor->GetPosition().GetDistance(f.At(next)));
				f.t = next;

				if (f.pose && !Mantle::Active(f.id)) {
					Locomotion::ClimbPose cp;
					cp.clip = ClimbClip::kJumpStart;
					cp.rig = f.rig;
					cp.w = (std::min)(1.0f, f.t / 0.1f);
					cp.t = f.t / f.T * std::clamp(f.T * 1.6f, 0.45f, 1.07f);
					Locomotion::SetClimb(f.id, &cp);
				}
				if (f.t < f.T) {
					Drive(actor, f.yaw, f.At(f.t), { f.vel.x, f.vel.y, f.vel.z - f.g * f.t }, true);
					++it;
					continue;
				}

				const float endErr = actor->GetPosition().GetDistance(f.end);
				Drive(actor, f.yaw, f.end, { f.vel.x, f.vel.y, 0.0f }, false);
				bool anim = false;
				SendEvent(actor, f.npc ? cfg.npcLandEvent : cfg.creatureLandEvent, anim);
				if (cfg.log) {
					logger::info("[{}] LAND {:08X} '{}' {} maxDrift={:.1f} endDrift={:.1f} anim={}", Tuning::ModeLabel(), f.id,
						f.name, TriggerName(f.trigger), f.maxErr, endErr, anim ? "yes" : "no");
				}
				Count([](Stats& s) { ++s.landed; });
				if (f.pose) {
					g_landPoses.push_back({ f.id, g_time, f.rig });
				}
				auto& m = g_memo[f.id];
				m.postCheckAt = g_time + kPostLandCheck;
				m.last = f;
				it = g_flights.erase(it);
			}
		}

		void PostLandChecks()
		{
			const auto& cfg = Config::Get().jump;
			for (auto& [id, m] : g_memo) {
				if (m.postCheckAt == 0 || g_time < m.postCheckAt) {
					continue;
				}
				m.postCheckAt = 0;
				auto* actor = RE::TESForm::LookupByID<RE::Actor>(id);
				if (!actor || !actor->Is3DLoaded()) {
					continue;
				}
				const bool stuck = actor->IsInJumpState();
				if (stuck) {
					bool anim = false;
					SendEvent(actor, m.last.npc ? cfg.npcLandEvent : cfg.creatureLandEvent, anim);
				}
				std::string pathInfo;
				if (m.last.trigger == Trigger::kShortcut) {
					if (auto pr = PathProbe::GetPathRef(actor)) {
						pathInfo = std::format(" samePath={} progress {:.2f} -> {:.2f} (landing node {})",
							pr->path == m.last.path ? "yes" : "no", m.last.progress, pr->progress, m.last.node);
					} else {
						pathInfo = " no path";
					}
				}
				if (cfg.log) {
					logger::info("[{}] AFTERLAND {:08X} '{}' jumpState={} dist={:.0f} from landing{}", Tuning::ModeLabel(), id,
						m.last.name, stuck ? "STUCK" : "no", actor->GetPosition().GetDistance(m.last.end), pathInfo);
				}
			}
		}

		struct RNode
		{
			Vec   p;
			float g = 0;
			int   parent = -1;
				bool  jump = false, climb = false, hang = false;
			float cdx = 0, cdy = 0;
		};

		struct Search
		{
			RE::ActorHandle handle;
			RE::FormID      id = 0, goalId = 0;
			std::string     name;
			Vec             start, goal;
			Caps            caps;
			float           gap = 100, waterZ = -1e30f, vanilla = 0;
			bool            stuck = false, melee = false;
			int             whyLogged = 0, expanded = 0, goalNode = -1, best = 0, maxNodes = 160;
			std::vector<RNode> nodes;
			std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> open;
			std::unordered_set<std::uint64_t> seen;
			float                             base = 0;
			mutable std::unordered_map<std::uint64_t, std::pair<bool, Vec>> gcache;
		};

		struct Pt
		{
			Vec  p;
				bool jump, climb, hang;
			float cdx = 0, cdy = 0;
		};

		struct Glide
		{
			RE::ActorHandle handle;
			RE::FormID      id = 0, goalId = 0;
			std::string     name;
			std::vector<Pt> pts;
			std::size_t     i = 1;
			Vec             goal;
			Caps            caps;
			double          until = 0;
			float           len = 0, gap = 0, speed = 150.0f, blocked = 0;
			bool            warned = false, logged = false;
				float           climbWait = 0;
			double          holdUntil = 0;
			double          slowLog = 0;
			std::size_t     wpIdx = 0;
			float           wpBest = 1e9f;
			double          wpTime = 0;
		};

		std::vector<Search> g_searches;
		std::vector<Glide>  g_glides;
		std::size_t         g_searchRr = 0;

		Glide* FindGlide(RE::FormID a_id)
		{
			auto it = std::ranges::find_if(g_glides, [&](const Glide& g) { return g.id == a_id; });
			return it == g_glides.end() ? nullptr : &*it;
		}

		bool Searching(RE::FormID a_id)
		{
			return std::ranges::any_of(g_searches, [&](const Search& s) { return s.id == a_id; });
		}

		std::uint64_t Key(const Vec& p)
		{
			auto c = [](float v, float s) { return static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(v / s)) & 0x1FFFFF); };
			return c(p.x, 48) << 42 | c(p.y, 48) << 21 | c(p.z, 60);
		}

		bool Ground(const Caster& r, float x, float y, float zTop, float zBot, Vec& out)
		{
			Hit h;
			if (!r.Cast({ x, y, zTop }, { x, y, zBot }, h) || IsActorRef(h.ref)) {
				return false;
			}
			out = { x, y, h.point.z };
			return true;
		}

		bool Headroom(const Caster& r, const Vec& p, float h)
		{
			Hit k;
			return !r.Cast(p + Vec{ 0, 0, 8 }, p + Vec{ 0, 0, h * 0.85f }, k);
		}

		bool GroundC(const Search& s, const Caster& r, float x, float y, float zTop, float zBot, Vec& out)
		{
			auto q = [](float v, float u) { return static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(v / u)) & 0x1FFFFF); };
			const std::uint64_t key = q(x, 6) << 43 | q(y, 6) << 22 | (q(zTop, 30) & 0x7FF) << 11 | (q(zBot, 30) & 0x7FF);
			if (auto it = s.gcache.find(key); it != s.gcache.end()) {
				out = it->second.second;
				return it->second.first;
			}
			const bool ok = Ground(r, x, y, zTop, zBot, out);
			s.gcache.emplace(key, std::make_pair(ok, out));
			return ok;
		}

		float WadeMax(float a_height) { return (std::max)(10.0f, a_height * Config::Get().jump.wadeDepth); }

		bool WalkEdge(const Search& s, const Caster& r, const Vec& p, float dx, float dy, float len, Vec& q)
		{
			const float h = s.caps.height, knee = (std::max)(35.0f, h * 0.35f);
			const float maxRise = (std::max)(45.0f, s.caps.steep);
			if (!GroundC(s, r, p.x + dx * len, p.y + dy * len, p.z + maxRise + 15, p.z - 70, q) || q.z - p.z > maxRise || q.z < s.waterZ - WadeMax(s.caps.height)) {
				return false;
			}
			Vec m;
			if (!GroundC(s, r, p.x + dx * len * 0.5f, p.y + dy * len * 0.5f, p.z + maxRise + 15, p.z - 70, m) ||
				std::fabs(m.z - (p.z + q.z) * 0.5f) > 40) {
				return false;
			}
			auto blocked = [&](const Vec& a, const Vec& b) {
				Hit  k;
				Vec  from = a;
				for (int n = 0; n < 4; ++n) {
					if (!r.Cast(from, b, k)) {
						return false;
					}
					if (!IsActorRef(k.ref)) {
						return true;
					}
					const Vec  d = b - from;
					const float len = d.Length();
					if (len < 3.0f) {
						return false;
					}
					from = k.point + d * (6.0f / len);
				}
				return false;
			};
			return !blocked(p + Vec{ 0, 0, knee }, q + Vec{ 0, 0, knee }) && !blocked(p + Vec{ 0, 0, h * 0.8f }, q + Vec{ 0, 0, h * 0.8f }) && Headroom(r, q, h) && Headroom(r, m, h);
		}

		bool JumpEdge(const Search& s, const Caster& r, const Vec& p, float dx, float dy, float d, Vec& q)
		{
			const auto& c = s.caps;
			if (!GroundC(s, r, p.x + dx * d, p.y + dy * d, p.z + c.up + 15, p.z - c.down, q)) {
				return false;
			}
			const float dz = q.z - p.z;
			if (dz > c.up || q.z < s.waterZ - 10 || !Headroom(r, q, c.height)) {
				return false;
			}
			const float H = 40 + 0.4f * (std::max)(dz, 0.0f);
			for (float u : { 0.5f, 0.25f, 0.75f }) {
				Vec t;
				if (Ground(r, p.x + dx * d * u, p.y + dy * d * u, p.z + c.up + c.height, (std::min)(p.z, q.z) - 40, t) &&
					p.z + dz * u + 4 * H * u * (1 - u) < t.z + Config::Get().jump.clearance) {
					return false;
				}
			}
			return true;
		}

		std::string WalkWhy(const Search& s, const Caster& r, const Vec& p, float dx, float dy, float len)
		{
			const float h = s.caps.height, knee = (std::max)(35.0f, h * 0.35f), maxRise = (std::max)(45.0f, s.caps.steep);
			auto        who = [](const Hit& k) {
                auto* n = k.ref ? k.ref->GetName() : nullptr;
                return std::format("layer={} ref={}", static_cast<int>(k.layer), n && *n ? n : (k.ref ? "<ref>" : "-"));
			};
			Hit q3, m3, k;
			const Vec top = { p.x + dx * len, p.y + dy * len, 0 };
			if (!r.Cast({ top.x, top.y, p.z + maxRise + 15 }, { top.x, top.y, p.z - 70 }, q3)) {
				return "noGround";
			}
			if (IsActorRef(q3.ref)) {
				return "groundIsActor " + who(q3);
			}
			if (q3.point.z - p.z > maxRise) {
				return std::format("rise {:.0f}>{:.0f}", q3.point.z - p.z, maxRise);
			}
			if (q3.point.z < s.waterZ - WadeMax(h)) {
				return "deepWater";
			}
			if (!r.Cast({ p.x + dx * len * 0.5f, p.y + dy * len * 0.5f, p.z + maxRise + 15 }, { p.x + dx * len * 0.5f, p.y + dy * len * 0.5f, p.z - 70 }, m3) ||
				IsActorRef(m3.ref) || std::fabs(m3.point.z - (p.z + q3.point.z) * 0.5f) > 40) {
				return std::format("midpoint dev {:.0f}", m3.point.z - (p.z + q3.point.z) * 0.5f);
			}
			const Vec q = q3.point;
			if (r.Cast(p + Vec{ 0, 0, knee }, q + Vec{ 0, 0, knee }, k)) {
				return "kneeRay " + who(k) + std::format(" at {:.0f}", (k.point - p).Length());
			}
			if (r.Cast(p + Vec{ 0, 0, h * 0.8f }, q + Vec{ 0, 0, h * 0.8f }, k)) {
				return "chestRay " + who(k) + std::format(" at {:.0f}", (k.point - p).Length());
			}
			if (!Headroom(r, q, h)) {
				return "headroomEnd";
			}
			return "headroomMid";
		}

		std::unordered_map<RE::FormID, std::pair<double, std::vector<std::uint64_t>>> g_bad;

		void MarkBad(RE::FormID a_id, const Vec& p)
		{
			auto& b = g_bad[a_id];
			if (g_time > b.first) {
				b.second.clear();
			}
			b.first = g_time + 30.0;
			if (b.second.size() < 40) {
				b.second.push_back(Key(p));
			}
		}

		bool IsBad(RE::FormID a_id, std::uint64_t a_key)
		{
			auto it = g_bad.find(a_id);
			return it != g_bad.end() && g_time <= it->second.first && std::ranges::find(it->second.second, a_key) != it->second.second.end();
		}

		void Push(Search& s, int a_parent, const Vec& q, float a_cost, bool a_jump, bool a_climb = false, bool a_hang = false, float a_dx = 0, float a_dy = 0)
		{
			if (IsBad(s.id, Key(q)) || !s.seen.insert(Key(q)).second) {
				return;
			}
			const int idx = static_cast<int>(s.nodes.size());
			s.nodes.push_back({ q, s.nodes[a_parent].g + a_cost, a_parent, a_jump, a_climb, a_hang, a_dx, a_dy });
			s.open.push({ s.nodes[idx].g + 1.2f * (s.goal - q).Length(), idx });
			if ((s.goal - q).Length() < (s.goal - s.nodes[s.best].p).Length()) {
				s.best = idx;
			}
		}

		constexpr int kRaysPerFrame = 500;

		bool Step(Search& s, RE::Actor* a_actor)
		{
			Caster r;
			if (!r.Init(a_actor)) {
				return true;
			}
			const auto& cfg = Config::Get().jump;
			for (int n = 0; n < 24 && (n == 0 || r.used < kRaysPerFrame); ++n) {
				if (s.open.empty() || s.expanded >= s.maxNodes) {
					return true;
				}
				const int i = s.open.top().second;
				s.open.pop();
				++s.expanded;
				const RNode cur = s.nodes[i];
				const Vec   tg = s.goal - cur.p;
				const float hh = s.caps.height;
				if (Len2D(tg) <= s.gap && (s.melee ? tg.z <= Config::Get().combat.swingRise * hh && tg.z >= -0.9f * hh : std::fabs(tg.z) <= 100)) {
					s.goalNode = i;
					return true;
				}
				bool walked[8]{};
				for (int k = 0; k < 8; ++k) {
					const float ang = s.base + k * 0.785398f, dx = std::cos(ang), dy = std::sin(ang);
					Vec         q;
					if (WalkEdge(s, r, cur.p, dx, dy, cfg.routeStep * (k & 1 ? 1.41421f : 1.0f), q)) {
						walked[k] = true;
						Push(s, i, q, cur.p.GetDistance(q) * (q.z - cur.p.z > 45.0f ? cfg.steepCost : 1.0f) * (q.z < s.waterZ - 10.0f ? cfg.wadeCost : 1.0f), false);
					}
				}
				if (walked[0] || walked[1] || walked[7]) {
					continue;
				}
				for (int k : { 0, 7, 1, 6, 2 }) {
					const float ang = s.base + k * 0.785398f, dx = std::cos(ang), dy = std::sin(ang);
					Vec         q;
					Mantle::Plan pl;
					if (s.caps.npc && Mantle::PlanClimb(r, cur.p, { dx, dy, 0.0f }, s.caps.height, 70.0f, false, pl)) {
						Push(s, i, pl.stand, 1.2f * cur.p.GetDistance(pl.stand) + 40.0f, false, true, false, dx, dy);
						continue;
					}
					if (s.caps.npc && s.caps.jumps && s.caps.up > 60.0f && Mantle::PlanClimb(r, cur.p, { dx, dy, 0.0f }, s.caps.height, 70.0f, true, pl, (std::min)(s.caps.up, 85.0f))) {
						Push(s, i, pl.stand, 1.3f * cur.p.GetDistance(pl.stand) + 120.0f, false, true, true, dx, dy);
						continue;
					}
					for (float d : { 110.0f, 180.0f, s.caps.dist * 0.85f }) {
						if (s.caps.jumps && JumpEdge(s, r, cur.p, dx, dy, d, q)) {
							if (cfg.log && s.whyLogged < 3 && !walked[0]) {
								++s.whyLogged;
								logger::info("[{}] ROUTEWHY {:08X} '{}' node={} atz={:.0f} toGoal={:.0f} dz={:+.0f} straight={} dir={} d={:.0f} walked={}{}{}{}{}{}{}{}",
									Tuning::ModeLabel(), s.id, s.name, i, cur.p.z, Len2D(s.goal - cur.p), s.goal.z - cur.p.z,
									WalkWhy(s, r, cur.p, std::cos(s.base), std::sin(s.base), cfg.routeStep), k, d,
									walked[0], walked[1], walked[2], walked[3], walked[4], walked[5], walked[6], walked[7]);
							}
							Push(s, i, q, 1.3f * cur.p.GetDistance(q) + 80.0f, true);
							break;
						}
					}
				}
			}
			return false;
		}

		void FinishSearch(Search& s, RE::Actor* a_actor)
		{
			const auto& cfg = Config::Get().jump;
			const bool  reached = s.goalNode >= 0;
			const int   end = reached ? s.goalNode : s.best;
			const float gain = Len2D(s.goal - s.start) - Len2D(s.goal - s.nodes[end].p);
			const float gain3 = (s.goal - s.start).Length() - (s.goal - s.nodes[end].p).Length();
			const bool  ok = end > 0 && (reached ? (s.vanilla <= 0 || s.nodes[end].g * 1.25f < s.vanilla) : (s.stuck && (gain > 150.0f || gain3 > 150.0f)));
			auto&       m = g_memo[s.id];
			m.routeNext = g_time + (ok ? 0.3 : 1.5);
			if (a_actor->GetPosition().GetDistance(s.start) > 400.0f) {
				return;
			}
			if (!ok) {
				Count([](Stats& st) { ++st.routesFailed; });
				if (cfg.log) {
					logger::info("[{}] ROUTEFAIL {:08X} '{}' why={} expanded={} reached={} gain={:.0f} cost={:.0f} vanilla={:.0f}",
						Tuning::ModeLabel(), s.id, s.name, s.stuck ? "stuck" : "detour", s.expanded, reached, gain, s.nodes[end].g, s.vanilla);
				}
				return;
			}
			std::erase_if(g_glides, [&](const Glide& o) { return o.id == s.id; });
			Glide g;
			g.handle = s.handle;
			g.id = s.id;
			g.goalId = s.goalId;
			g.name = s.name;
			g.goal = s.goal;
			g.gap = s.gap;
			g.caps = s.caps;
			for (int i = end; i >= 0; i = s.nodes[i].parent) {
				g.pts.push_back({ s.nodes[i].p, s.nodes[i].jump, s.nodes[i].climb, s.nodes[i].hang, s.nodes[i].cdx, s.nodes[i].cdy });
			}
			std::ranges::reverse(g.pts);
			g.len = s.nodes[end].g;
			g.until = g_time + g.len / (std::min)(cfg.glideSpeed, 200.0f) * 2.0 + 8.0;
			int jumps = 0, climbs = 0;
			std::vector<Vec> pts;
			for (auto& p : g.pts) {
				jumps += p.jump;
					climbs += p.climb;
				pts.push_back(p.p);
			}
			Count([](Stats& st) { ++st.routes; });
			if (cfg.log) {
				logger::info("[{}] ROUTE {:08X} '{}' why={} {} nodes={} jumps={} climbs={} cost={:.0f} vanilla={:.0f} expanded={} goalDist={:.0f}",
					Tuning::ModeLabel(), s.id, s.name, s.stuck ? "stuck" : "detour", reached ? "full" : "partial", g.pts.size(), jumps, climbs,

					g.len, s.vanilla, s.expanded, (s.goal - s.nodes[end].p).Length());
			}
			RecordDebug(std::move(pts), true, std::format("route {} hops {} jumps", g.pts.size() - 1, jumps));
			g_glides.push_back(std::move(g));
		}

		void TickSearches()
		{
			if (g_searches.empty()) {
				return;
			}
			g_searchRr %= g_searches.size();
			auto& s = g_searches[g_searchRr];
			auto  ptr = s.handle.get();
			auto* actor = ptr.get();
			if (!actor || actor->IsDead() || !actor->Is3DLoaded()) {
				g_searches.erase(g_searches.begin() + g_searchRr);
				return;
			}
			if (Step(s, actor)) {
				FinishSearch(s, actor);
				g_searches.erase(g_searches.begin() + g_searchRr);
			} else {
				++g_searchRr;
			}
		}

		void EndGlide(const Glide& g, const char* a_reason, bool a_done)
		{
			if (a_done) {
				Count([](Stats& st) { ++st.routesDone; });
			}
			auto& m = g_memo[g.id];
			m.rBest = (std::numeric_limits<float>::max)();
			m.routeNext = (std::max<double>)(m.routeNext, g_time + (a_done ? 0.5 : 2.0));
			auto* gl = RE::TESForm::LookupByID<RE::TESObjectREFR>(g.goalId);
			if (auto ptr = g.handle.get(); ptr.get() && gl && !std::string_view(a_reason).starts_with("calm") &&
				(Len2D(gl->GetPosition() - ptr->GetPosition()) > g.gap + 60.0f || gl->GetPosition().z - ptr->GetPosition().z > Config::Get().combat.swingRise * g.caps.height)) {
				m.forceStuck = true;
				m.routeNext = g_time + (a_done ? 0.05 : 0.6);
			}
			if (Config::Get().jump.log) {
				auto  ptr = g.handle.get();
				float d = ptr.get() ? ptr->GetPosition().GetDistance(g.goal) : -1.0f;
				logger::info("[{}] ROUTEEND {:08X} '{}' {} i={}/{} goalDist={:.0f}", Tuning::ModeLabel(), g.id, g.name, a_reason, g.i,
					g.pts.size(), d);
			}
		}

		void StartSearch(RE::Actor* a_actor, const Caps& a_caps, RE::TESObjectREFR* a_goal, float a_gap, bool stuck, float vanilla);

		bool BeginHold(Glide& g, RE::Actor* a_actor)
		{
			auto* gl = RE::TESForm::LookupByID<RE::TESObjectREFR>(g.goalId);
			if (!gl || !a_actor->IsInCombat() || g.holdUntil > 0 || g_searches.size() >= 5 || Searching(g.id) ||
				!(Len2D(gl->GetPosition() - a_actor->GetPosition()) > g.gap + 60.0f || gl->GetPosition().z - a_actor->GetPosition().z > Config::Get().combat.swingRise * g.caps.height)) {
				return false;
			}
			g.holdUntil = g_time + 1.5;
			StartSearch(a_actor, g.caps, gl, g.gap - g.caps.radius - 40.0f, true, 0.0f);
			return true;
		}

		void UpdateGlides(float a_dt)
		{
			const auto& cfg = Config::Get().jump;
			for (auto it = g_glides.begin(); it != g_glides.end();) {
				auto& g = *it;
				auto  ptr = g.handle.get();
				auto* actor = ptr.get();
				const char* drop = nullptr;
				if (!actor || actor->IsDead() || !actor->Is3DLoaded() || actor->IsInRagdollState() ||
					(actor->AsActorState() && actor->AsActorState()->GetKnockState() != RE::KNOCK_STATE_ENUM::kNormal)) {
					drop = "lost";
				} else if (SpiderClimb::Active(g.id)) {
					drop = "crawl";
				} else if (g_time > g.until) {
					drop = "timeout";
				} else if (!actor->IsInCombat() && !actor->IsPlayerTeammate()) {
					drop = "calm";
				} else if (!actor->IsInCombat() && Len2D(RE::PlayerCharacter::GetSingleton()->GetPosition() - actor->GetPosition()) < 0.6f * cfg.followRouteMin) {
					drop = "arrived";
				} else if (auto* goal = RE::TESForm::LookupByID<RE::TESObjectREFR>(g.goalId);
						   !goal || goal->GetPosition().GetDistance(g.goal) > 800.0f) {
					drop = "goalmoved";
				}
				if (drop) {
					EndGlide(g, drop, false);
					it = g_glides.erase(it);
					continue;
				}
				if (g.holdUntil > 0) {
					if (g_time > g.holdUntil || !Searching(g.id)) {
						EndGlide(g, "done", true);
						it = g_glides.erase(it);
						continue;
					}
					const Vec cur = actor->GetPosition();
					if (auto* goal = RE::TESForm::LookupByID<RE::TESObjectREFR>(g.goalId)) {
						const Vec to = goal->GetPosition() - cur;
						Drive(actor, TurnToward(actor, std::atan2(to.x, to.y)), cur, { 0, 0, 0 }, false);
					}
					Locomotion::NoteDriven(g.id, 0.0f);
					++it;
					continue;
				}
				if (std::ranges::any_of(g_flights, [&](const Flight& f) { return f.id == g.id; }) || Mantle::Active(g.id)) {
						++it;
						continue;
					}
					const Vec cur = actor->GetPosition();
					if (g.pts[g.i].climb) {
					Vec dir = g.pts[g.i].p - cur;
					dir.z = 0.0f;
					const float dl = Len2D(dir);
					if (dl > 1.0f) {
						dir = dir * (1.0f / dl);
					}
					const Vec  planned{ g.pts[g.i].cdx, g.pts[g.i].cdy, 0.0f };
					const bool hang = g.pts[g.i].hang;
					if ((Len2D(planned) > 0.5f && Mantle::TryStart(actor, hang, &planned, 90.0f, 85.0f)) || (dl > 1.0f && Mantle::TryStart(actor, hang, &dir, 90.0f, 85.0f))) {
						++g.i;
						g.climbWait = 0;
						if (g.i >= g.pts.size() && !BeginHold(g, actor)) {
							EndGlide(g, "done", true);
							it = g_glides.erase(it);
						} else {
							++it;
						}
						continue;
					}
					if ((g.climbWait += a_dt) > 2.0f) {
						MarkBad(g.id, g.pts[g.i].p);
						EndGlide(g, "climbfail", false);
						it = g_glides.erase(it);
					} else {
						++it;
					}
					continue;
				}
				if (g.pts[g.i].jump) {
					Plan       plan;
					const auto r = PlanJump(actor, g.caps, cur, g.pts[g.i].p, RE::TESForm::LookupByID<RE::TESObjectREFR>(g.goalId), plan);
					if (r == Reject::kOccupied && (g.climbWait += a_dt) < 0.8f) {
						++it;
						continue;
					}
					if (r != Reject::kOk) {
						CountReject(r);
						RecordDebug({ cur, g.pts[g.i].p }, false, std::format("route: {}", RejectName(r)));
						MarkBad(g.id, g.pts[g.i].p);
						EndGlide(g, RejectName(r), false);
						it = g_glides.erase(it);
						continue;
					}
					Launch(actor, g.caps, plan, Trigger::kRoute, 0.0f);
					g.climbWait = 0;
					++g.i;
				} else {
					const Motion mv = RaceMotion(actor);
					const Vec    wp = g.pts[g.i].p;
					const Vec    to = wp - cur;
					const float  dh = (std::max)(Len2D(to), 0.01f);
					const std::size_t nx = g.i + 1;
					const bool        nextWalk = nx < g.pts.size() && !g.pts[nx].climb && !g.pts[nx].jump;
					const float       look = std::clamp(g.speed * 0.45f, 50.0f, 180.0f);
					Vec               tgt = wp;
					if (nextWalk && dh < look) {
						Vec seg = g.pts[nx].p - wp;
						seg.z = 0.0f;
						if (const float sl = Len2D(seg); sl > 1.0f) {
							tgt = wp + seg * ((std::min)(look - dh, sl) / sl);
						}
					}
					const Vec   tt = tgt - cur;
					const float yaw = std::atan2(tt.x, tt.y);
					const float hCur = actor->GetHeading(false);
					const float facing = (std::cos)(WrapPi(yaw - hCur));
					float       want = mv.run * std::clamp((facing - 0.1f) / 0.7f, 0.0f, 1.0f);
					const float radiusCap = (std::max)(60.0f, mv.rotMove * 0.01745f * dh * 0.8f);
					want = (std::min)(want, radiusCap + (std::max)(0.0f, mv.run - radiusCap) * std::clamp((facing - 0.8f) / 0.15f, 0.0f, 1.0f));
					const float acc = cfg.glideAccel * mv.run / 400.0f;
					g.speed = want > g.speed ? (std::min)(want, g.speed + acc * a_dt) : (std::max)(want, g.speed - 2.0f * acc * a_dt);
					const float rate = mv.rotStand + (mv.rotMove - mv.rotStand) * std::clamp(g.speed / (0.5f * mv.run), 0.0f, 1.0f);
					const float hNew = TurnToward(actor, yaw, rate);
					const float step = g.speed * a_dt;
					Locomotion::NoteDriven(g.id, g.speed);
					const Vec  fwd{ std::sin(hNew), std::cos(hNew), 0.0f };
					Vec        np{ cur.x + fwd.x * step, cur.y + fwd.y * step, cur.z };
					const bool lastOrSpecial = !nextWalk;
					const Vec  segDir = wp - g.pts[g.i - 1].p;
					const bool passed = dh < 150.0f && segDir.x * to.x + segDir.y * to.y < 0.0f;
					const bool reached = lastOrSpecial ? dh <= (std::max)(step, 25.0f) : (dh <= std::clamp(g.speed * 0.15f, 25.0f, 60.0f) || passed);
					if (g.wpIdx != g.i || dh < g.wpBest - 20.0f) {
						g.wpIdx = g.i;
						g.wpBest = dh;
						g.wpTime = g_time;
					} else if (g_time - g.wpTime > 2.0 && !reached) {
						if (nextWalk) {
							++g.i;
							++it;
							continue;
						}
						MarkBad(g.id, wp);
						EndGlide(g, "orbit", false);
						it = g_glides.erase(it);
						continue;
					}
					if (reached && lastOrSpecial && dh <= step) {
						np = wp;
					}
					Caster r;
					Vec    gp;
					const bool haveRay = r.Init(actor);
					const std::size_t after = g.i + (reached ? 1 : 0);
					const bool        nextSpecial = after < g.pts.size() && (g.pts[after].climb || g.pts[after].jump);
					bool blockedNow = false;
					if (haveRay && !nextSpecial && g.speed > 5.0f) {
						const float len = step + g.caps.radius * 0.7f + 8.0f, h = g.caps.height;
						Hit         k;
						for (float hz : { (std::max)(35.0f, h * 0.35f), h * 0.8f }) {
							if (r.Cast(cur + Vec{ 0, 0, hz }, cur + fwd * len + Vec{ 0, 0, hz }, k) && !Raycast::IsActorRef(k.ref)) {
								np = cur;
								blockedNow = true;
								g.blocked += a_dt;
								if (!g.warned && cfg.log) {
									g.warned = true;
									logger::info("[{}] WALKBLOCK {:08X} '{}' hit '{}' ({:08X}) at z+{:.0f} dist={:.0f}", Tuning::ModeLabel(), g.id, g.name,
										k.ref && k.ref->GetName() ? k.ref->GetName() : "<terrain/none>", k.ref ? k.ref->GetFormID() : 0, hz, cur.GetDistance(k.point));
								}
								break;
							}
							g.blocked = (std::max)(0.0f, g.blocked - a_dt * 2.0f);
						}
						if (g.blocked > 0.5f) {
							MarkBad(g.id, g.pts[g.i].p);
							EndGlide(g, "blocked", false);
							it = g_glides.erase(it);
							continue;
						}
					}
					if (cfg.log && g.speed < 0.4f * mv.run && g_time >= g.slowLog) {
						g.slowLog = g_time + 1.0;
						Vec av;
						actor->GetLinearVelocity(av);
						logger::info("[{}] ROUTESLOW {:08X} '{}' wp={}/{} dh={:.0f} cmd={:.0f} want={:.0f} measured={:.0f} facing={:.2f} radiusCap={:.0f} blockedNow={} blocked={:.2f} "
									 "heading={:.0f} toWp={:.0f} turnRate={:.0f} nextWalk={}",
							Tuning::ModeLabel(), g.id, g.name, g.i, g.pts.size(), dh, g.speed, want, Len2D(av), facing, radiusCap, blockedNow, g.blocked,
							hCur * 57.2958f, yaw * 57.2958f, rate, nextWalk);
					}
					if (haveRay && Ground(r, np.x, np.y, np.z + 40, np.z - 60, gp)) {
						np.z = cur.z + std::clamp(gp.z - cur.z, -cfg.driveFall * a_dt, (g.caps.steep > 0 ? 700.0f : cfg.driveRise) * a_dt);
					} else {
						np.z = cur.z;
					}
					if (cfg.log && !g.logged) {
						g.logged = true;
						logger::info("[{}] ROUTEDRIVE {:08X} '{}' run {:.0f} u/s, turn in place {:.0f} / moving {:.0f} deg/s (raceMotion {})", Tuning::ModeLabel(), g.id, g.name,
							mv.run, mv.rotStand, mv.rotMove, cfg.raceMotion);
					}
					Drive(actor, hNew, np, { fwd.x * g.speed, fwd.y * g.speed, 0.0f }, false);
					if (reached && !blockedNow) {
						++g.i;
					}
				}
				if (g.i >= g.pts.size() && !BeginHold(g, actor)) {
					EndGlide(g, "done", true);
					it = g_glides.erase(it);
				} else {
					++it;
				}
			}
		}

		void StartSearch(RE::Actor* a_actor, const Caps& a_caps, RE::TESObjectREFR* a_goal, float a_gap, bool stuck, float vanilla);

		void TryRoute(RE::Actor* a_actor, const Caps& a_caps, RE::TESObjectREFR* a_goal, float a_gap, Memo& a_memo, bool a_navOk = false)
		{
			const auto& cfg = Config::Get().jump;
			if (!a_goal || !a_goal->Is3DLoaded()) {
				return;
			}
			const Vec   pos = a_actor->GetPosition(), tp = a_goal->GetPosition();
			const float dist = (tp - pos).Length(), dh = Len2D(tp - pos);
			if (a_memo.rTarget != a_goal->GetFormID()) {
				a_memo.rTarget = a_goal->GetFormID();
				a_memo.rBest = (std::numeric_limits<float>::max)();
				a_memo.rBestTime = g_time;
			}
			if (dist < a_memo.rBest - 60.0f) {
				a_memo.rBest = dist;
				a_memo.rBestTime = g_time;
			}
			bool mismatch = false, vertOut = false;
			if (a_actor->IsInCombat() && a_gap < 100.0f) {
				Vec mv;
				a_actor->GetLinearVelocity(mv);
				const float dz = tp.z - pos.z;
				vertOut = dz < -cfg.vertMismatch * a_caps.height || dz > Config::Get().combat.swingRise * a_caps.height;
				mismatch = (Len2D(mv) < 50.0f || a_memo.forceStuck) && dh < 400.0f && vertOut;
			}
			if (dh > cfg.routeMaxDist || (dist < a_caps.radius + a_gap + 120.0f && !vertOut) || g_time < a_memo.routeNext || g_searches.size() >= 4 ||
				Searching(a_actor->GetFormID()) || FindGlide(a_actor->GetFormID())) {
				return;
			}
			const bool follower = !a_actor->IsInCombat() && a_actor->IsPlayerTeammate();
			if (follower) {
				Vec fv;
				a_actor->GetLinearVelocity(fv);
				if (dh < cfg.followRouteMin || Len2D(fv) > 50.0f) {
					a_memo.rBestTime = g_time;
					return;
				}
			}
			Vec rv;
			a_actor->GetLinearVelocity(rv);
			if (Len2D(rv) > 100.0f) {
				a_memo.rBestTime = g_time;
			}
			const bool stuck = a_memo.forceStuck || mismatch || g_time - a_memo.rBestTime >= (follower ? 2.0f * cfg.routeStuckSec : cfg.routeStuckSec);
			float      vanilla = 0;
			if (follower && !stuck) {
				return;
			}
			if (!stuck) {
				auto pr = PathProbe::GetPathRef(a_actor);
				static PathProbe::PathGeometry geo;
				if (!pr || !pr->path || (pr->state != 4 && pr->state != 0xd) || !PathProbe::ReadPath(pr->path, geo) ||
					geo.total != geo.count || geo.count < 2) {
					if (cfg.log && g_time >= a_memo.nextPfLog) {
						a_memo.nextPfLog = g_time + 5.0;
						logger::info("[{}] ROUTENOPATH {:08X} '{}' ref={} path={} state={} count={} total={} dist={:.0f}", Tuning::ModeLabel(),
							a_actor->GetFormID(), a_actor->GetName(), pr.has_value(), pr && pr->path, pr ? static_cast<int>(pr->state) : -1,
							geo.count, geo.total, dist);
					}
					return;
				}
				float acc = 0;
				Vec   prev = pos;
				for (std::uint32_t j = static_cast<std::uint32_t>(pr->progress) + 1; j < geo.count; ++j) {
					acc += prev.GetDistance(geo.nodes[j].pos);
					prev = geo.nodes[j].pos;
				}
				vanilla = acc + prev.GetDistance(tp);
				if (vanilla < cfg.minRatio * dist || vanilla - dist < cfg.minSaving) {
					return;
				}
			}
			StartSearch(a_actor, a_caps, a_goal, a_gap, stuck, vanilla);
			a_memo.forceStuck = false;
		}

		bool NavReaches(RE::Actor* a_actor, const Vec& a_goal)
		{
			auto pr = PathProbe::GetPathRef(a_actor);
			static PathProbe::PathGeometry geo;
			if (!pr || !pr->path || (pr->state != 4 && pr->state != 0xd) || !PathProbe::ReadPath(pr->path, geo) || geo.total != geo.count || geo.count < 2) {
				return false;
			}
			const Vec e = geo.nodes[geo.count - 1].pos;
			return Len2D(e - a_goal) <= 150.0f && std::fabs(e.z - a_goal.z) <= 120.0f;
		}

		int PlayerRank(RE::Actor* a_actor)
		{
			auto* lists = RE::ProcessLists::GetSingleton();
			auto* player = RE::PlayerCharacter::GetSingleton();
			if (!lists || !player) {
				return 99;
			}
			const Vec   pp = player->GetPosition();
			const float d = Len2D(a_actor->GetPosition() - pp);
			int         rank = 0;
			for (auto& h : lists->highActorHandles) {
				auto* o = h.get().get();
				if (o && o != a_actor && o != player && o->IsInCombat() && !o->IsDead() && Len2D(o->GetPosition() - pp) < d) {
					++rank;
				}
			}
			return rank;
		}

		void StartSearch(RE::Actor* a_actor, const Caps& a_caps, RE::TESObjectREFR* a_goal, float a_gap, bool stuck, float vanilla)
		{
			const Vec pos = a_actor->GetPosition(), tp = a_goal->GetPosition();
			g_memo[a_actor->GetFormID()].routeNext = g_time + 1.0;
			Search s;
			s.handle = a_actor->GetHandle();
			s.id = a_actor->GetFormID();
			s.goalId = a_goal->GetFormID();
			auto* n = a_actor->GetName();
			s.name = n && *n ? n : "<unnamed>";
			s.start = pos;
			s.goal = tp;
			s.caps = a_caps;
			s.gap = a_caps.radius + a_gap + 40.0f;
			s.stuck = stuck;
			s.melee = a_gap < 100.0f && a_actor->IsInCombat();
			s.vanilla = vanilla;
			s.maxNodes = a_actor->IsInCombat() && PlayerRank(a_actor) < Config::Get().jump.routeNearCount ? Config::Get().jump.routeMaxNodesNear : Config::Get().jump.routeMaxNodes;
			if (auto* cell = a_actor->GetParentCell()) {
				s.waterZ = cell->GetExteriorWaterHeight();
			}
			s.base = std::atan2(tp.y - pos.y, tp.x - pos.x);
			s.nodes.push_back({ pos, 0, -1, false });
			s.seen.insert(Key(pos));
			s.open.push({ (tp - pos).Length(), 0 });
			g_searches.push_back(std::move(s));
		}

		void TryReplan(RE::Actor* a_actor, const Caps& a_caps, Glide& a_g, Memo& a_m)
		{
			auto* goal = RE::TESForm::LookupByID<RE::TESObjectREFR>(a_g.goalId);
			if (!goal || g_time < a_m.routeNext || g_searches.size() >= 4 || Searching(a_g.id) ||
				goal->GetPosition().GetDistance(a_g.goal) < 150.0f) {
				return;
			}
			StartSearch(a_actor, a_caps, goal, a_g.gap - a_caps.radius - 40.0f, true, 0.0f);
		}

		bool TryShortcut(RE::Actor* a_actor, const Caps& a_caps, int& a_plans)
		{
			const auto& cfg = Config::Get().jump;
			auto        pr = PathProbe::GetPathRef(a_actor);
			if (!pr || !pr->path || (pr->state != 4 && pr->state != 0xd)) {
				return false;
			}
			static PathProbe::PathGeometry geo;
			if (!PathProbe::ReadPath(pr->path, geo) || geo.count < 2) {
				return false;
			}
			const int i0 = static_cast<int>(pr->progress);
			if (i0 + 1 >= static_cast<int>(geo.count)) {
				return false;
			}
			const Vec pos = a_actor->GetPosition();

			std::array<float, PathProbe::PathGeometry::kMaxNodes> arc{};
			int   last = i0;
			float acc = 0;
			for (int j = i0 + 1; j < static_cast<int>(geo.count); ++j) {
				acc += (j == i0 + 1 ? pos : geo.nodes[j - 1].pos).GetDistance(geo.nodes[j].pos);
				arc[j] = acc;
				if (geo.nodes[j].action) {
					break;
				}
				last = j;
			}

			for (int j = last; j > i0 && a_plans > 0; --j) {
				const Vec   d = geo.nodes[j].pos - pos;
				const float dh = Len2D(d);
				if (dh > a_caps.dist || dh < 40.0f || d.z > a_caps.up || -d.z > a_caps.down) {
					continue;
				}
				const float d3 = d.Length();
				const float saving = arc[j] - d3;
				if (saving < cfg.minSaving || arc[j] < cfg.minRatio * d3) {
					continue;
				}
				--a_plans;
				Plan       plan;
				const auto r = PlanJump(a_actor, a_caps, pos, geo.nodes[j].pos, nullptr, plan);
				if (r == Reject::kOk) {
					Launch(a_actor, a_caps, plan, Trigger::kShortcut, saving, pr->path, j, pr->progress);
					return true;
				}
				CountReject(r);
				RecordDebug({ pos, geo.nodes[j].pos }, false, std::format("shortcut: {}", RejectName(r)));
			}
			return false;
		}

		bool TryReach(RE::Actor* a_actor, const Caps& a_caps, RE::Actor* a_target, Trigger a_trigger, float a_stuckSec,
			float a_minRise, float a_gap, Memo& a_memo, int& a_plans)
		{
			const auto& cfg = Config::Get().jump;
			if (!a_target || a_target->IsDead() || !a_target->Is3DLoaded() || a_target->IsInMidair()) {
				a_memo.target = 0;
				return false;
			}
			const Vec   pos = a_actor->GetPosition();
			const Vec   tpos = a_target->GetPosition();
			const Vec   d = tpos - pos;
			const float dh = Len2D(d);

			if (a_memo.target != a_target->GetFormID()) {
				a_memo.target = a_target->GetFormID();
				a_memo.best = (std::numeric_limits<float>::max)();
				a_memo.bestTime = g_time;
			}
			const float dist = d.Length();
			if (dist < a_memo.best - 40.0f) {
				a_memo.best = dist;
				a_memo.bestTime = g_time;
			}
			if (!(d.z >= a_minRise || d.z <= -cfg.pursuitMinDrop)) {
				a_memo.best = dist;
				a_memo.bestTime = g_time;
				return false;
			}
			const float gap = a_caps.radius + a_target->GetBoundRadius() + a_gap;
			if (dh > a_caps.dist + gap || g_time - a_memo.bestTime < a_stuckSec || a_plans <= 0) {
				return false;
			}

			Vec back = dh > 1.0f ? Vec{ -d.x / dh, -d.y / dh, 0.0f } : Vec{ 0, -1, 0 };
			Reject first = Reject::kOk;
			for (float deg : { 0.0f, 45.0f, -45.0f, 90.0f, -90.0f }) {
				if (a_plans <= 0) {
					break;
				}
				const float c = std::cos(deg * 0.0174533f), s = std::sin(deg * 0.0174533f);
				const Vec   dir{ back.x * c - back.y * s, back.x * s + back.y * c, 0.0f };
				const Vec   land = tpos + dir * gap;
				--a_plans;
				Plan       plan;
				const auto r = PlanJump(a_actor, a_caps, pos, land, a_target, plan);
				if (r == Reject::kOk) {
					if (std::fabs(plan.end.z - tpos.z) <= 60.0f) {
						Launch(a_actor, a_caps, plan, a_trigger, 0.0f);
						return true;
					}
					CountReject(Reject::kLanding);
					first = first == Reject::kOk ? Reject::kLanding : first;
					continue;
				}
				CountReject(r);
				RecordDebug({ pos, land }, false, std::format("{}: {}", TriggerName(a_trigger), RejectName(r)));
				first = first == Reject::kOk ? r : first;
			}
			if (cfg.log && g_time >= a_memo.nextRejectLog) {
				a_memo.nextRejectLog = g_time + 5.0;
				auto* n = a_actor->GetName();
				logger::info("[{}] JUMPREJECT {:08X} '{}' {} dh={:.0f} dz={:+.0f} first={} caps(h={:.0f} up={:.0f} dist={:.0f})",
					Tuning::ModeLabel(), a_actor->GetFormID(), n ? n : "", TriggerName(a_trigger), dh, d.z, RejectName(first),
					a_caps.height, a_caps.up, a_caps.dist);
			}
			a_memo.nextEval = g_time + 0.5;
			return false;
		}

		void Evaluate(RE::Actor* a_actor, int& a_plans)
		{
			const auto& cfg = Config::Get().jump;
			if (SpiderClimb::Active(a_actor->GetFormID())) {
				return;
			}
			auto&       m = g_memo[a_actor->GetFormID()];
			if (g_time < m.nextEval || g_time < m.cooldownUntil) {
				return;
			}
			m.nextEval = g_time + kEvalInterval;
			for (auto& f : g_flights) {
				if (f.id == a_actor->GetFormID()) {
					return;
				}
			}
			Caps caps;
			if (!CanJump(a_actor, caps)) {
				return;
			}
			if (auto* g = FindGlide(a_actor->GetFormID())) {
				TryReplan(a_actor, caps, *g, m);
				return;
			}
			bool navOk = false;
			if (!cfg.jumpOnNavmesh && a_actor->IsInCombat()) {
				if (auto tgt = a_actor->GetActorRuntimeData().currentCombatTarget.get(); tgt && NavReaches(a_actor, tgt->GetPosition())) {
					navOk = true;
				}
			}
			if (!caps.jumps) {
			} else if (a_actor->IsInCombat()) {
				if (cfg.pursuit && !IsRanged(a_actor) && !navOk) {
					auto tgt = a_actor->GetActorRuntimeData().currentCombatTarget.get();
					if (TryReach(a_actor, caps, tgt.get(), Trigger::kPursuit, cfg.pursuitStuckSec, cfg.pursuitMinRise, 15.0f, m, a_plans)) {
						return;
					}
				}
			} else if (cfg.follow && a_actor->IsPlayerTeammate()) {
				if (TryReach(a_actor, caps, RE::PlayerCharacter::GetSingleton(), Trigger::kFollow, cfg.followStuckSec, 100.0f, 60.0f, m, a_plans)) {
					return;
				}
			}
			const bool hurry = a_actor->IsPlayerTeammate() || InHurry(a_actor);
			if (cfg.shortcut && hurry) {
				if (caps.jumps && TryShortcut(a_actor, caps, a_plans)) {
					return;
				}
				if (cfg.route) {
					if (a_actor->IsInCombat()) {
						TryRoute(a_actor, caps, a_actor->GetActorRuntimeData().currentCombatTarget.get().get(), IsRanged(a_actor) ? 450.0f : 15.0f, m, navOk);
					} else if (a_actor->IsPlayerTeammate()) {
						TryRoute(a_actor, caps, RE::PlayerCharacter::GetSingleton(), 60.0f, m);
					}
				}
			}
		}

		void LogStatsIfDue()
		{
			if (g_time < g_nextStatsLog) {
				return;
			}
			g_nextStatsLog = g_time + 60.0;
			Stats s;
			{
				std::scoped_lock l(g_statsLock);
				s = g_stats;
			}
			if (s.evals == g_loggedStats.evals) {
				return;
			}
			g_loggedStats = s;
			logger::info("[{}] JumpStats plans={} launched shortcut={} pursuit={} follow={} route={} landed={} aborted={} "
						 "rejected range={} profile={} rise={} sweep={} landing={} occupied={} routes planned={} failed={} done={}",
				Tuning::ModeLabel(), s.evals, s.launched[1], s.launched[2], s.launched[3], s.launched[4], s.landed, s.aborted, s.rejRange,
				s.rejProfile, s.rejRise, s.rejSweep, s.rejLanding, s.rejOccupied, s.routes, s.routesFailed, s.routesDone);
		}

		void Tick(float a_dt)
		{
			auto* ui = RE::UI::GetSingleton();
			if (a_dt <= 0.0f || (ui && ui->GameIsPaused())) {
				return;
			}
			a_dt = (std::min)(a_dt, 0.1f);
			g_time += a_dt;

			UpdateFlights(a_dt);
			UpdateLandPoses();
			UpdateGlides(a_dt);
			PostLandChecks();
			LogStatsIfDue();
			if (!enabled || !Config::Get().jump.enabled) {
				return;
			}
			TickSearches();

			auto* lists = RE::ProcessLists::GetSingleton();
			auto* player = RE::PlayerCharacter::GetSingleton();
			if (!lists || !player || !player->Is3DLoaded()) {
				return;
			}
			std::vector<RE::Actor*> actors;
			actors.reserve(lists->highActorHandles.size());
			for (auto& h : lists->highActorHandles) {
				if (auto a = h.get().get(); a && a != player) {
					actors.push_back(a);
				}
			}
			if (actors.empty()) {
				return;
			}
			if (g_time >= g_nextRefresh) {
				g_nextRefresh = g_time + 0.1;
				PathProbe::Refresh(actors);
			}
			int plans = kPlansPerFrame;
			for (std::size_t i = 0; i < actors.size() && plans > 0; ++i) {
				Evaluate(actors[(g_rr + i) % actors.size()], plans);
			}
			g_rr = (g_rr + 1) % actors.size();

			if (g_time >= g_nextPrune) {
				g_nextPrune = g_time + 30.0;
				std::unordered_set<RE::FormID> live;
				for (auto* a : actors) {
					live.insert(a->GetFormID());
				}
				std::erase_if(g_memo, [&](const auto& kv) { return !live.contains(kv.first) && kv.second.postCheckAt == 0 && g_time > kv.second.cooldownUntil; });
			}
		}
	}

	bool InHurry(RE::Actor* a_actor)
	{
		const auto* st = a_actor->AsActorState();
		const auto* process = a_actor->GetActorRuntimeData().currentProcess;
		const bool  fleeing = process && process->middleHigh && process->middleHigh->isFleeing;
		Vec         vel;
		a_actor->GetLinearVelocity(vel);
		return a_actor->IsInCombat() || fleeing || (st && (st->IsRunning() || st->IsSprinting())) || Len2D(vel) > 200.0f;
	}

	void OnFrame(float a_dt)
	{
		g_dt = std::clamp(a_dt, 0.001f, 0.1f);
		Tick(a_dt);
	}

	void Reset()
	{
		g_flights.clear();
		g_glides.clear();
		g_searches.clear();
		g_memo.clear();
		g_raceExcluded.clear();
	}

	bool OnNavmesh(RE::Actor* a_actor, RE::Actor* a_target)
	{
		return a_actor && a_target && NavReaches(a_actor, a_target->GetPosition());
	}

	bool Routing(RE::FormID a_id)
	{
		return Searching(a_id) || FindGlide(a_id) != nullptr;
	}

	bool InFlight(RE::FormID a_id)
	{
		return std::ranges::any_of(g_flights, [&](const Flight& f) { return f.id == a_id; });
	}

	int TraceState(RE::FormID a_id)
	{
		for (auto& f : g_flights) {
			if (f.id == a_id) {
				return static_cast<int>(f.trigger);
			}
		}
		for (auto& g : g_glides) {
			if (g.id == a_id) {
				return static_cast<int>(Trigger::kRoute);
			}
		}
		return 0;
	}

	Stats GetStats()
	{
		std::scoped_lock l(g_statsLock);
		return g_stats;
	}

	void GetDebugArcs(std::vector<DebugArc>& a_out)
	{
		a_out.clear();
		for (auto& e : g_debug) {
			const float age = static_cast<float>(g_time - e.time);
			if (age < (e.launched ? 15.0f : 4.0f)) {
				a_out.push_back({ e.points, age, e.launched, e.label });
			}
		}
	}
}
