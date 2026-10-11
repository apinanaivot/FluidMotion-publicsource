#include "Telemetry.h"
#include "Config.h"
#include "JumpNav.h"
#include "PathRequestProbe.h"
#include "PathProbe.h"
#include "Tuning.h"

namespace Telemetry
{
	namespace
	{
		using Clock = std::chrono::steady_clock;

		constexpr float kRadToDeg = 57.2957795f;

		struct Track
		{
			std::string name;
			RE::NiPoint3 lastPos;
			bool         hasLast = false;
			std::uint64_t lastSeenTick = 0;

			RE::NiPoint3 segStart;
			float        segHeading = 0.0f;
			bool         hasSegHeading = false;

			bool         active = false;
			float        graceLeft = 0.0f;
			float        duration = 0.0f;
			float        slowTime = 0.0f;
			float        pathLen = 0.0f;
			float        maxSpeed = 0.0f;
			int          stalls = 0;
			int          sharpTurns = 0;
			bool         wasFast = false;
			RE::NiPoint3 startPos;
			float        startPlayerDist = 0.0f;
			float        lastPlayerDist = 0.0f;
			float        fleeFlagTime = 0.0f;
			float        combatTime = 0.0f;
			std::unordered_map<std::string, float> packageTime;
			std::string  lastPackage;
			int          packageSwitches = 0;
			std::uintptr_t loggedPath = 0;
			RE::NiPoint3   loggedPathStart;
			std::uint32_t  loggedPathCount = 0;
			std::array<float, 64> speedHist{};
		};

		constexpr float kSpeedBin = 25.0f;

		float SpeedQuantile(const Track& a_t, float a_q)
		{
			float acc = 0.0f;
			for (std::size_t i = 0; i < a_t.speedHist.size(); ++i) {
				acc += a_t.speedHist[i];
				if (acc >= a_q * a_t.duration) {
					return (i + 0.5f) * kSpeedBin;
				}
			}
			return a_t.maxSpeed;
		}

		float FullSpeedShare(const Track& a_t)
		{
			const float threshold = 0.8f * SpeedQuantile(a_t, 0.9f);
			float       t = 0.0f;
			for (std::size_t i = 0; i < a_t.speedHist.size(); ++i) {
				if ((i + 0.5f) * kSpeedBin >= threshold) {
					t += a_t.speedHist[i];
				}
			}
			return a_t.duration > 0 ? t / a_t.duration : 0.0f;
		}

		std::unordered_map<RE::FormID, Track> g_tracks;
		std::uint64_t                         g_tick = 0;
		Clock::time_point                     g_lastTickTime{};
		bool                                  g_hasLastTickTime = false;
		double                                g_clock = 0.0;
		std::ofstream                         g_trace;
		std::ofstream                         g_paths;

		std::atomic_bool g_started{ false };
		std::atomic_bool g_taskPending{ false };
		std::atomic_int  g_intervalMs{ 33 };

		std::mutex                  g_snapLock;
		std::vector<LiveActor>      g_live;
		std::deque<EpisodeSummary>  g_recent;
		constexpr std::size_t       kMaxRecent = 30;
		std::atomic_bool            g_traceEnabled{ false };

		float Dist2D(const RE::NiPoint3& a, const RE::NiPoint3& b)
		{
			return std::hypot(a.x - b.x, a.y - b.y);
		}

		float AngleDiffDeg(float a, float b)
		{
			float d = std::fmod(std::fabs(a - b), 360.0f);
			return d > 180.0f ? 360.0f - d : d;
		}

		void OpenTrace()
		{
			g_trace.close();
			g_paths.close();
			if (!g_traceEnabled) {
				return;
			}
			auto dir = logger::log_directory();
			if (!dir) {
				return;
			}
			const auto now = std::chrono::zoned_time{ std::chrono::current_zone(),
				std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now()) };
			auto path = *dir / std::format("FluidMotion_trace_{:%Y%m%d_%H%M%S}.csv", now);
			g_trace.open(path, std::ios::out | std::ios::trunc);
			g_trace << "t,formid,name,x,y,z,speed,fleeing,combat,episode,package,yaw,gait,gSpeed,gTurnDelta,gDirection,mode,pfPath,pfPoints,pfProgress,pfState,pf158,rqSpeed,rqDist,rqEnd,rqFlag,tcSpeed,tcDist,tcEnd,tcAcc,tcDec,tcNode,tcFlag,tcFix,pfRawE0,clAngVel,clAngAcc,clDec,clOmega,clVanilla,clLimit,acDes,acYaw,acW0,ac6C,acWmax,acAlpha,acCmd,ac50,playerDist,px,py,jump,swim\n";
			g_paths.open(*dir / std::format("FluidMotion_trace_{:%Y%m%d_%H%M%S}_paths.csv", now), std::ios::out | std::ios::trunc);
			g_paths << "t,formid,name,pfPath,total,incomplete,i,x,y,z,tx,ty,tz,action\n";
		}

		void WritePathIfNew(Track& a_track, RE::Actor* a_actor, std::uintptr_t a_path)
		{
			if (!g_paths.is_open() || !a_path) {
				return;
			}
			static PathProbe::PathGeometry geo;
			if (!PathProbe::ReadPath(a_path, geo) || geo.count == 0) {
				return;
			}
			const auto& first = geo.nodes[0].pos;
			if (a_path == a_track.loggedPath && geo.total == a_track.loggedPathCount &&
				first.x == a_track.loggedPathStart.x && first.y == a_track.loggedPathStart.y) {
				return;
			}
			a_track.loggedPath = a_path;
			a_track.loggedPathStart = first;
			a_track.loggedPathCount = geo.total;
			for (std::uint32_t i = 0; i < geo.count; ++i) {
				const auto& n = geo.nodes[i];
				g_paths << std::format("{:.3f},{:08X},\"{}\",{:X},{},{},{},{:.1f},{:.1f},{:.1f},{:.3f},{:.3f},{:.3f},{}\n",
					g_clock, a_actor->GetFormID(), a_track.name, a_path, geo.total, geo.incomplete ? 1 : 0, i,
					n.pos.x, n.pos.y, n.pos.z, n.tangent.x, n.tangent.y, n.tangent.z, n.action ? 1 : 0);
			}
		}

		std::string DescribePackage(RE::Actor* a_actor)
		{
			auto* pkg = a_actor->GetCurrentPackage();
			if (!pkg) {
				return "none";
			}
			auto* type = pkg->GetObjectTypeName();
			return std::format("{}:{:08X}", type && *type ? type : "?", pkg->GetFormID());
		}

		std::string SummarizePackages(const Track& a_t)
		{
			std::vector<std::pair<std::string, float>> v(a_t.packageTime.begin(), a_t.packageTime.end());
			std::ranges::sort(v, [](auto& a, auto& b) { return a.second > b.second; });
			std::string out;
			for (std::size_t i = 0; i < v.size() && i < 3; ++i) {
				out += std::format("{}{} {:.0f}%", i ? ", " : "", v[i].first, 100.0f * v[i].second / a_t.duration);
			}
			return out.empty() ? "none" : out;
		}

		void EndEpisode(RE::FormID a_id, Track& a_t, const char* a_end)
		{
			const auto& cfg = Config::Get().telemetry;
			if (a_t.active && a_t.duration >= cfg.minEpisodeSec) {
				const char* reason = a_t.fleeFlagTime > 0 ? "flee" : a_t.combatTime > 0 ? "combat" : "run";
				{
					std::scoped_lock l(g_snapLock);
					g_recent.push_front({ Tuning::ModeLabel(), a_t.name, reason, a_t.duration,
						a_t.slowTime / a_t.duration, FullSpeedShare(a_t),
						a_t.pathLen > 0 ? Dist2D(a_t.startPos, a_t.lastPos) / a_t.pathLen : 0.0f,
						a_t.pathLen / a_t.duration, SpeedQuantile(a_t, 0.5f), SpeedQuantile(a_t, 0.9f), a_t.stalls, a_t.packageSwitches, a_end });
					if (g_recent.size() > kMaxRecent) {
						g_recent.pop_back();
					}
				}
				const float disp = Dist2D(a_t.startPos, a_t.lastPos);
				const float avg = a_t.duration > 0 ? a_t.pathLen / a_t.duration : 0.0f;
				const float minutes = a_t.duration / 60.0f;
				logger::info(
					"[{}] EPISODE {:08X} '{}' ({}) dur={:.1f}s slow={:.0f}% stalls={} ({:.1f}/min) sharpTurns={} "
					"path={:.0f} disp={:.0f} eff={:.2f} avg={:.0f}u/s max={:.0f}u/s playerDist {:.0f}->{:.0f} "
					"fullSpeed={:.0f}% pkgSwitches={} p50={:.0f}u/s p90={:.0f}u/s end={} fleeFlag={:.0f}% combat={:.0f}% packages=[{}]",
					Tuning::ModeLabel(), a_id, a_t.name, reason, a_t.duration,
					100.0f * a_t.slowTime / a_t.duration, a_t.stalls, a_t.stalls / minutes, a_t.sharpTurns,
					a_t.pathLen, disp, a_t.pathLen > 0 ? disp / a_t.pathLen : 0.0f, avg, a_t.maxSpeed,
					a_t.startPlayerDist, a_t.lastPlayerDist,
					100.0f * FullSpeedShare(a_t), a_t.packageSwitches,
					SpeedQuantile(a_t, 0.5f), SpeedQuantile(a_t, 0.9f), a_end,
					100.0f * a_t.fleeFlagTime / a_t.duration, 100.0f * a_t.combatTime / a_t.duration,
					SummarizePackages(a_t));
			}
			a_t.active = false;
		}

		void ResetAll()
		{
			for (auto& [id, t] : g_tracks) {
				EndEpisode(id, t, "load");
			}
			g_tracks.clear();
			g_hasLastTickTime = false;
		}

		void Tick()
		{
			g_taskPending = false;
			PathRequestProbe::LogIfDue();
			const auto& cfg = Config::Get().telemetry;
			if (!cfg.enabled) {
				return;
			}

			auto* ui = RE::UI::GetSingleton();
			auto* player = RE::PlayerCharacter::GetSingleton();
			auto* lists = RE::ProcessLists::GetSingleton();
			if (!player || !lists || !player->Is3DLoaded() || (ui && ui->GameIsPaused())) {
				g_hasLastTickTime = false;
				for (auto& [id, t] : g_tracks) {
					t.hasLast = false;
				}
				return;
			}

			const auto now = Clock::now();
			const float dt = g_hasLastTickTime ? std::chrono::duration<float>(now - g_lastTickTime).count() : 0.0f;
			g_lastTickTime = now;
			g_hasLastTickTime = true;
			if (dt <= 0.0f || dt > 0.5f) {
				for (auto& [id, t] : g_tracks) {
					t.hasLast = false;
				}
				if (dt <= 0.0f) {
					return;
				}
			}
			g_clock += dt;
			++g_tick;

			const auto playerPos = player->GetPosition();

			std::vector<RE::Actor*> actors;
			for (auto& handle : lists->highActorHandles) {
				if (auto a = handle.get().get(); a && a != player) {
					actors.push_back(a);
				}
			}
			PathProbe::Refresh(actors);

			std::vector<LiveActor> live;
			for (auto& handle : lists->highActorHandles) {
				auto actorPtr = handle.get();
				auto* actor = actorPtr.get();
				if (!actor || actor == player || actor->IsDead() || actor->IsDisabled() || !actor->Is3DLoaded()) {
					continue;
				}
				const auto  pos = actor->GetPosition();
				const float playerDist = Dist2D(pos, playerPos);
				if (playerDist > cfg.radius) {
					continue;
				}

				auto* process = actor->GetActorRuntimeData().currentProcess;
				const bool fleeing = process && process->middleHigh && process->middleHigh->isFleeing;
				const bool combat = actor->IsInCombat();

				auto& t = g_tracks[actor->GetFormID()];
				t.lastSeenTick = g_tick;
				if (t.name.empty()) {
					auto* n = actor->GetName();
					t.name = n && *n ? n : "<unnamed>";
				}

				std::string pkgCache;
				auto        pkgNow = [&]() -> const std::string& {
                    if (pkgCache.empty()) {
                        pkgCache = DescribePackage(actor);
                    }
                    return pkgCache;
				};
				float speed = 0.0f;
				float step = 0.0f;
				if (t.hasLast && dt > 0.0f && dt <= 0.5f) {
					step = Dist2D(pos, t.lastPos);
					speed = step / dt;
				}
				const bool flagged = (cfg.trackFleeing && fleeing) || (cfg.trackCombat && combat);
				const bool start = flagged || (cfg.trackRunning && speed >= cfg.fastSpeed);
				const bool keep = flagged || (cfg.trackRunning && speed > cfg.slowSpeed);

				if (!t.active && start) {
					t = Track{ .name = t.name, .lastPos = t.lastPos, .hasLast = t.hasLast, .lastSeenTick = t.lastSeenTick };
					t.active = true;
					t.startPos = pos;
					t.segStart = pos;
					t.startPlayerDist = playerDist;
				}
				if (t.active) {
					if (start || keep) {
						t.graceLeft = cfg.episodeGraceSec;
					} else {
						t.graceLeft -= dt;
						if (t.graceLeft <= 0.0f) {
							EndEpisode(actor->GetFormID(), t, "stopped");
						}
					}
				}

				if (t.active && t.hasLast) {
					t.duration += dt;
					t.pathLen += step;
					t.maxSpeed = (std::max)(t.maxSpeed, speed);
					t.lastPlayerDist = playerDist;
					t.fleeFlagTime += fleeing ? dt : 0.0f;
					t.combatTime += combat ? dt : 0.0f;
					const auto& pkg = pkgNow();
					if (!t.lastPackage.empty() && pkg != t.lastPackage) {
						++t.packageSwitches;
						logger::info("[{}] PKGSWITCH {:08X} '{}' t={:.1f}s {} -> {} speed={:.0f}u/s playerDist={:.0f}",
							Tuning::ModeLabel(), actor->GetFormID(), t.name, t.duration, t.lastPackage, pkg, speed, playerDist);
					}
					t.lastPackage = pkg;
					t.packageTime[pkg] += dt;
					t.speedHist[(std::min)(static_cast<std::size_t>(speed / kSpeedBin), t.speedHist.size() - 1)] += dt;
					if (speed < cfg.slowSpeed) {
						t.slowTime += dt;
					}
					if (speed >= cfg.fastSpeed) {
						t.wasFast = true;
					} else if (t.wasFast && speed < cfg.slowSpeed) {
						t.wasFast = false;
						++t.stalls;
					}
					if (Dist2D(pos, t.segStart) >= cfg.turnSegment) {
						const float heading = std::atan2(pos.x - t.segStart.x, pos.y - t.segStart.y) * kRadToDeg;
						if (t.hasSegHeading && AngleDiffDeg(heading, t.segHeading) >= cfg.sharpTurnDeg) {
							++t.sharpTurns;
						}
						t.segHeading = heading;
						t.hasSegHeading = true;
						t.segStart = pos;
					}
				}

				if (t.active || speed > cfg.slowSpeed) {
					live.push_back({ actor->GetFormID(), t.name, t.active ? t.lastPackage : pkgNow(), speed, playerDist,
						t.active, t.duration, t.stalls, FullSpeedShare(t) });
				}

				if (g_trace.is_open() && t.active) {
					static const RE::BSFixedString kSpeed{ "Speed" }, kTurnDelta{ "TurnDelta" }, kDirection{ "Direction" };
					auto graphFloat = [&](const RE::BSFixedString& a_name) {
						float v = 0.0f;
						return actor->GetGraphVariableFloat(a_name, v) ? std::format("{:.3f}", v) : std::string{};
					};
					const auto* state = actor->AsActorState();
					const char* gait = !state                ? "" :
					                   state->IsSprinting()  ? "sprint" :
					                   state->IsRunning()    ? "run" :
					                   state->IsWalking()    ? "walk" :
					                                           "other";
					std::string pf = ",,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,";
					if (auto ps = PathProbe::Get(actor)) {
						std::string raw;
						for (auto v : ps->raw) {
							raw += std::format("{}{:g}", raw.empty() ? "" : " ", v);
						}
						const auto rq = ps->hasRequest ?
							std::format("{:.1f},{:.1f},{:.1f},{}", ps->rqSpeed, ps->rqDist, ps->rqEnd, ps->rqFlag) :
							std::string{ ",,," };
						const auto tc = ps->hasTc ?
							std::format("{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{},{}", ps->tcSpeed, ps->tcDist, ps->tcEnd,
								ps->tcAcc, ps->tcDec, ps->tcNode, ps->tcFlag, ps->tcFix) :
							std::string{ ",,,,,,," };
						const auto cl = ps->hasCl ?
							std::format("{:.3f},{:.3f},{:.1f},{:.3f},{:.1f},{:.1f}", ps->cl.angVel, ps->cl.angAcc, ps->cl.dec,
								ps->cl.omega, ps->cl.vanilla, ps->cl.used) :
							std::string{ ",,,,," };
						const auto ac = ps->hasAc ?
							std::format("{:.4f},{:.4f},{:.4f},{:.4f},{:.3f},{:.3f},{:.4f},{:.4f}", ps->ac.desired, ps->ac.yaw, ps->ac.w0,
								ps->ac.f6c, ps->ac.wmax, ps->ac.alpha, ps->ac.cmd, ps->ac.f50) :
							std::string{ ",,,,,,," };
						pf = std::format("{:X},{},{:.3f},{},{:.3f},{},{},{},{},{}", ps->path, ps->points, ps->progress, ps->state, ps->f158, rq, tc, raw, cl, ac);
						WritePathIfNew(t, actor, ps->path);
					}
					g_trace << std::format("{:.3f},{:08X},\"{}\",{:.1f},{:.1f},{:.1f},{:.1f},{},{},{},{},{:.4f},{},{},{},{},{},{},{:.0f},{:.0f},{:.0f},{},{}\n",
						g_clock, actor->GetFormID(), t.name, pos.x, pos.y, pos.z, speed,
						fleeing ? 1 : 0, combat ? 1 : 0, t.active ? 1 : 0, pkgNow(),
						actor->GetAngleZ(), gait, graphFloat(kSpeed), graphFloat(kTurnDelta), graphFloat(kDirection), Tuning::ModeLabel(), pf,
						playerDist, playerPos.x, playerPos.y, JumpNav::TraceState(actor->GetFormID()), actor->AsActorState()->IsSwimming() ? 1 : 0);
				}

				t.lastPos = pos;
				t.hasLast = true;
			}

			std::ranges::sort(live, [](auto& a, auto& b) { return a.playerDist < b.playerDist; });
			{
				std::scoped_lock l(g_snapLock);
				g_live = std::move(live);
			}

			for (auto it = g_tracks.begin(); it != g_tracks.end();) {
				if (it->second.lastSeenTick != g_tick) {
					EndEpisode(it->first, it->second, "out-of-range");
					it = g_tracks.erase(it);
				} else {
					++it;
				}
			}
		}
	}

	void Start()
	{
		if (g_started.exchange(true)) {
			return;
		}
		g_traceEnabled = Config::Get().telemetry.writeTraceCsv;
		PathProbe::recording = g_traceEnabled.load() && Config::Get().telemetry.enabled;
		OpenTrace();
		g_intervalMs = Config::Get().telemetry.sampleIntervalMs;
		std::thread([] {
			for (;;) {
				std::this_thread::sleep_for(std::chrono::milliseconds(g_intervalMs.load()));
				if (!g_taskPending.exchange(true)) {
					SKSE::GetTaskInterface()->AddTask([] { Tick(); });
				}
			}
		}).detach();
		logger::info("Telemetry sampler started");
	}

	void OnLoad()
	{
		g_intervalMs = Config::Get().telemetry.sampleIntervalMs;
		SKSE::GetTaskInterface()->AddTask([] {
			PathProbe::recording = g_traceEnabled.load() && Config::Get().telemetry.enabled;
			ResetAll();
			OpenTrace();
		});
	}

	void EndAllEpisodes()
	{
		for (auto& [id, t] : g_tracks) {
			EndEpisode(id, t, "mode-switch");
		}
	}

	void SetTraceEnabled(bool a_enabled)
	{
		SKSE::GetTaskInterface()->AddTask([a_enabled] {
			g_traceEnabled = a_enabled;
			PathProbe::recording = a_enabled && Config::Get().telemetry.enabled;
			OpenTrace();
			logger::info("Trace CSV {}", a_enabled ? "recording" : "stopped");
		});
	}

	bool IsTraceEnabled() { return g_traceEnabled; }

	void GetSnapshot(std::vector<LiveActor>& a_live, std::vector<EpisodeSummary>& a_recent)
	{
		std::scoped_lock l(g_snapLock);
		a_live = g_live;
		a_recent.assign(g_recent.begin(), g_recent.end());
	}
}
