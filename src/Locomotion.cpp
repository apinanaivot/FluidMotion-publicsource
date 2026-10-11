#include "Locomotion.h"
#include "AnimMath.h"
#include "ClimbClip.h"
#include "CrashLog.h"
#include "JumpNav.h"
#include "Mantle.h"
#include "MotionMatch.h"
#include "SpiderLegs.h"
#include "TDM.h"
#include "Raycast.h"
#include "Tuning.h"

namespace Locomotion
{
	using namespace AnimMath;

	namespace
	{
		Config::Anim                g_cfg;
		std::atomic<float>          g_time{ 0.0f };
		std::atomic<std::uint32_t>  g_frame{ 0 };
		std::atomic<std::uint32_t>  g_dodgeFrame{ 0 };
		std::thread::id             g_mainThread;
		std::atomic_bool            g_strideOn{ false };
		V3                          g_mmGoal{};
		bool                        g_mmGoalOn = false;
		bool                        g_fpOff = false;
		bool                        PlayerDodging() { return g_dodgeFrame.load(std::memory_order_relaxed) != 0 && g_frame.load(std::memory_order_relaxed) - g_dodgeFrame.load(std::memory_order_relaxed) <= 4; }
		float                       g_mmMag = 0.0f;
		std::atomic<float>          g_visYawOff{ 0.0f };
		std::atomic_bool            g_visOwned{ false };
		V3                          g_camOff{};
		float                       g_camDt = 0.0f;
		std::uint32_t               g_camTick = 0;
		struct CamApplied
		{
			bool                  on = false;
			const void*           state = nullptr;
			V3                    world{};
		} g_cam;
		struct CamStats
		{
			float maxOff = 0, maxStep = 0, wcOff = 0, steadyOff = 0;
			int   steadyN = 0;
			int   sets = 0, lost[3]{};
		} g_vst;

		struct Counters
		{
			std::atomic<std::uint64_t> postNs{ 0 }, posts{ 0 }, rays{ 0 }, speedOvr{ 0 };
			std::atomic<float>         staleMax{ 0.0f };
			std::atomic<std::uint64_t> mmSearch{ 0 }, mmSwitch{ 0 }, mmNs{ 0 }, mmPosts{ 0 };
		} g_n;
		std::atomic<std::uint32_t> g_activeActors{ 0 };
		Stats                      g_stats;

		constexpr const char* kBoneCom = "NPC COM [COM ]";
		constexpr const char* kSpine[3] = { "NPC Spine [Spn0]", "NPC Spine1 [Spn1]", "NPC Spine2 [Spn2]" };
		constexpr const char* kHead = "NPC Head [Head]";
		constexpr const char* kThigh[2] = { "NPC L Thigh [LThg]", "NPC R Thigh [RThg]" };
		constexpr const char* kCalf[2] = { "NPC L Calf [LClf]", "NPC R Calf [RClf]" };
		constexpr const char* kFoot[2] = { "NPC L Foot [Lft ]", "NPC R Foot [Rft ]" };
		constexpr const char* kUArm[2] = { "NPC L UpperArm [LUar]", "NPC R UpperArm [RUar]" };
		constexpr const char* kFArm[2] = { "NPC L Forearm [LLar]", "NPC R Forearm [RLar]" };
		constexpr const char* kHand[2] = { "NPC L Hand [LHnd]", "NPC R Hand [RHnd]" };

		struct Bones
		{
			RE::NiAVObject *com = nullptr, *spine[3]{}, *head = nullptr;
			RE::NiAVObject *thigh[2]{}, *calf[2]{}, *foot[2]{}, *uarm[2]{}, *farm[2]{}, *hand[2]{};
			bool            arms = false;
		};

		struct Probe
		{
			bool  valid = false;
			float gz = 0, cx = 0, cy = 0, t = -9.0f;
			V3    n{ 0, 0, 1 };
		};

		struct Foot
		{
			V3    anim{};
			bool  haveAnim = false;
			float lift = 0;
			Probe under, ahead;
		};

		struct PlayerRedirect
		{
			struct Entry
			{
				RE::BShkbAnimationGraph* graph;
				int                      idx;
				RE::NiNode*              real;
				RE::NiNode*              dummy;
				RE::NiNode*              origNode;
				std::uint32_t            origSlot;
			};
			std::vector<Entry>                                        redir;
			std::vector<RE::BSTSmartPointer<RE::BShkbAnimationGraph>> graphRefs;
			std::vector<RE::NiPointer<RE::NiNode>>                    dummies;
			RE::NiPointer<RE::NiAVObject>                             root;
			RE::NiAVObject*                                           failedRoot = nullptr;
			std::mutex                                                m;
			std::vector<RE::NiTransform>                              pose;
			std::atomic<bool>                                         hooked{ false };
			std::atomic<bool>                                         busy{ false };
			std::atomic<bool>                                         inPass{ false };
			bool                                                      player = false;
		};
		std::atomic<std::uint32_t> g_preRenderFrame{ 0 };
		std::atomic_int            g_passOverlap{ 0 };

		void PrUnhook(PlayerRedirect& pr, const char* a_why);

		struct Slot
		{
			~Slot();
			PlayerRedirect pr;
			std::mutex     m;
			RE::FormID     id = 0;
			bool           player = false;
			std::uint32_t  sampleFrame = 0, lastRegFrame = 0;
			float          sampleT = -9.0f, animT = -9.0f, nextCast = 0.0f;

			bool  havePos = false, haveHeading = false, eligible = false, hasNormal = false;
			V3    pos{}, vel{}, normal{ 0, 0, 1 }, vdir{ 0, 1, 0 };
			float speedH = 0, speed3 = 0, heading = 0, turnRate = 0, height = 128.0f, slopeUp = 0, duckNeed = 0;
			float popLast = 0, popAmp = 0, popT0 = -9.0f, staleFrames = 0;
			int   popFlips = 0, popN = 0;
			float dipKick = 0, accel = 0, prevSpeedH = 0, airTopZ = 0, dipCool = 0, airT = -9.0f, airStart = -9.0f, histPush = -9.0f, stepVis = 0, driven = 0;
			bool  prevGrounded = false, checkRB = false;
			float setEff = 0, setCmd = 0;
			float cmdSpeed = 0, cmdT = -9.0f, lowSince = -9.0f, blockedT = 0, touchT = -9.0f, lastPostT = 0;
			std::uint32_t postFrame = 0, climbFrame = 0, overrideFrame = 0;
			struct Touch
			{
				bool valid = false;
				V3   p{}, n{ 0, 0, 1 };
			};
			Touch      touch[6];
			float      touchW[2]{};
			bool       touchPlant[2]{};
			float      touchLift[2]{};
			V3         touchN[2]{};
			V3         touchP[2]{};
			ClimbPose  climbPose;
			std::vector<RE::NiAVObject*> cnodes;
			std::vector<RE::NiAVObject*> jnodes[3];
			std::uint32_t drivenFrame = 0;
			float histT[16]{}, histV[16]{};
			int   histN = 0, histI = 0;

			float gate = 0, bank = 0, head = 0, lean = 0, slopeP = 0, duck = 0, dip = 0, dipV = 0, k = 1.0f;
						Foot  foot[2];

			RE::NiPointer<RE::NiAVObject> root;
			Bones                         b;
			bool                          bonesBad = false;
			MotionMatch::State            mm;
			float                         mmW = 0, mmArmW = 0, mmIdle = 0, mmSteady = 0, mmDes = 0, mmLogT = 0, mmUnit = 300.0f, mmBlockT = 0;
			bool                          mmOk = false, drawn = false, combat = false;
			const RE::NiAVObject*         mmRoot = nullptr;
			std::vector<RE::NiAVObject*>  mmNodes;
			std::vector<float>            mmGq;
			std::uint32_t                 bonesFrame = 0;
			const void*                   chars[2]{};
			std::vector<std::pair<RE::NiAVObject*, RE::NiTransform>> commit;
			std::uint32_t                                            commitFrame = 0;
		};

		std::mutex                                            g_tableMutex;
		std::unordered_map<RE::FormID, std::shared_ptr<Slot>> g_table;

		std::shared_ptr<Slot> Find(RE::FormID a_id)
		{
			std::lock_guard lk(g_tableMutex);
			const auto      it = g_table.find(a_id);
			return it == g_table.end() ? nullptr : it->second;
		}

		bool Active() { return g_cfg.enabled && Tuning::GetMode() == Tuning::Mode::kFluid; }

		std::atomic_bool g_raceMenu{ false };

		struct MenuSink : RE::BSTEventSink<RE::MenuOpenCloseEvent>
		{
			RE::BSEventNotifyControl ProcessEvent(const RE::MenuOpenCloseEvent* a_event, RE::BSTEventSource<RE::MenuOpenCloseEvent>*) override
			{
				if (a_event && a_event->menuName == RE::RaceSexMenu::MENU_NAME) {
					g_raceMenu.store(a_event->opening);
					logger::info("Locomotion: RaceMenu {}, player layer {}", a_event->opening ? "opened" : "closed", a_event->opening ? "off" : "back on");
				}
				return RE::BSEventNotifyControl::kContinue;
			}
		} g_menuSink;

		bool PlayerLayer() { return Active() && g_cfg.player && !g_raceMenu.load(std::memory_order_relaxed); }

		bool PlayerMotionOn() { return PlayerLayer() && g_cfg.playerMotion; }

		bool PlayerMotion() { return PlayerMotionOn(); }

		float Facing(RE::Actor* a_actor)
		{
			const float a = a_actor->data.angle.z;
			return a_actor->IsPlayerRef() && PlayerMotionOn() ? a + g_visYawOff.load(std::memory_order_relaxed) : a;
		}

		bool SmoothCamLoaded()
		{
			static const bool v = GetModuleHandleA("SmoothCam.dll") != nullptr;
			return v;
		}

		bool PoseFeatures(bool a_player)
		{
			const auto& c = g_cfg;
			return c.duck || c.stepOver || c.slopeLean || c.turnLean || c.headLead || c.landCrouch || c.stopSettle || c.touchWalls ||
			       (c.climb && (!a_player || (c.playerClimb && c.playerMotion))) || (a_player ? c.motionMatch && c.playerMotion : c.jumpPose);
		}

		bool FirstPerson()
		{
			const auto* cam = RE::PlayerCamera::GetSingleton();
			return cam && cam->IsInFirstPerson();
		}

		struct CharEntry
		{
			std::atomic<const void*>   ch{ nullptr };
			std::atomic<float>         k{ 1.0f };
			std::atomic<std::uint32_t> frame{ 0 };
			std::atomic_bool           probe{ false };
			std::atomic<std::uint32_t> dodge{ 0 };
		};
		CharEntry g_chars[128];

		CharEntry* Register(const void* a_ch)
		{
			for (auto& e : g_chars) {
				if (e.ch.load() == a_ch) {
					return &e;
				}
			}
			for (auto& e : g_chars) {
				const void* expected = nullptr;
				if (e.ch.compare_exchange_strong(expected, a_ch)) {
					e.k.store(1.0f);
					return &e;
				}
			}
			return nullptr;
		}

		CharEntry* FindChar(const void* a_ch)
		{
			for (auto& e : g_chars) {
				if (e.ch.load(std::memory_order_relaxed) == a_ch) {
					return &e;
				}
			}
			return nullptr;
		}

		void FindChars(RE::Actor* a_actor, const void* (&a_out)[2])
		{
			a_out[0] = a_out[1] = nullptr;
			RE::BSTSmartPointer<RE::BSAnimationGraphManager> mgr;
			if (a_actor->GetAnimationGraphManager(mgr) && mgr) {
				for (std::uint32_t i = 0; i < mgr->graphs.size() && i < 2; ++i) {
					if (mgr->graphs[i]) {
						a_out[i] = &mgr->graphs[i]->characterInstance;
					}
				}
			}
		}

		bool Has(const char* a_s, const char* a_needle)
		{
			for (; *a_s; ++a_s) {
				const char* p = a_s;
				const char* q = a_needle;
				while (*p && *q && std::tolower(static_cast<unsigned char>(*p)) == *q) {
					++p;
					++q;
				}
				if (!*q) {
					return true;
				}
			}
			return false;
		}

		bool IsLocoClip(const char* a_name)
		{
			if (!a_name) {
				return false;
			}
			if (Has(a_name, "jump") || Has(a_name, "fall") || Has(a_name, "land") || Has(a_name, "swim") || Has(a_name, "attack") ||
				Has(a_name, "turn") || Has(a_name, "bash") || Has(a_name, "idle")) {
				return false;
			}
			return Has(a_name, "run") || Has(a_name, "walk") || Has(a_name, "sprint") || Has(a_name, "jog") || Has(a_name, "sneak");
		}

		std::atomic<std::uint32_t> g_clipSeen[256];
		void LogClipName(const char* a_name)
		{
			if (!a_name || !g_cfg.log) {
				return;
			}
			std::uint32_t h = 2166136261u;
			for (const char* p = a_name; *p; ++p) {
				h = (h ^ static_cast<unsigned char>(*p)) * 16777619u;
			}
			h |= 1u;
			for (auto& s : g_clipSeen) {
				std::uint32_t cur = s.load();
				if (cur == h) {
					return;
				}
				if (cur == 0 && s.compare_exchange_strong(cur, h)) {
					logger::info("ANIM clip '{}' loco={}", a_name, IsLocoClip(a_name));
					return;
				}
			}
		}

		struct ClipUpdate
		{
			static void thunk(RE::hkbClipGenerator* a_this, const RE::hkbContext& a_ctx, float a_dt)
			{
				CrashLog::Crumb("clip hook");
				CharEntry* pe = a_ctx.character ? FindChar(a_ctx.character) : nullptr;
				if (pe && pe->probe.load(std::memory_order_relaxed)) {
					const auto* an = reinterpret_cast<const char*>(*reinterpret_cast<const std::uintptr_t*>(&a_this->animationName) & ~std::uintptr_t{ 1 });
					if (an && (Has(an, "dodge") || Has(an, "\\cgo\\") || Has(an, "/cgo/"))) {
						const auto f = g_frame.load(std::memory_order_relaxed);
						if (f - g_dodgeFrame.load(std::memory_order_relaxed) > 30 && g_cfg.log) {
							logger::info("ANIMDODGE clip '{}' (pose layer yields while it plays)", an);
						}
						g_dodgeFrame.store(f, std::memory_order_relaxed);
					}
				}
				const CharEntry* e = g_strideOn.load(std::memory_order_relaxed) ? pe : nullptr;
				if (!e) {
					func(a_this, a_ctx, a_dt);
					return;
				}
				const char* name = reinterpret_cast<const char*>(*reinterpret_cast<const std::uintptr_t*>(&a_this->name) & ~std::uintptr_t{ 1 });
				if (e->probe.load(std::memory_order_relaxed)) {
					LogClipName(name);
				}
				const float k = e->k.load(std::memory_order_relaxed);
				if (std::fabs(k - 1.0f) < 0.01f || !IsLocoClip(name)) {
					func(a_this, a_ctx, a_dt);
					return;
				}
				const float old = a_this->playbackSpeed;
				a_this->playbackSpeed = old * k;
				func(a_this, a_ctx, a_dt);
				a_this->playbackSpeed = old;
			}
			static inline REL::Relocation<decltype(thunk)> func;
		};

		RE::NiAVObject* FindBone(RE::NiAVObject* a_root, const char* a_name)
		{
			auto* n = a_root->GetObjectByName(RE::BSFixedString(a_name));
			if (n) {
				return n;
			}
			if (std::string_view(a_name) == kFoot[0]) {
				if (n = a_root->GetObjectByName(RE::BSFixedString("NPC L Foot [LLft ]")); n) {
					return n;
				}
			}
			std::string alt = a_name;
			if (const auto p = alt.find(" L "); p != std::string::npos) {
				if (const auto q = alt.find("[L", p); q != std::string::npos) {
					alt[q + 1] = 'R';
					if (n = a_root->GetObjectByName(RE::BSFixedString(alt.c_str())); n) {
						return n;
					}
				}
			}
			std::string base = a_name;
			if (const auto p = base.find(" ["); p != std::string::npos) {
				base.resize(p);
			}
			RE::NiAVObject* found = nullptr;
			std::function<void(RE::NiAVObject*)> walk = [&](RE::NiAVObject* o) {
				if (!o || found) {
					return;
				}
				std::string_view nm = o->name.c_str();
				if (nm == base || (nm.starts_with(base) && nm.substr(base.size()).starts_with(" ["))) {
					found = o;
					return;
				}
				if (auto* nd = o->AsNode()) {
					for (auto& ch : nd->GetChildren()) {
						walk(ch.get());
					}
				}
			};
			walk(a_root);
			return found;
		}

		void ResolveBones(Slot& s, RE::NiAVObject* a_root)
		{
			s.root.reset(a_root);
			s.b = Bones{};
			s.bonesFrame = g_frame.load();
			auto get = [&](const char* a_name) { return FindBone(a_root, a_name); };
			auto& b = s.b;
			b.com = get(kBoneCom);
			b.head = get(kHead);
			bool ok = b.com && b.head;
			for (int i = 0; i < 3; ++i) {
				b.spine[i] = get(kSpine[i]);
				ok = ok && b.spine[i];
			}
			bool arms = true;
			for (int i = 0; i < 2; ++i) {
				b.thigh[i] = get(kThigh[i]);
				b.calf[i] = get(kCalf[i]);
				b.foot[i] = get(kFoot[i]);
				ok = ok && b.thigh[i] && b.calf[i] && b.foot[i];
				b.uarm[i] = get(kUArm[i]);
				b.farm[i] = get(kFArm[i]);
				b.hand[i] = get(kHand[i]);
				arms = arms && b.uarm[i] && b.farm[i] && b.hand[i];
			}
			b.arms = arms;
			if (ok) {
				ok = b.spine[0]->parent && b.spine[1]->parent == b.spine[0] && b.spine[2]->parent == b.spine[1] && b.com->parent;
				for (int i = 0; i < 2 && ok; ++i) {
					ok = b.calf[i]->parent == b.thigh[i] && b.foot[i]->parent == b.calf[i];
				}
			}
			if (b.arms) {
				for (int i = 0; i < 2; ++i) {
					b.arms = b.arms && b.farm[i]->parent == b.uarm[i] && b.hand[i]->parent == b.farm[i];
				}
			}
			s.cnodes.clear();
			for (auto& j : s.jnodes) {
				j.clear();
			}
			if (ok) {
				if (const auto* clip = ClimbClip::Get()) {
					for (const auto& n : clip->bones) {
						s.cnodes.push_back(get(n.c_str()));
					}
				}
			}
			s.bonesBad = !ok;
			if (!ok && g_cfg.log) {
				logger::info("ANIM actor {:08X}: skeleton not recognised, layer off", s.id);
				std::string miss, all;
				for (const char* n : { kBoneCom, kHead, kSpine[0], kSpine[1], kSpine[2], kThigh[0], kThigh[1], kCalf[0], kCalf[1], kFoot[0], kFoot[1] }) {
					if (!get(n)) {
						miss += std::string(n) + " | ";
					}
				}
				int count = 0;
				std::function<void(RE::NiAVObject*, int)> walk = [&](RE::NiAVObject* o, int d) {
					if (!o || count > 150) {
						return;
					}
					if (auto* n = o->AsNode()) {
						all += std::format("{}{} ", std::string(d, '.'), o->name.c_str());
						++count;
						for (auto& ch : n->GetChildren()) {
							walk(ch.get(), d + 1);
						}
					}
				};
				walk(a_root, 0);
				logger::info("ANIM skeleton missing: {}", miss);
				logger::info("ANIM skeleton nodes: {}", all);
			}
		}

		float GroundAt(const Probe& p, float x, float y)
		{
			return p.gz - (p.n.x * (x - p.cx) + p.n.y * (y - p.cy)) / (std::max)(p.n.z, 0.3f);
		}
		bool Fresh(const Probe& p, float now, float x, float y)
		{
			return p.valid && now - p.t < 0.25f && std::hypot(x - p.cx, y - p.cy) < 70.0f;
		}

		void Pre(RE::Actor* a_actor, Slot& s)
		{
			CrashLog::Crumb("Pre");
			const auto frame = g_frame.load();
			std::lock_guard lk(s.m);
			if (frame - s.lastRegFrame > 30) {
				s.lastRegFrame = frame;
				CrashLog::Crumb("Pre: find chars");
				FindChars(a_actor, s.chars);
				for (auto* ch : s.chars) {
					if (ch) {
						if (auto* e = Register(ch)) {
							e->frame.store(frame);
							e->probe.store(s.player);
						}
					}
				}
			}
			for (auto* ch : s.chars) {
				if (ch) {
					if (auto* e = FindChar(ch)) {
						e->frame.store(frame);
						e->k.store(s.k);
					}
				}
			}
			if (g_cfg.speedSync && frame - s.overrideFrame <= 2) {
				static const RE::BSFixedString kDamped{ "SpeedDamped" };
				a_actor->SetGraphVariableFloat(kDamped, s.setEff);
			}
		}

		bool SafeFormID(RE::Actor* a_actor, RE::FormID& a_id)
		{
			if (!a_actor || reinterpret_cast<std::uintptr_t>(a_actor) < 0x10000) {
				return false;
			}
			__try {
				a_id = a_actor->GetFormID();
				return a_id != 0;
			} __except (EXCEPTION_EXECUTE_HANDLER) {
				return false;
			}
		}

		struct SpeedChannel
		{
			static void thunk(void* a_this, bool a_arg)
			{
				func(a_this, a_arg);
				if (!Active()) {
					return;
				}
				auto* actor = *reinterpret_cast<RE::Actor**>(static_cast<char*>(a_this) + 0x28);
				RE::FormID id = 0;
				if (!SafeFormID(actor, id)) {
					return;
				}
				CrashLog::Crumb("speed channel");
				auto slot = Find(id);
				if (!slot) {
					return;
				}
				auto&           s = *slot;
				float&          v = *reinterpret_cast<float*>(static_cast<char*>(a_this) + 0x18);
				const auto      frame = g_frame.load();
				std::lock_guard lk(s.m);
				s.cmdSpeed = v;
				s.cmdT = g_time.load();
				if (frame - s.drivenFrame <= 2) {
					v = s.driven;
					s.setEff = v;
					s.overrideFrame = frame;
					return;
				}
				if (s.player && g_cfg.playerMotion && s.mmBlockT > 0.25f && s.eligible) {
					v = 0.0f;
					return;
				}
				if (!g_cfg.speedSync || s.player || !s.eligible || frame - s.sampleFrame > 3 || v < 30.0f) {
					return;
				}
				const float r = s.speedH / v;
				if (r >= 0.85f || s.accel > 100.0f) {
					s.lowSince = -9.0f;
					return;
				}
				if (s.lowSince < 0.0f) {
					s.lowSince = s.cmdT;
				}
				if (s.cmdT - s.lowSince < 0.25f) {
					return;
				}
				s.checkRB = true;
				s.setCmd = v;
				v = s.speedH < 45.0f ? 0.0f : v * r / 0.85f;
				s.setEff = v;
				s.overrideFrame = frame;
				g_n.speedOvr.fetch_add(1, std::memory_order_relaxed);
			}
			static inline REL::Relocation<decltype(thunk)> func;
		};

		void ReachArm(const Bones& B, int i, const V3& a_target, const V3& a_pole, float w)
		{
			RE::NiAVObject* clav = B.uarm[i]->parent;
			const V3        sh = B.uarm[i]->world.translate;
			if (clav && clav->parent && w > 0.01f) {
				const float armLen = (B.farm[i]->world.translate - sh).Length() + (B.hand[i]->world.translate - B.farm[i]->world.translate).Length();
				const V3    to = a_target - sh;
				const float d = to.Length(), shift = Clamp((d - 0.7f * armLen) * 0.6f, 0.0f, 8.0f) * w;
				const V3    cp = clav->world.translate;
				if (shift > 0.2f && d > 1e-3f) {
					RotateWorld(clav, RotBetween(Norm(sh - cp, a_pole), Norm(sh + to * (shift / d) - cp, a_pole)));
					Refresh(clav);
				}
			}
			const V3 cur = B.hand[i]->world.translate;
			SolveTwoBone(B.uarm[i], B.farm[i], B.hand[i], cur * (1.0f - w) + a_target * w, a_pole);
		}

		void PostClimb(RE::Actor* a_actor, Slot& s)
		{
			CrashLog::Crumb("Post: climb");
			RE::NiAVObject* root = a_actor->Get3D(false);
			const auto*     clip = ClimbClip::Get(s.climbPose.clip, s.climbPose.rig);
			if (!root || !clip || s.bonesBad) {
				return;
			}
			const auto frame = g_frame.load();
			if (s.root.get() != root || frame - s.bonesFrame > 600) {
				ResolveBones(s, root);
				if (s.bonesBad) {
					return;
				}
			}
			const Bones& B = s.b;
			const auto&  cp = s.climbPose;
			const bool jumpClip = cp.clip >= ClimbClip::kJumpStart;
			auto&      nodes = jumpClip ? s.jnodes[cp.clip - ClimbClip::kJumpStart] : s.cnodes;
			if (jumpClip && nodes.empty()) {
				for (const auto& name : clip->bones) {
					nodes.push_back(FindBone(root, name.c_str()));
				}
			}
			if (nodes.size() != clip->bones.size()) {
				return;
			}
			for (std::size_t k = 0; k < nodes.size(); ++k) {
				auto* n = nodes[k];
				if (!n) {
					continue;
				}
				float cq[4], gq[4];
				clip->Rot(cp.t, k, cq);
				ToQuat(n->local.rotate, gq);
				const float sg = (gq[0] * cq[0] + gq[1] * cq[1] + gq[2] * cq[2] + gq[3] * cq[3]) < 0.0f ? -1.0f : 1.0f;
				float       r[4], len = 0;
				for (int c = 0; c < 4; ++c) {
					r[c] = gq[c] + (cq[c] * sg - gq[c]) * cp.w;
					len += r[c] * r[c];
				}
				len = 1.0f / std::sqrt(len);
				for (float& v : r) {
					v *= len;
				}
				n->local.rotate = FromQuat(r);
			}
			const auto fr = clip->Sample(cp.t);
			if (jumpClip) {
				B.com->local.translate.z += (fr.comZ - 71.5f) * cp.w;
			} else {
				B.com->local.translate.z += (fr.comZ - B.com->local.translate.z) * cp.w;
			}
			Refresh(B.com);
			if (B.arms && cp.contact > 0.01f) {
				const float ang = Facing(a_actor);
				const V3    right{ std::cos(ang), -std::sin(ang), 0.0f };
				for (int i = 0; i < 2; ++i) {
					ReachArm(B, i, cp.hand[i], right * (i == 0 ? -1.0f : 1.0f), cp.contact);
				}
			}
		}

		bool PostMotionMatch(RE::Actor* a_actor, Slot& s, RE::NiAVObject* a_root, float dt, bool a_want)
		{
			const auto* db = MotionMatch::Get();
			if (!db) {
				return false;
			}
			const auto&  c = g_cfg;
			const Bones& B = s.b;
			const M3&   P = B.com->parent->world.rotate;
			const float gYaw = std::atan2(P.entry[0][1], P.entry[1][1]);
			const bool  free = !s.drawn && !s.combat;
			const float des = s.speedH > 40.0f ? std::atan2(s.vel.x, s.vel.y) : gYaw;
			s.mmIdle = s.speedH < 8.0f ? s.mmIdle + dt : 0.0f;
			const bool graphMoves = s.cmdSpeed > 30.0f && g_time.load() - s.cmdT < 0.25f;
			const bool moving = s.speedH > 30.0f || graphMoves || (s.mmW > 0.01f && s.mmIdle < 0.6f);
			const bool steady = c.mmVanillaFwd && s.speedH > 30.0f && std::fabs(Wrap(gYaw - des)) < 0.2f && std::fabs(s.turnRate) < 0.6f && std::fabs(s.accel) < 150.0f;
			s.mmSteady = steady ? s.mmSteady + dt : 0.0f;
			const bool stopping = s.player && g_mmGoalOn && g_mmGoal.Length() < 0.05f;
			const bool walled = s.player && s.mmBlockT > 0.25f;
			if (s.player && FirstPerson()) {
				s.mmW = 0.0f;
				s.mm.playing = false;
				return false;
			}
			const bool on = a_want && s.mmOk && moving && !stopping && !walled;
			s.mmW = on ? (s.mm.playing ? Approach(s.mmW, 1.0f, 0.05f, dt) : 1.0f) : Approach(s.mmW, 0.0f, stopping || walled ? c.mmStopFade * 1.5f : 0.3f, dt);
			if (s.mmW < 0.01f && !on) {
				s.mm.playing = false;
				return false;
			}
			if (s.mmRoot != a_root || s.mmNodes.size() != db->nb) {
				s.mmRoot = a_root;
				s.mmNodes.clear();
				for (const auto& n : db->bones) {
					s.mmNodes.push_back(FindBone(a_root, n.c_str()));
				}
			}
			const std::uint64_t t0 = std::chrono::steady_clock::now().time_since_epoch().count();
			s.mmGq.assign(db->nb * 4, 0.0f);
			for (std::uint32_t k = 0; k < db->nb; ++k) {
				float gq[4]{ 0, 0, 0, 1 };
				if (s.mmNodes[k]) {
					ToQuat(s.mmNodes[k]->local.rotate, gq);
				}
				std::copy(gq, gq + 4, &s.mmGq[k * 4]);
			}
			const float ang = gYaw;
			const V3    fv{ std::sin(ang), std::cos(ang), 0.0f }, rv{ std::cos(ang), -std::sin(ang), 0.0f };
			MotionMatch::Input in;
			in.vx = s.vel.Dot(rv);
			in.vy = s.vel.Dot(fv);
			in.omega = s.turnRate;
			if (s.player && g_mmGoalOn) {
				const float im = g_mmMag;
				if (im > 0.3f && s.speedH > 30.0f) {
					s.mmUnit += (s.speedH / im - s.mmUnit) * (1.0f - std::exp(-dt / 0.3f));
				}
				const V3 g = s.mmBlockT > 0.25f ? V3{} : g_mmGoal * s.mmUnit;
				in.haveGoal = true;
				in.gx = g.Dot(rv);
				in.gy = g.Dot(fv);
			}
			for (int k = 0; k < 3; ++k) {
				in.face[k] = s.turnRate * 0.5f * (1.0f - std::exp(-(k + 1) / 3.0f / 0.5f));
			}
			in.preferVanilla = s.mmSteady > 0.2f;
			if (!s.mm.playing || std::fabs(Wrap(des - s.mmDes)) > 0.5f) {
				in.force = s.mm.playing;
				s.mmDes = des;
			}
			const V3 p0 = a_actor->GetPosition();
			for (int i = 0; i < 2; ++i) {
				const V3 d = B.foot[i]->world.translate - p0;
				in.foot[i][0] = d.Dot(rv);
				in.foot[i][1] = d.Dot(fv);
				in.foot[i][2] = d.z;
			}
			in.haveFeet = true;
			in.graphQ = s.mmGq.data();
			in.graphZ = B.com->local.translate.z;
			const int sw0 = s.mm.switches, se0 = s.mm.searches;
			const bool fresh = !s.mm.playing;
			MotionMatch::Params prm{ c.mmSearchHz, c.mmHalflife, c.mmMargin, c.mmRateMin, c.mmRateMax, c.mmVanillaBias, c.mmCadence };
			MotionMatch::Step(*db, s.mm, in, prm, dt);
			g_n.mmSearch.fetch_add(static_cast<std::uint64_t>(s.mm.searches - se0), std::memory_order_relaxed);
			g_n.mmSwitch.fetch_add(static_cast<std::uint64_t>(s.mm.switches - sw0), std::memory_order_relaxed);
			if (fresh && c.log) {
				logger::info("ANIMMM start {} speed={:.0f} frame={} drawn={}", a_actor->GetName(), s.speedH, s.mm.frame, s.drawn);
			}
			s.mmLogT += dt;
			if (c.log && s.mmLogT > 1.0f) {
				s.mmLogT = 0.0f;
				constexpr float kD = 180.0f / kPi;
				logger::info("ANIMMM yaw {} free={} w={:.2f} speed={:.0f} move-graph={:.0f} turn={:.0f}/s steady={:.1f} vanilla={} rate={:.2f} stride={:.2f} switches={}", a_actor->GetName(), free, s.mmW,
					s.speedH, Wrap(des - gYaw) * kD, s.turnRate * kD, s.mmSteady, s.mm.vanilla, s.mm.rate, s.mm.stride, s.mm.switches);
			}
			const float w = s.mmW;
			Refresh(B.com);
			const M3 headW = B.head ? B.head->world.rotate : M3{};
			const V3 upG = B.head && B.spine[0] ? B.head->world.translate - B.spine[0]->world.translate : V3{ 0, 0, 1 };
			V3       armG[2]{};
			for (int i = 0; i < 2 && B.arms; ++i) {
				armG[i] = Norm(B.farm[i]->world.translate - B.uarm[i]->world.translate, V3{ 0, 0, -1 });
			}
			const bool blocked = graphMoves && s.speedH < 0.5f * s.cmdSpeed;
			s.mmArmW = Approach(s.mmArmW, !s.drawn ? 1.0f : 0.0f, 0.2f, dt);
			for (std::uint32_t k = 0; k < db->nb; ++k) {
				auto* n = s.mmNodes[k];
				if (!n) {
					continue;
				}
				const std::string_view nm = db->bones[k];
				const bool upper = nm.find("Clavicle") != nm.npos || nm.find("UpperArm") != nm.npos || nm.find("Forearm") != nm.npos || nm.find("Hand") != nm.npos ||
				                   nm.find("Neck") != nm.npos || nm.find("Head") != nm.npos || nm.find("Twist") != nm.npos;
				const bool spine = nm.find("Spine") != nm.npos;
				if ((upper && !c.mmArms && s.mmArmW < 0.01f) || (spine && s.drawn)) {
					continue;
				}
				const float part = nm.find("Clavicle") != nm.npos ? c.mmClavicle : nm.find("Forearm") != nm.npos ? c.mmForearm : nm.find("Hand") != nm.npos ? c.mmHand :
				                   nm.find("UpperArm") != nm.npos || nm.find("Upperarm") != nm.npos ? c.mmUpperArm : 1.0f;
				const float bk = Smooth(s.mmBlockT, 0.25f, 0.6f);
				const float wb = (upper && !c.mmArms ? w * s.mmArmW : w) * (part + (1.0f - part) * bk);
				const float* g = &s.mmGq[k * 4];
				const float* o = &s.mm.out[k * 4];
				const float  sg = (g[0] * o[0] + g[1] * o[1] + g[2] * o[2] + g[3] * o[3]) < 0.0f ? -1.0f : 1.0f;
				float        r[4], len = 0;
				for (int j = 0; j < 4; ++j) {
					r[j] = g[j] + (o[j] * sg - g[j]) * wb;
					len += r[j] * r[j];
				}
				len = 1.0f / std::sqrt(len);
				for (float& v : r) {
					v *= len;
				}
				n->local.rotate = FromQuat(r);
			}
			B.com->local.translate.z += (s.mm.outZ - in.graphZ) * w;
			RefreshXform(B.com);
			for (int i = 0; i < 2 && B.arms && s.mmArmW > 0.01f; ++i) {
				const V3 d = Norm(B.farm[i]->world.translate - B.uarm[i]->world.translate, armG[i]);
				const V3 t = Norm(d + rv * ((armG[i].Dot(rv) - d.Dot(rv)) * c.mmArmLateral * s.mmArmW * w), d);
				RotateWorld(B.uarm[i], RotBetween(d, t));
				Refresh(B.uarm[i]);
			}
			if (B.head && B.spine[0] && c.mmLeanCut > 0.0f) {
				const V3    up{ 0, 0, 1 }, d = B.head->world.translate - B.spine[0]->world.translate;
				const float pm = std::atan2(d.Dot(fv), d.Dot(up)), pg = std::atan2(upG.Dot(fv), upG.Dot(up));
				const float cut = Clamp(pm - pg, -0.6f, 0.6f) * c.mmLeanCut * w;
				if (std::fabs(cut) > 0.002f) {
					RotateWorld(B.spine[0], AxisAngle(rv, cut));
					Refresh(B.spine[0]);
				}
			}
			if (std::fabs(s.mm.stride - 1.0f) > 0.01f && s.speedH > 35.0f) {
				const V3 vd = Norm(V3{ s.vel.x, s.vel.y, 0.0f }, fv);
				for (int i = 0; i < 2; ++i) {
					const V3    P = B.foot[i]->world.translate;
					const float d = Clamp((P - B.com->world.translate).Dot(vd) * (s.mm.stride - 1.0f) * w, -25.0f, 25.0f);
					if (std::fabs(d) > 0.2f) {
						const M3 fr = B.foot[i]->world.rotate;
						SolveTwoBone(B.thigh[i], B.calf[i], B.foot[i], P + vd * d, fv);
						B.foot[i]->local.rotate = B.calf[i]->world.rotate.Transpose() * fr;
						Refresh(B.foot[i]);
					}
				}
			}
			if (B.head && B.head->parent && !c.mmArms && !blocked) {
				float a[4], b[4], r[4], len = 0;
				ToQuat(B.head->world.rotate, a);
				ToQuat(headW, b);
				const float sg = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3] < 0.0f ? -1.0f : 1.0f;
				const float hw = w;
				for (int j = 0; j < 4; ++j) {
					r[j] = a[j] + (b[j] * sg - a[j]) * hw;
					len += r[j] * r[j];
				}
				for (float& v : r) {
					v /= std::sqrt(len);
				}
				B.head->local.rotate = B.head->parent->world.rotate.Transpose() * FromQuat(r);
				Refresh(B.head);
			}
			g_n.mmNs.fetch_add(static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count() - t0), std::memory_order_relaxed);
			g_n.mmPosts.fetch_add(1, std::memory_order_relaxed);
			return w > 0.5f;
		}

		void PostLocked(RE::Actor* a_actor, Slot& s, float dt)
		{
			const auto& c = g_cfg;
			const auto  frame = g_frame.load();
			const float now = g_time.load();
			if (frame - s.climbFrame <= 2 && s.climbPose.w > 0.001f) {
				PostClimb(a_actor, s);
				return;
			}
			const bool  fresh = frame - s.sampleFrame < 4;
			const bool  want = fresh && s.eligible && !s.bonesBad && !(s.player && FirstPerson());
			s.gate = Approach(s.gate, want ? 1.0f : 0.0f, 0.12f, dt);
			if (s.gate < 0.01f && !want) {
				s.bank = s.head = s.lean = s.slopeP = s.duck = s.dip = s.dipV = 0.0f;
				for (auto& f : s.foot) {
					f.haveAnim = false;
					f.lift = 0.0f;
				}
				return;
			}
			if (s.bonesBad) {
				return;
			}
			CrashLog::Crumb("Post: bones");
			RE::NiAVObject* root = a_actor->Get3D(false);
			if (!root) {
				return;
			}
			if (s.root.get() != root || frame - s.bonesFrame > 600) {
				ResolveBones(s, root);
				if (s.bonesBad) {
					return;
				}
			}
			const Bones& B = s.b;
			const bool   mmOn = c.motionMatch && c.playerMotion && s.player && PostMotionMatch(a_actor, s, root, dt, want);

			const float speed = s.speedH;
			const float h = s.height;
			const float moveW = Smooth(speed, 50.0f, 140.0f);
			constexpr float kDeg = kPi / 180.0f;

			const float ang = Facing(a_actor);
			float bankT = 0, headT = 0, slopeT = 0, duckT = 0, leanT = 0;
			if (want) {
				if (c.turnLean) {
					bankT = Clamp(std::atan2(speed / 70.0f * s.turnRate, 9.81f) * c.bankGain, -c.bankMaxDeg * kDeg, c.bankMaxDeg * kDeg) * moveW;
				}
				if (c.headLead) {
					headT = Clamp(s.turnRate * c.headLeadSec, -c.headMaxDeg * kDeg, c.headMaxDeg * kDeg) * moveW;
				}
				if (c.slopeLean) {
					const float vs = std::atan2(s.vel.z, (std::max)(speed, 1.0f));
					const float up = Clamp(s.slopeUp, vs - 0.12f, vs + 0.12f), minR = c.slopeMinDeg * kDeg, w = Smooth(speed, 30.0f, 100.0f);
					if (up > minR) {
						slopeT = (std::min)((up - minR) * c.slopeGain, c.slopeMaxDeg * kDeg) * w;
					} else if (up < -minR * 1.2f) {
						slopeT = -(std::min)((-up - minR * 1.2f) * c.slopeBackGain, c.slopeBackMaxDeg * kDeg) * w;
					}
				}
				if (c.duck) {
					duckT = s.duckNeed;
				}
			}
			if (mmOn) {
				bankT = leanT = 0.0f;
			}
			s.bank = Approach(s.bank, bankT, 0.12f, dt);
			s.head = Approach(s.head, headT, 0.10f, dt);
			s.lean = Approach(s.lean, leanT, 0.15f, dt);
			s.slopeP = Approach(s.slopeP, slopeT, 0.20f, dt);
			s.duck = Approach(s.duck, duckT, duckT > s.duck ? 0.08f : 0.30f, dt);

			{
				const float aA = Facing(a_actor);
				const V3    fv{ std::sin(aA), std::cos(aA), 0.0f }, rv{ std::cos(aA), -std::sin(aA), 0.0f };
				const bool  live = want && c.touchWalls && B.arms && now - s.touchT < 0.25f;
				for (int i = 0; i < 2; ++i) {
					const V3    sh = B.uarm[i]->world.translate;
					const float armLen = (B.farm[i]->world.translate - sh).Length() + (B.hand[i]->world.translate - B.farm[i]->world.translate).Length();
					auto        ahead = [&](const V3& p) { return (p - sh).Dot(fv); };
					auto        reach = [&](const V3& p) {
						return (std::min)(Clamp((armLen * 1.05f - (p - sh).Length()) / (0.3f * armLen), 0.0f, 1.0f), Smooth(ahead(p), 4.0f, 18.0f));
					};
					V3    tg{}, tn{ 0, 0, 1 };
					float best = 0.0f;
					if (live) {
						auto consider = [&](const Slot::Touch& tc, const V3& target) {
							const float a = ahead(target);
							if (tc.valid && a > 0.4f * armLen && reach(target) > 0.5f && a > best) {
								best = a;
								tg = target;
								tn = tc.n;
							}
						};
						consider(s.touch[i == 0 ? 4 : 5], s.touch[i == 0 ? 4 : 5].p + s.touch[i == 0 ? 4 : 5].n * 2.0f);
						const auto& f = s.touch[0];
						consider(f, f.p + f.n * 2.0f + Norm(V3{ f.n.y, -f.n.x, 0.0f }, rv) * (i == 0 ? -14.0f : 14.0f));
						const auto& ce = s.touch[3];
						consider(ce, ce.p + ce.n * 2.0f + rv * (i == 0 ? -12.0f : 12.0f));
					}
					if (s.touchPlant[i] && (!live || reach(s.touchP[i]) < 0.2f)) {
						s.touchPlant[i] = false;
					}
					if (!s.touchPlant[i] && best > 0.0f && s.touchW[i] < 0.12f) {
						s.touchP[i] = tg;
						s.touchN[i] = tn;
						s.touchPlant[i] = true;
					}
					s.touchLift[i] = Approach(s.touchLift[i], s.touchPlant[i] ? 0.0f : 1.0f, 0.12f, dt);
					s.touchW[i] = Approach(s.touchW[i], s.touchPlant[i] ? reach(s.touchP[i]) : 0.0f, s.touchPlant[i] ? 0.25f : 0.2f, dt);
				}
			}

			if (s.dipKick > 0.0f) {
				s.dipV += 38.0f * s.dipKick;
				s.dipKick = 0.0f;
			}
			{
				constexpr float w = 14.0f;
				s.dipV += (-w * w * s.dip - 2.0f * w * s.dipV) * dt;
				s.dip += s.dipV * dt;
				if (s.dip < 0.0f) {
					s.dip = 0.0f;
					s.dipV = (std::max)(s.dipV, 0.0f);
				}
				s.dip = (std::min)(s.dip, 1.5f);
			}

			s.stepVis = (std::max)(0.0f, s.stepVis - 220.0f * dt);
			float drop = s.gate * (s.duck * 0.5f + s.dip * c.dipDrop + (c.stepOver ? s.stepVis : 0.0f));
			drop = (std::min)(drop, 0.28f * h);
			float pitch = s.gate * (s.lean + s.slopeP + std::asin(Clamp(s.duck * 0.5f / (0.45f * h), 0.0f, 0.6f)) + s.dip * c.dipPitchDeg * kDeg);
			pitch = Clamp(pitch, -0.3f, 0.75f);
			const float bank = s.gate * s.bank, headYaw = s.gate * s.head;

			const bool moving = speed > 35.0f;
			const bool  feetOn = s.gate > 0.01f && moving && c.stepOver;
			const bool  torsoOn = std::fabs(bank) > 0.003f || std::fabs(pitch) > 0.003f || drop > 0.1f || std::fabs(headYaw) > 0.003f;
			const bool touchOn = B.arms && (s.touchW[0] > 0.02f || s.touchW[1] > 0.02f);
			const bool levelOn = s.player && B.head && c.headLevel > 0.0f && ((c.motionMatch && c.playerMotion) || c.slopeLean || c.turnLean);
			if (!torsoOn && !feetOn && !touchOn && !levelOn) {
				for (auto& f : s.foot) {
					f.haveAnim = false;
					f.lift = 0.0f;
				}
				return;
			}

			CrashLog::Crumb("Post: refresh");
			V3 before = B.foot[0]->world.translate;
			Refresh(B.com);
			g_n.staleMax.store((std::max)(g_n.staleMax.load(), (B.foot[0]->world.translate - before).Length()), std::memory_order_relaxed);

			M3 footRot[2];
			for (int i = 0; i < 2; ++i) {
				auto& F = s.foot[i];
				F.anim = B.foot[i]->world.translate;
				F.haveAnim = true;
				footRot[i] = B.foot[i]->world.rotate;
			}
			s.animT = now;

			const V3    fwd{ std::sin(ang), std::cos(ang), 0.0f }, right{ std::cos(ang), -std::sin(ang), 0.0f };

			CrashLog::Crumb("Post: feet");
			V3   T[2];
			bool ik[2]{};
			for (int i = 0; i < 2; ++i) {
				Foot&    F = s.foot[i];
				const V3 P = F.anim;
				T[i] = P;
				if (!feetOn) {
					F.lift = Approach(F.lift, 0.0f, 0.1f, dt);
					continue;
				}
				const bool  uv = Fresh(F.under, now, P.x, P.y);
				const float gzU = uv ? GroundAt(F.under, P.x, P.y) : 0.0f;
				float       liftT = 0.0f;
				if (uv && Fresh(F.ahead, now, F.ahead.cx, F.ahead.cy) && speed > 40.0f) {
					const float need = F.ahead.gz + c.footClear - P.z;
					if (need > 0.0f && F.ahead.gz - gzU > 3.0f) {
						liftT = (std::min)(need, c.stepMax);
					}
				}
				F.lift = Approach(F.lift, liftT, liftT > F.lift ? 0.04f : 0.10f, dt);
				if (F.lift > 0.2f) {
					T[i].z += F.lift;
					ik[i] = true;
				}
			}

			CrashLog::Crumb("Post: spine");
			if (torsoOn) {
				if (drop > 0.05f) {
					MoveWorld(B.com, B.com->world.translate - V3{ 0, 0, drop });
				}
				constexpr float wts[3] = { 0.30f, 0.35f, 0.35f };
				M3              prev = B.spine[0]->parent->world.rotate;
				for (int k = 0; k < 3; ++k) {
					const M3 d = AxisAngle(right, -pitch * wts[k]) * AxisAngle(fwd, bank * wts[k]);
					const M3 L = B.spine[k]->local.rotate;
					B.spine[k]->local.rotate = prev.Transpose() * d * prev * L;
					prev = d * prev * L;
				}
				Refresh(B.com);
			}

			CrashLog::Crumb("Post: legs");
			for (int i = 0; i < 2; ++i) {
				if (ik[i] || drop > 0.1f) {
					SolveTwoBone(B.thigh[i], B.calf[i], B.foot[i], T[i], fwd);
					B.foot[i]->local.rotate = B.calf[i]->world.rotate.Transpose() * footRot[i];
					Refresh(B.foot[i]);
				}
			}

			CrashLog::Crumb("Post: hands");
			if (touchOn) {
				for (int i = 0; i < 2; ++i) {
					const float w = Smooth(s.gate * s.touchW[i], 0.0f, 1.0f);
					if (w < 0.02f) {
						continue;
					}
					ReachArm(B, i, s.touchP[i] + s.touchN[i] * (14.0f * s.touchLift[i]), right * (i == 0 ? -1.0f : 1.0f), w);
				}
			}

			CrashLog::Crumb("Post: head");
			if (std::fabs(pitch) > 0.003f || std::fabs(headYaw) > 0.003f) {
				RotateWorld(B.head, AxisAngle(V3{ 0, 0, 1 }, -headYaw) * AxisAngle(right, 0.6f * pitch));
				Refresh(B.head);
			}
			if (levelOn) {
				const M3& R = B.head->world.rotate;
				V3        ax[3];
				for (int k = 0; k < 3; ++k) {
					ax[k] = { R.entry[0][k], R.entry[1][k], R.entry[2][k] };
				}
				int ui = 0;
				for (int k = 1; k < 3; ++k) {
					ui = std::fabs(ax[k].z) > std::fabs(ax[ui].z) ? k : ui;
				}
				int fi = ui == 0 ? 1 : 0;
				for (int k = 0; k < 3; ++k) {
					fi = k != ui && std::fabs(ax[k].Dot(fwd)) > std::fabs(ax[fi].Dot(fwd)) ? k : fi;
				}
				const V3 u = ax[ui] * (ax[ui].z < 0.0f ? -1.0f : 1.0f), f = ax[fi] * (ax[fi].Dot(fwd) < 0.0f ? -1.0f : 1.0f);
				V3       zp = V3{ 0, 0, 1 } - f * f.z;
				if (zp.Length() > 0.3f) {
					zp = zp * (1.0f / zp.Length());
					const float roll = std::atan2(u.Cross(zp).Dot(f), u.Dot(zp));
					if (std::fabs(roll) > 0.003f) {
						RotateWorld(B.head, AxisAngle(f, roll * c.headLevel * s.gate));
						Refresh(B.head);
					}
				}
			}
		}

		struct PoseStats
		{
			std::atomic_int main{ 0 }, job{ 0 }, again{ 0 }, edited{ 0 };
		} g_pose;

		void PoseNodes(const Slot& s, std::vector<RE::NiAVObject*>& a_out)
		{
			const Bones& B = s.b;
			a_out.clear();
			for (auto* n : { B.com, B.spine[0], B.spine[1], B.spine[2], B.head, B.thigh[0], B.thigh[1], B.calf[0], B.calf[1], B.foot[0], B.foot[1], B.uarm[0],
					 B.uarm[1], B.farm[0], B.farm[1], B.hand[0], B.hand[1] }) {
				if (n) {
					a_out.push_back(n);
				}
			}
			for (auto* n : s.mmNodes) {
				if (n && std::find(a_out.begin(), a_out.end(), n) == a_out.end()) {
					a_out.push_back(n);
				}
			}
		}

		bool SameLocal(const RE::NiTransform& a, const RE::NiTransform& b)
		{
			return std::memcmp(&a.rotate, &b.rotate, sizeof(a.rotate)) == 0 && std::memcmp(&a.translate, &b.translate, sizeof(a.translate)) == 0;
		}

		bool Recommit(Slot& s)
		{
			bool diff = false;
			for (auto& [n, t] : s.commit) {
				if (!SameLocal(n->local, t)) {
					n->local = t;
					diff = true;
				}
			}
			if (diff && s.root) {
				Refresh(s.root.get());
			}
			return diff;
		}

		void Post(RE::Actor* a_actor, Slot& s, float dt)
		{
			static std::atomic_int threadLogs{ 0 };
			const bool             onMain = std::this_thread::get_id() == g_mainThread;
			if (!s.player && g_cfg.log && !onMain && threadLogs.fetch_add(1) < 3) {
				logger::info("ANIM npc pose pass runs off the main thread");
			}
			const auto      t0 = std::chrono::steady_clock::now();
			std::lock_guard lk(s.m);
			const auto      frame = g_frame.load();
			const float     now = g_time.load();
			if (s.player) {
				(onMain ? g_pose.main : g_pose.job).fetch_add(1, std::memory_order_relaxed);
			}
			if (s.postFrame == frame) {
				if (s.commitFrame == frame && Recommit(s) && s.player) {
					g_pose.again.fetch_add(1, std::memory_order_relaxed);
				}
				return;
			}
			s.postFrame = frame;
			if (!(dt > 0.0f)) {
				dt = now - s.lastPostT;
			}
			s.lastPostT = now;
			if (!(dt > 1e-4f && dt < 0.25f)) {
				return;
			}
			if (s.checkRB) {
				s.checkRB = false;
				static std::atomic_int rbLogs{ 0 };
				if (g_cfg.log && rbLogs.fetch_add(1) < 25) {
					float sp = -1, sd = -1;
					a_actor->GetGraphVariableFloat(RE::BSFixedString("Speed"), sp);
					a_actor->GetGraphVariableFloat(RE::BSFixedString("SpeedDamped"), sd);
					logger::info("ANIM speedvar {:08X} cmd={:.0f} set={:.0f} measured={:.0f} after graph: Speed={:.0f} SpeedDamped={:.0f}", s.id,
						s.setCmd, s.setEff, s.speedH, sp, sd);
				}
			}
			static thread_local std::vector<RE::NiAVObject*> nodes;
			static thread_local std::vector<RE::NiTransform> pre;
			auto* const                                      root = a_actor->Get3D(false);
			const bool                                       snap = root && s.root.get() == root && !s.bonesBad && s.b.com;
			if (snap) {
				PoseNodes(s, nodes);
				pre.clear();
				for (auto* n : nodes) {
					pre.push_back(n->local);
				}
			}
			RE::NiAVObject* bend[2]{};
			M3              bendG[2];
			if (s.player && snap && s.b.head && s.b.head->parent && g_cfg.neckMaxBendDeg > 0.0f) {
				bend[0] = s.b.head->parent;
				bend[1] = s.b.head;
				for (int i = 0; i < 2; ++i) {
					bendG[i] = bend[i]->local.rotate;
				}
			}
			PostLocked(a_actor, s, dt);
			if (bend[0] && s.root.get() == root) {
				const float     maxA = g_cfg.neckMaxBendDeg * kPi / 180.0f;
				float           dev[2]{};
				bool            any = false;
				for (int i = 0; i < 2; ++i) {
					float g[4], q[4];
					ToQuat(bendG[i], g);
					ToQuat(bend[i]->local.rotate, q);
					float d = g[0] * q[0] + g[1] * q[1] + g[2] * q[2] + g[3] * q[3];
					if (d < 0.0f) {
						for (float& v : q) {
							v = -v;
						}
						d = -d;
					}
					dev[i] = 2.0f * std::acos((std::min)(d, 1.0f));
					if (dev[i] > maxA) {
						const float t = maxA / dev[i], th = dev[i] * 0.5f, sn = std::sin(th);
						const float wa = std::sin((1.0f - t) * th) / sn, wb = std::sin(t * th) / sn;
						float       r[4];
						for (int j = 0; j < 4; ++j) {
							r[j] = g[j] * wa + q[j] * wb;
						}
						bend[i]->local.rotate = FromQuat(r);
						any = true;
					}
				}
				if (any) {
					Refresh(bend[0]);
				}
				static float logT = 0.0f, maxDev[2]{};
				static int   limited = 0, n = 0;
				maxDev[0] = (std::max)(maxDev[0], dev[0]);
				maxDev[1] = (std::max)(maxDev[1], dev[1]);
				limited += any;
				++n;
				if (g_cfg.log && (logT += dt) > 2.0f) {
					logger::info("ANIMNECK last 2 s: biggest bend off vanilla (to the parent bone) neck {:.0f} head {:.0f} deg, limited to {:.0f} in {} of {} frames",
						maxDev[0] * 57.3f, maxDev[1] * 57.3f, g_cfg.neckMaxBendDeg, limited, n);
					logT = maxDev[0] = maxDev[1] = 0.0f;
					limited = n = 0;
				}
			}
			s.commit.clear();
			if (snap && s.root.get() == root) {
				for (std::size_t i = 0; i < nodes.size(); ++i) {
					if (!SameLocal(nodes[i]->local, pre[i])) {
						for (auto* n : nodes) {
							s.commit.emplace_back(n, n->local);
						}
						s.commitFrame = frame;
						if (s.player) {
							g_pose.edited.fetch_add(1, std::memory_order_relaxed);
						}
						break;
					}
				}
			}
			g_n.postNs.fetch_add(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count(),
				std::memory_order_relaxed);
			g_n.posts.fetch_add(1, std::memory_order_relaxed);
		}

		void PrUnhook(PlayerRedirect& pr, const char* a_why)
		{
			if (!pr.hooked) {
				return;
			}
			std::lock_guard lk(pr.m);
			for (const auto& r : pr.redir) {
				auto& arr = r.graph->boneNodes;
				if (static_cast<std::uint32_t>(r.idx) < arr.size() && arr[r.idx].node == r.dummy) {
					r.real->local = r.dummy->local;
					arr[r.idx].node = r.origNode;
					arr[r.idx].unk08 = r.origSlot;
				}
			}
			if (pr.root) {
				Refresh(pr.root.get());
			}
			pr.redir.clear();
			pr.graphRefs.clear();
			pr.dummies.clear();
			pr.root.reset();
			pr.hooked = false;
			if (g_cfg.log && pr.player) {
				logger::info("PLAYERHOOK unhooked ({})", a_why);
			}
		}

		bool Below(const RE::NiAVObject* a_n, const RE::NiAVObject* a_root)
		{
			for (; a_n; a_n = a_n->parent) {
				if (a_n == a_root) {
					return true;
				}
			}
			return false;
		}

		void PrHook(PlayerRedirect& pr, RE::Actor* a, RE::NiAVObject* a_root)
		{
			RE::BSTSmartPointer<RE::BSAnimationGraphManager> mgr;
			if (!a->GetAnimationGraphManager(mgr) || !mgr) {
				return;
			}
			std::vector<PlayerRedirect::Entry>           found;
			std::unordered_map<RE::NiNode*, RE::NiNode*> made;
			for (std::uint32_t gi = 0; gi < mgr->graphs.size(); ++gi) {
				auto* g = mgr->graphs[gi].get();
				if (!g || g->boneNodes.size() == 0 || g->boneNodes.size() > 1024 || !g->boneNodes.data()) {
					continue;
				}
				auto& arr = g->boneNodes;
				for (std::uint32_t j = 0; j < arr.size(); ++j) {
					RE::NiNode* real = arr[j].node;
					const auto  slot = static_cast<std::int32_t>(arr[j].unk08);
					if (real && slot >= 0) {
						auto* tree = netimmerse_cast<RE::BSFlattenedBoneTree*>(real);
						if (!tree || static_cast<std::uint32_t>(slot) >= tree->numBones || !tree->boneEntries) {
							continue;
						}
						auto* n = tree->boneEntries[slot].node;
						real = n ? n->AsNode() : nullptr;
					}
					if (real && Below(real, a_root) && std::string_view(real->name.c_str()).starts_with("NPC ")) {
						found.push_back({ g, static_cast<int>(j), real, nullptr, arr[j].node, arr[j].unk08 });
					}
				}
			}
			if (found.empty()) {
				pr.failedRoot = a_root;
				if (pr.player) {
					logger::info("PLAYERHOOK not redirected: no graph bone entries in the third-person skeleton ({} graphs)", mgr->graphs.size());
				}
				return;
			}
			std::vector<RE::NiPointer<RE::NiNode>> dummies;
			for (auto& r : found) {
				auto& d = made[r.real];
				if (!d) {
					RE::NiPointer<RE::NiNode> n(RE::NiNode::Create(0));
					if (!n) {
						pr.failedRoot = a_root;
						return;
					}
					n->local = r.real->local;
					dummies.push_back(n);
					d = n.get();
				}
				r.dummy = d;
			}
			for (std::uint32_t gi = 0; gi < mgr->graphs.size(); ++gi) {
				if (mgr->graphs[gi]) {
					pr.graphRefs.push_back(mgr->graphs[gi]);
				}
			}
			for (auto& r : found) {
				r.graph->boneNodes[r.idx].node = r.dummy;
				r.graph->boneNodes[r.idx].unk08 = 0xFFFFFFFFu;
			}
			std::lock_guard lk(pr.m);
			pr.redir = std::move(found);
			pr.dummies = std::move(dummies);
			pr.root.reset(a_root);
			pr.pose.clear();
			for (const auto& r : pr.redir) {
				pr.pose.push_back(r.real->local);
			}
			pr.player = a->IsPlayerRef();
			pr.hooked = true;
			if (g_cfg.log && pr.player) {
				std::string names;
				for (const auto& [real, d] : made) {
					names += std::string(real->name.c_str()) + "|";
				}
				logger::info("PLAYERHOOK redirected {} graph bone entries ({} nodes, {} graphs): {}", pr.redir.size(), made.size(), pr.graphRefs.size(), names);
			}
		}

		bool PrAfterGraph(RE::Actor* a, Slot& s)
		{
			auto& pr = s.pr;
			if (pr.inPass.load()) {
				g_passOverlap.fetch_add(1, std::memory_order_relaxed);
			}
			auto*      root = a->Get3D(false);
			const bool want = Active() && (s.player ? PlayerLayer() : g_cfg.npcs) && PoseFeatures(s.player) && root && !a->IsDead() && !a->IsInRagdollState() &&
			                  !SpiderLegs::Has(s.id) && g_frame.load() - g_preRenderFrame.load() <= 3;
			if (pr.hooked) {
				const char* why = !want ? "off/dead/ragdoll/no pre-render" : root != pr.root.get() ? "3D changed" : nullptr;
				if (!why) {
					for (const auto& r : pr.redir) {
						if (static_cast<std::uint32_t>(r.idx) >= r.graph->boneNodes.size() || r.graph->boneNodes[r.idx].node != r.dummy) {
							why = "bone table rebuilt";
							break;
						}
					}
				}
				if (why) {
					PrUnhook(pr, why);
					return false;
				}
				std::lock_guard lk(pr.m);
				for (std::size_t i = 0; i < pr.redir.size(); ++i) {
					pr.pose[i] = pr.redir[i].dummy->local;
				}
				return true;
			}
			if (want && root != pr.failedRoot) {
				PrHook(pr, a, root);
			}
			return pr.hooked;
		}

		Slot::~Slot()
		{
			if (pr.hooked) {
				PrUnhook(pr, "slot dropped");
			}
		}

		using JobFn = void (*)(std::uint32_t);
		JobFn g_jobOrig = nullptr;

		void JobThunk(std::uint32_t a_handle)
		{
			CrashLog::Crumb("anim job");
			RE::NiPointer<RE::Actor> hold;
			RE::Actor*               a = nullptr;
			std::shared_ptr<Slot>    slot;
			{
				RE::ActorHandle h;
				std::memcpy(&h, &a_handle, sizeof(a_handle));
				hold = h.get();
				a = hold.get();
				if (a) {
					slot = Find(a->GetFormID());
				}
			}
			const bool on = slot && Active();
			if (slot) {
				std::lock_guard lk(slot->pr.m);
				slot->pr.busy = true;
			}
			if (on) {
				Pre(a, *slot);
			}
			g_jobOrig(a_handle);
			const bool deferred = slot && PrAfterGraph(a, *slot);
			if (slot) {
				slot->pr.busy = false;
			}
			if (on && !deferred) {
				Post(a, *slot, 0.0f);
			}
			if (a) {
				SpiderLegs::Post(a, 0);
			}
		}

		bool InstallJobHook()
		{
			const auto fn = REL::ID{ 41370 }.address();
			const auto site = fn + 0x13E;
			const auto orig = REL::ID{ 41453 }.address();
			const auto* p = reinterpret_cast<const std::uint8_t*>(site);
			if (p[0] != 0x48 || p[1] != 0x8D || p[2] != 0x15 ||
				site + 7 + *reinterpret_cast<const std::int32_t*>(p + 3) != orig) {
				logger::error("Locomotion: NPC animation job site not as expected - NPC pose layer NOT installed");
				return false;
			}
			auto* stub = static_cast<std::uint8_t*>(SKSE::GetTrampoline().allocate(16));
			stub[0] = 0xFF;
			stub[1] = 0x25;
			*reinterpret_cast<std::int32_t*>(stub + 2) = 0;
			*reinterpret_cast<std::uintptr_t*>(stub + 6) = reinterpret_cast<std::uintptr_t>(&JobThunk);
			const auto delta = reinterpret_cast<std::intptr_t>(stub) - static_cast<std::intptr_t>(site + 7);
			if (delta > INT32_MAX || delta < INT32_MIN) {
				logger::error("Locomotion: job stub out of range");
				return false;
			}
			g_jobOrig = reinterpret_cast<JobFn>(orig);
			REL::safe_write(site + 3, static_cast<std::int32_t>(delta));
			logger::info("Locomotion: hooked the NPC animation job (image+{:#x})", orig - REL::Module::get().base());
			return true;
		}

		std::atomic<float> g_playerSpeedPct{ 0.0f };

		template <int N>
		struct UpdateAnim
		{
			static void thunk(RE::Actor* a_this, float a_dt)
			{
				std::shared_ptr<Slot> slot = Find(a_this->GetFormID());
				if (!slot) {
					func(a_this, a_dt);
					SpiderLegs::Post(a_this, 1);
					return;
				}
				const bool on = Active() && (!slot->player || PlayerLayer());
				if (on) {
					Pre(a_this, *slot);
				}
				func(a_this, a_dt);
				if (!PrAfterGraph(a_this, *slot) && on) {
					Post(a_this, *slot, a_dt);
				}
				SpiderLegs::Post(a_this, 1);
			}
			static inline REL::Relocation<decltype(thunk)> func;
		};

		bool Eligible(RE::Actor* a_actor, bool a_grounded)
		{
			if (!a_grounded || a_actor->IsDead() || a_actor->IsInRagdollState() || a_actor->IsInKillMove() || a_actor->IsOnMount()) {
				return false;
			}
			const auto* st = a_actor->AsActorState();
			if (!st || st->IsFlying() || st->IsSwimming() || st->IsBleedingOut() || st->GetSitSleepState() != RE::SIT_SLEEP_STATE::kNormal) {
				return false;
			}
			return !JumpNav::InFlight(a_actor->GetFormID()) && !Mantle::Active(a_actor->GetFormID());
		}

		void ProbeGraph(RE::Actor* a_actor, const char* a_label)
		{
			RE::BSTSmartPointer<RE::BSAnimationGraphManager> mgr;
			if (!a_actor->GetAnimationGraphManager(mgr) || !mgr || mgr->graphs.size() == 0 || !mgr->graphs[0]) {
				return;
			}
			auto& ch = mgr->graphs[0]->characterInstance;
			logger::info("ANIM probe {}: graphs={} footIkDriver={} handIkDriver={}", a_label, mgr->graphs.size(), static_cast<bool>(ch.footIkDriver),
				static_cast<bool>(ch.handIkDriver));
			if (auto* fd = ch.footIkDriver.get()) {
				logger::info("ANIM probe {}: foot IK legs={} disableFootIk={}", a_label, fd->internalLegData.size(), fd->disableFootIk);
			}
			std::string names;
			for (const char* n : { "Speed", "SpeedSampled", "SpeedWalk", "SpeedRun", "Direction", "TurnDelta", "TurnDeltaSpeed", "IsFootIK",
					 "FootIK", "bFootIK", "footIKEnable", "SpeedAcc", "SpeedDamped", "Pitch", "Slope", "Roll" }) {
				float v = 0.0f;
				if (a_actor->GetGraphVariableFloat(RE::BSFixedString(n), v)) {
					names += std::format(" {}={:.2f}", n, v);
				}
			}
			logger::info("ANIM probe {}: graph variables:{}", a_label, names);
		}

		void SampleActor(RE::Actor* a_actor, Slot& s, float now, float a_dt, bool a_isPlayer, int& a_raysLeft)
		{
			CrashLog::Crumb("Sample");
			const auto& c = g_cfg;
			const V3    pos = a_actor->GetPosition();
			auto* cc = a_actor->GetCharController();
			const bool  grounded = cc && cc->context.currentState == RE::hkpCharacterStateType::kOnGround;
			V3          normal{ 0, 0, 1 };
			bool        hasN = false;
			if (cc && grounded && cc->surfaceInfo.supportedState.get() != RE::hkpSurfaceInfo::SupportedState::kUnsupported) {
				const auto* q = reinterpret_cast<const float*>(&cc->surfaceInfo.surfaceNormal);
				normal = Norm({ q[0], q[1], q[2] }, { 0, 0, 1 });
				hasN = normal.z > 0.3f;
			}
			float h = a_actor->GetHeight();
			if (!(h > 60.0f && h < 400.0f)) {
				h = 128.0f;
			}
			const float facing = Facing(a_actor);
			const bool  eligible = Eligible(a_actor, grounded);
			const auto* st0 = a_actor->AsActorState();
			const bool  unarmed = st0 && !st0->IsWeaponDrawn();
			if (a_isPlayer && cc) {
				g_playerSpeedPct.store(cc->speedPct, std::memory_order_relaxed);
			}

			bool  tryClimb = false, tryHang = false;
			Foot  feet[2];
			V3    vdir;
			float speedH;
			float climbReach = 75.0f;
			bool  doCast = false;
			{
				std::lock_guard lk(s.m);
				s.player = a_isPlayer;
				s.height = h;
				const float dt = now - s.sampleT;
				if (s.havePos && dt > 0.004f && dt < 0.25f) {
					if (c.log && grounded && s.prevGrounded) {
							static std::atomic_int snapLogs{ 0 };
							const float dz = pos.z - s.pos.z, pop = dz - s.vel.z * dt, hd = std::hypot(pos.x - s.pos.x, pos.y - s.pos.y);
							if (s.speedH > 100.0f && dz == 0.0f && hd == 0.0f) {
								s.staleFrames += 1.0f;
							}
							if (std::fabs(pop) > 3.0f) {
								if (now - s.popT0 > 1.0f) {
									if (s.popFlips >= 4 && snapLogs.fetch_add(1) < 400) {
										logger::info("ANIMSHAKE {:08X} '{}' events={} flips={} amp={:.1f} staleFrames={:.0f} speedH={:.0f}", s.id, a_actor->GetName(), s.popN,
											s.popFlips, s.popAmp, s.staleFrames, s.speedH);
									}
									s.popT0 = now;
									s.popFlips = s.popN = 0;
									s.popAmp = s.staleFrames = 0.0f;
								}
								++s.popN;
								s.popAmp = (std::max)(s.popAmp, std::fabs(pop));
								if (pop * s.popLast < 0.0f) {
									++s.popFlips;
								}
								s.popLast = pop;
								if (std::fabs(pop) > 8.0f && snapLogs.fetch_add(1) < 400) {
									logger::info("ANIMSNAP {:08X} '{}' pop={:.1f} dz={:.1f} exp={:.1f} dt={:.3f} horiz={:.1f} speedH={:.0f} nz={:.2f} step={:.0f} driven={} elig={}", s.id,
										a_actor->GetName(), pop, dz, s.vel.z * dt, dt, hd, s.speedH, normal.z, s.stepVis, g_frame.load() - s.drivenFrame <= 2, eligible);
								}
							}
						}
						V3 raw = (pos - s.pos) * (1.0f / dt);
					if (raw.Length() > 3000.0f) {
						s.vel = V3{ 0, 0, 0 };
					} else {
						const float k = 1.0f - std::exp(-dt / 0.06f);
						s.vel = s.vel + (raw - s.vel) * k;
					}
					s.speedH = std::hypot(s.vel.x, s.vel.y);
					if (a_isPlayer) {
						s.mmBlockT = s.cmdSpeed > 30.0f && now - s.cmdT < 0.25f && s.speedH < 0.35f * s.cmdSpeed ? s.mmBlockT + dt : 0.0f;
					}
					s.speed3 = s.vel.Length();
					if (s.speedH > 60.0f) {
						s.vdir = { s.vel.x / s.speedH, s.vel.y / s.speedH, 0.0f };
						const float hd = a_isPlayer && PlayerMotionOn() && g_visOwned.load(std::memory_order_relaxed) ? facing : std::atan2(s.vel.x, s.vel.y);
						if (s.haveHeading) {
							const float rate = Clamp(Wrap(hd - s.heading) / dt, -8.0f, 8.0f);
							s.turnRate += (rate - s.turnRate) * (1.0f - std::exp(-dt / 0.10f));
						}
						s.heading = hd;
						s.haveHeading = true;
					} else {
						s.turnRate *= std::exp(-dt / 0.15f);
						s.haveHeading = false;
					}
					s.accel += ((s.speedH - s.prevSpeedH) / dt - s.accel) * (1.0f - std::exp(-dt / 0.15f));
					s.prevSpeedH = s.speedH;
					if (now - s.histPush >= 0.02f) {
						s.histPush = now;
						s.histT[s.histI] = now;
						s.histV[s.histI] = s.speedH;
						s.histI = (s.histI + 1) % 16;
						s.histN = (std::min)(s.histN + 1, 16);
					}
					float peak = 0.0f;
					for (int i = 0; i < s.histN; ++i) {
						if (now - s.histT[i] < 0.25f) {
							peak = (std::max)(peak, s.histV[i]);
						}
					}
					s.dipCool -= dt;
					const float drop = peak - s.speedH;
					if (c.stopSettle && !(a_isPlayer && c.motionMatch && c.playerMotion) && eligible && grounded && s.dipCool <= 0.0f && now - s.airT > 0.5f && peak > 200.0f && drop > c.dipDecel) {
						const float kick = Clamp((drop - 0.5f * c.dipDecel) / c.dipRef, 0.25f, 1.2f) * c.stopScale;
						s.dipKick += kick;
						s.dipCool = 0.5f;
						if (c.log) {
							logger::info("ANIM stop {:08X} drop={:.0f} peak={:.0f} kick={:.2f}", s.id, drop, peak, kick);
						}
					}
				} else if (dt >= 0.25f) {
					s.vel = V3{ 0, 0, 0 };
					s.haveHeading = false;
					s.histN = 0;
					s.accel = 0.0f;
					s.prevSpeedH = 0.0f;
				}
				if (!grounded) {
					if (s.prevGrounded) {
						s.airStart = now;
						s.airTopZ = pos.z;
					}
					s.airTopZ = (std::max)(s.airTopZ, pos.z);
					s.airT = now;
				} else if (!s.prevGrounded && s.havePos && now - s.sampleT < 0.25f) {
					const float fall = s.airTopZ - pos.z;
					if (c.landCrouch && eligible && fall >= c.landMinFall && now - s.airStart > 0.15f) {
						const float kick = Clamp(0.4f + (fall - c.landMinFall) / c.landRef, 0.4f, 1.2f);
						s.dipKick += kick;
						s.dipCool = 0.5f;
						if (c.log) {
							logger::info("ANIM crouch {:08X} fall={:.0f} kick={:.2f}", s.id, fall, kick);
						}
					}
				}
				if (s.havePos && grounded && s.prevGrounded && now - s.sampleT < 0.1f) {
					const float dz = pos.z - s.pos.z;
					if (dz > 5.0f && dz < 80.0f) {
						s.stepVis = (std::min)(s.stepVis + dz, 45.0f);
					}
				}
				s.prevGrounded = grounded;
				s.pos = pos;
				s.havePos = true;
				s.sampleT = now;
				s.sampleFrame = g_frame.load();
				s.eligible = eligible && !s.bonesBad && !(a_isPlayer && PlayerDodging());
				{
					const auto* st = a_actor->AsActorState();
					s.mmOk = s.eligible && !(a_isPlayer && g_fpOff) && st && !st->IsSneaking() && !st->IsSprinting() && st->GetAttackState() == RE::ATTACK_STATE_ENUM::kNone && !a_actor->IsBlocking();
					s.drawn = st && st->IsWeaponDrawn();
					s.combat = a_actor->IsInCombat();
				}
					{
						const bool moving = s.cmdSpeed > 60.0f && now - s.cmdT < 0.2f && g_frame.load() - s.drivenFrame > 2;
						tryClimb = c.climb && eligible && moving && !a_isPlayer && JumpNav::InHurry(a_actor);
						climbReach = s.speedH < 40.0f ? 75.0f : 48.0f;
						const auto* st1 = a_actor->AsActorState();
						tryHang = c.climb && c.hang && !grounded && cc && !a_actor->IsDead() && !a_actor->IsInRagdollState() && !a_actor->IsOnMount() &&
						          st1 && !st1->IsFlying() && !st1->IsSwimming() && st1->GetSitSleepState() == RE::SIT_SLEEP_STATE::kNormal &&
						          !JumpNav::InFlight(a_actor->GetFormID()) && now - s.airStart > 0.05f && now - s.airStart < 1.8f && s.vel.z > -250.0f &&
						          (a_isPlayer || moving);
					}
					s.hasNormal = hasN;
					s.normal = normal;
					const V3 f{ std::sin(facing), std::cos(facing), 0.0f };
				const float up = hasN ? std::atan(-(normal.x * f.x + normal.y * f.y) / normal.z) : 0.0f;
				s.slopeUp = hasN ? s.slopeUp + (up - s.slopeUp) * (1.0f - std::exp(-a_dt / 0.15f)) : s.slopeUp * std::exp(-a_dt / 0.15f);
				doCast = s.eligible && now >= s.nextCast && a_raysLeft >= 5 && (c.duck || c.stepOver);
				if (doCast) {
					s.nextCast = now + 1.0f / c.castHz;
					feet[0] = s.foot[0];
					feet[1] = s.foot[1];
					if (now - s.animT > 0.15f) {
						feet[0].haveAnim = feet[1].haveAnim = false;
					}
				}
				vdir = s.vdir;
				speedH = s.speedH;
			}
				if (tryClimb) {
					Mantle::TryStart(a_actor, false, nullptr, climbReach);
				} else if (tryHang) {
					Mantle::TryStart(a_actor, true, nullptr, 50.0f);
				}
				if (!doCast) {
					return;
				}

				CrashLog::Crumb("Sample: rays");
			Raycast::Caster ray;
			if (!ray.Init(a_actor)) {
				return;
			}
			int       used = 0;
			auto      cast = [&](const V3& from, const V3& to, Raycast::Hit& hit) {
                ++used;
                return ray.Cast(from, to, hit) && !Raycast::IsActorRef(hit.ref);
			};
			float     duckNeed = 0.0f;
			Probe     under[2], ahead[2];
			bool      haveUnder[2]{}, haveAhead[2]{};
			Raycast::Hit hit;
			if (c.duck) {
				const float lead = Clamp(speedH * 0.25f, 0.0f, 60.0f);
				const V3    base = pos + V3{ vdir.x, vdir.y, 0.0f } * lead;
				const float top = pos.z + h;
				if (cast({ base.x, base.y, pos.z + h * 0.55f }, { base.x, base.y, top + c.duckMax }, hit) && hit.point.z > pos.z + h * 0.55f + 2.0f) {
					duckNeed = (std::max)(duckNeed, Clamp(c.duckMargin - (hit.point.z - top), 0.0f, c.duckMax));
				}
			}
			Slot::Touch tOut[6];
			if (c.touchWalls && unarmed) {
				const V3    fv{ std::sin(facing), std::cos(facing), 0.0f }, rv{ std::cos(facing), -std::sin(facing), 0.0f };
				const float sz = pos.z + h * 0.88f;
				auto        dirProbe = [&](int idx, const V3& dir, float len) {
                    const V3 from{ pos.x, pos.y, sz };
                    if (cast(from, from + dir * len, hit) && std::fabs(hit.normal.z) < 0.7f) {
                        tOut[idx] = { true, hit.point, Norm(hit.normal, dir * -1.0f) };
                    }
				};
				dirProbe(0, fv, 62.0f + Clamp(speedH * 0.15f, 0.0f, 40.0f));
				dirProbe(1, rv * -1.0f, 62.0f);
				dirProbe(2, rv, 62.0f);
				dirProbe(4, Norm(fv * 0.77f - rv * 0.64f, fv), 80.0f + Clamp(speedH * 0.15f, 0.0f, 40.0f));
				dirProbe(5, Norm(fv * 0.77f + rv * 0.64f, fv), 80.0f + Clamp(speedH * 0.15f, 0.0f, 40.0f));
				const V3 from{ pos.x, pos.y, pos.z + h };
				if (cast(from, from + V3{ 0, 0, 44.0f }, hit) && hit.normal.z < -0.5f) {
					tOut[3] = { true, hit.point, Norm(hit.normal, { 0, 0, -1 }) };
				}
			}
			if (c.stepOver) {
				for (int i = 0; i < 2; ++i) {
					if (!feet[i].haveAnim) {
						continue;
					}
					const V3 P = feet[i].anim;
					auto     probe = [&](float x, float y, Probe& out) {
                        const float top = P.z + c.stepMax;
                        if (cast({ x, y, top }, { x, y, P.z - 60.0f }, hit) && hit.point.z < top - 2.0f && hit.normal.z > 0.2f) {
                            out = { true, hit.point.z, x, y, now, Norm(hit.normal, { 0, 0, 1 }) };
                            return true;
                        }
                        return false;
					};
					haveUnder[i] = probe(P.x, P.y, under[i]);
					if (c.stepOver && speedH > 40.0f) {
						const float l = Clamp(speedH * c.leadTime, 0.0f, 45.0f);
						haveAhead[i] = probe(P.x + vdir.x * l, P.y + vdir.y * l, ahead[i]);
					}
				}
			}
			a_raysLeft -= used;
			g_n.rays.fetch_add(used, std::memory_order_relaxed);
			{
				std::lock_guard lk(s.m);
					s.duckNeed = duckNeed;
					for (int k = 0; k < 6; ++k) {
						s.touch[k] = tOut[k];
					}
					s.touchT = now;
for (int i = 0; i < 2; ++i) {
					s.foot[i].under = haveUnder[i] ? under[i] : Probe{};
					s.foot[i].ahead = haveAhead[i] ? ahead[i] : Probe{};
				}
			}
		}

		float g_logT = 0.0f;
		int   g_probed = 0;

		void LogStats(float now)
		{
			if (now - g_logT < 10.0f) {
				return;
			}
			const float span = now - g_logT;
			g_logT = now;
			const auto posts = g_n.posts.exchange(0), ns = g_n.postNs.exchange(0), rays = g_n.rays.exchange(0), ovr = g_n.speedOvr.exchange(0);
			g_stats.actors = g_activeActors.load();
			g_stats.postUs = posts ? static_cast<float>(ns) / 1000.0f / static_cast<float>(posts) : 0.0f;
			g_stats.raysPerSec = static_cast<float>(rays) / span;
			g_stats.speedOverrides = static_cast<std::uint32_t>(ovr);
			const auto mmS = g_n.mmSearch.exchange(0), mmW = g_n.mmSwitch.exchange(0), mmNs = g_n.mmNs.exchange(0), mmP = g_n.mmPosts.exchange(0);
			if (g_cfg.log && mmP) {
				logger::info("ANIMMM stats: {:.1f}us/pass x{:.0f}/s searches={:.1f}/s switches={:.1f}/s", static_cast<float>(mmNs) / 1000.0f / static_cast<float>(mmP), static_cast<float>(mmP) / span,
					static_cast<float>(mmS) / span, static_cast<float>(mmW) / span);
			}
			if (g_cfg.log && (posts || rays)) {
				logger::info("ANIMSTAT actors={} post={:.1f}us x{}/s rays={:.0f}/s speedOverrides={} staleWorld={:.2f}u speedPct(player)={:.2f}",
					g_stats.actors, g_stats.postUs, static_cast<float>(posts) / span, g_stats.raysPerSec, ovr,
					g_n.staleMax.exchange(0.0f), g_playerSpeedPct.load());
			}
		}
	}

	namespace
	{
		RE::NiPoint2 g_inRaw{}, g_inSmooth{}, g_inLast{};
		V3           g_inVel{};
		float        g_inAng = 0.0f;
		bool         g_inMoving = false;
		bool         g_inOn = false;
		float        g_visAng = 0.0f, g_visOff = 0.0f, g_visCheckYaw = 0.0f;
		bool         g_visOn = false, g_visDone = false;
		RE::NiPointer<RE::NiAVObject> g_visCheck;
		int          g_visConv = 1;
		bool         g_visChecked = false;
		int          g_visSet = 0, g_visLost = 0, g_visSync = 0;
		struct VisProbe
		{
			int                           arm = 0, lines = 0;
			float                         heading = 0, root = 0, npcRoot = 0, head[4]{}, prevHead[4]{};
			bool                          havePrev = false, pass = false;
			RE::NiPointer<RE::NiAVObject> npcRootNode, headNode;
		} g_vp;

		struct SneakHook
		{
			static void thunk(RE::SneakHandler* a_this, RE::ButtonEvent* a_event, RE::PlayerControlsData* a_data)
			{
				if (g_inOn && PlayerMotion() && a_event && a_event->IsDown() && g_inRaw.x * g_inRaw.x + g_inRaw.y * g_inRaw.y < 0.0025f) {
					g_inVel = V3{};
					g_inSmooth = g_inLast = RE::NiPoint2{};
					a_data->moveInputVec = RE::NiPoint2{};
				}
				func(a_this, a_event, a_data);
			}
			static inline REL::Relocation<void (*)(RE::SneakHandler*, RE::ButtonEvent*, RE::PlayerControlsData*)> func;
		};

		template <int Slot>
		struct InputHook
		{
			template <class Ev>
			static void thunk(RE::MovementHandler* a_this, Ev* a_event, RE::PlayerControlsData* a_data)
			{
				if (g_inOn && !PlayerMotion()) {
					a_data->moveInputVec = g_inRaw;
					g_inOn = false;
				}
				if (g_inOn) {
					a_data->moveInputVec = g_inRaw;
				}
				func(a_this, a_event, a_data);
				if (g_inOn) {
					g_inRaw = a_data->moveInputVec;
					a_data->moveInputVec = g_inSmooth;
					g_inLast = g_inSmooth;
				}
			}
			static inline REL::Relocation<void (*)(RE::MovementHandler*, void*, RE::PlayerControlsData*)> func;
		};

		bool g_ownHeading = false;

		struct FreeRotGet
		{
			static bool thunk(RE::ThirdPersonState* a_this) { return (g_ownHeading && PlayerMotion()) || func(a_this); }
			static inline REL::Relocation<decltype(thunk)> func;
		};

		struct FreeRotSet
		{
			static void thunk(RE::ThirdPersonState* a_this, bool a_on)
			{
				if (g_ownHeading && PlayerMotion()) {
					a_this->freeRotationEnabled = true;
					return;
				}
				func(a_this, a_on);
			}
			static inline REL::Relocation<decltype(thunk)> func;
		};

		void InputStep(float dt)
		{
			auto* pc = RE::PlayerControls::GetSingleton();
			if (!pc) {
				return;
			}
			auto& v = pc->data.moveInputVec;
			g_mmGoalOn = false;
			const bool on = PlayerMotion();
			bool       tdmWant = false;
			if (on) {
				auto*       p0 = RE::PlayerCharacter::GetSingleton();
				const auto* a0 = p0 ? p0->AsActorState() : nullptr;
				tdmWant = p0 && a0 && (Mantle::Active(p0->GetFormID()) || (g_cfg.playerHeading && !TDM::TargetLocked() && !p0->IsOnMount() && !a0->IsSwimming()));
			}
			{
				auto* c0 = RE::PlayerCamera::GetSingleton();
				g_fpOff = !g_cfg.firstPersonMotion && c0 && c0->IsInFirstPerson();
			}
			if (!on || (!(g_cfg.playerAccelOn && g_cfg.playerAccel > 0.0f) && !g_cfg.playerHeading) || g_fpOff) {
				if (g_inOn) {
					v = g_inRaw;
					g_inOn = false;
				}
				g_ownHeading = false;
				g_visOn = false;
				g_visOff = 0.0f;
				g_visYawOff.store(0.0f);
				g_visOwned.store(false);
				TDM::Hold(tdmWant);
				return;
			}
			auto* pc2 = RE::PlayerCharacter::GetSingleton();
			if (!g_inOn) {
				g_inOn = true;
				g_inRaw = g_inSmooth = g_inLast = v;
				g_inVel = V3{};
			} else if (v.x != g_inLast.x || v.y != g_inLast.y) {
				g_inRaw = v;
			}
			float camYaw = pc2 ? pc2->data.angle.z : 0.0f, posYaw = camYaw;
			if (auto* cam = RE::PlayerCamera::GetSingleton(); cam && pc2 && cam->IsInThirdPerson()) {
				const auto* t = static_cast<RE::ThirdPersonState*>(cam->currentState.get());
				if (t->freeRotationEnabled) {
					camYaw = Wrap(camYaw + t->freeRotation.x);
				}
				if (cam->cameraRoot) {
					const V3 d = pc2->GetPosition() - cam->cameraRoot->world.translate;
					posYaw = std::atan2(d.x, d.y);
				}
			}
			const V3    fw{ std::sin(camYaw), std::cos(camYaw), 0.0f }, rt{ std::cos(camYaw), -std::sin(camYaw), 0.0f };
			const V3    goal = rt * g_inRaw.x + fw * g_inRaw.y;
			const float gm = goal.Length();
			float       m = g_inVel.Length();
			{
				const V3 tgt = gm > 0.05f ? goal : V3{};
				if (pc->data.running && g_cfg.playerAccelOn && g_cfg.playerAccel > 0.0f) {
					const float k = 1.0f - std::exp(-0.6931f * dt / (g_cfg.playerAccel * (tgt.Length() < m ? 0.5f : 1.0f)));
					g_inVel = g_inVel + (tgt - g_inVel) * k;
				} else {
					g_inVel = tgt;
				}
				m = g_inVel.Length();
				if (gm <= 0.05f && m < 0.12f) {
					g_inVel = V3{};
					m = 0.0f;
				}
				if (m > 0.05f) {
					g_inAng = std::atan2(g_inVel.x, g_inVel.y);
				}
				g_inMoving = m > 0.05f;
			}
			g_mmGoal = goal;
			g_mmGoalOn = true;
			g_mmMag = m;
			g_inSmooth = { g_inVel.Dot(rt), g_inVel.Dot(fw) };

			auto*       cam = RE::PlayerCamera::GetSingleton();
			auto*       tps = cam && cam->IsInThirdPerson() ? static_cast<RE::ThirdPersonState*>(cam->currentState.get()) : nullptr;
			const auto* ast = pc2 ? pc2->AsActorState() : nullptr;
			const bool  climbing = tps && pc2 && Mantle::Active(pc2->GetFormID());
			const bool  combat = ast && ast->IsWeaponDrawn();
			const bool  hold = climbing || (g_cfg.playerHeading && tps && ast && !TDM::TargetLocked() && !pc2->IsOnMount() && !ast->IsSwimming());
			const bool  own = hold && (climbing || g_cfg.combatFreeCamera || !combat);
			const bool  dodging = own && !climbing && PlayerDodging();
			static float prevYaw = 0.0f;
			const bool  aim = own && !climbing && combat && g_cfg.attackTurn && (ast->GetAttackState() != RE::ATTACK_STATE_ENUM::kNone || pc2->IsBlocking());
			static float logT = 0.0f, lastSet = 0.0f;
			static bool  owned = false;
			logT += dt;
			if (climbing) {
				tps->freeRotationEnabled = true;
				g_inAng = pc2->data.angle.z;
			} else if (dodging) {
				const float now = pc2->data.angle.z;
				tps->freeRotationEnabled = true;
				tps->freeRotation.x = Wrap(tps->freeRotation.x + Wrap(prevYaw - now));
				g_inAng = now;
				g_inSmooth = g_inRaw;
			} else if (aim) {
				const float before = pc2->data.angle.z;
				const float now = Wrap(before + Clamp(Wrap(camYaw - before), -12.0f * dt, 12.0f * dt));
				tps->freeRotationEnabled = true;
				tps->freeRotation.x = Wrap(tps->freeRotation.x + Wrap(before - now));
				pc2->SetHeading(now);
				g_inAng = lastSet = now;
				g_inSmooth = g_inRaw;
			} else if (own && m <= 0.05f) {
				tps->freeRotationEnabled = true;
			} else if (own) {
				const float before = pc2->data.angle.z;
				if (g_cfg.log && logT > 1.0f) {
					logT = 0.0f;
					constexpr float kD = 180.0f / kPi;
					logger::info("PLAYERHEAD engine turned the actor {:.0f} deg since our last set (0 = not fighting), body {:.0f} goal-body {:.0f} cam-actor {:.0f} cam(pos)-cam {:.1f} freeRot {:.0f} enabled {} speed {:.2f}",
						owned ? Wrap(before - lastSet) * kD : 0.0f, g_inAng * kD, Wrap(std::atan2(goal.x, goal.y) - g_inAng) * kD, Wrap(camYaw - before) * kD,
						Wrap(posYaw - camYaw) * kD, tps->freeRotation.x * kD, tps->freeRotationEnabled, m);
				}
				tps->freeRotationEnabled = true;
				tps->freeRotation.x = Wrap(tps->freeRotation.x + Wrap(before - g_inAng));
				if (g_cfg.log && std::fabs(Wrap(g_inAng - before)) > 0.7f && g_vp.lines < 150) {
					g_vp.arm = 6;
				}
				pc2->SetHeading(g_inAng);
				lastSet = g_inAng;
				g_inSmooth = { 0.0f, m };
			}
			owned = own && !climbing && m > 0.05f;
			g_ownHeading = own;
			prevYaw = pc2->data.angle.z;
			if (own && !climbing && !dodging && !aim && g_visOn) {
				const float turn = g_cfg.playerTurnDeg * kPi / 180.0f * dt;
				g_visAng = Wrap(g_visAng + Clamp(Wrap(prevYaw - g_visAng), -turn, turn));
			} else {
				g_visAng = prevYaw;
			}
			g_visOn = true;
			g_visOff = Wrap(g_visAng - prevYaw);
			g_visYawOff.store(g_visConv ? g_visOff : 0.0f);
			g_visOwned.store(own && !climbing && !dodging && !aim);
			TDM::Hold(tdmWant);
			v = g_inSmooth;
			g_inLast = g_inSmooth;
		}

		float RootYaw(const RE::NiMatrix3& R, int a_conv)
		{
			return a_conv == 2 ? std::atan2(R.entry[1][0], R.entry[1][1]) : std::atan2(R.entry[0][1], R.entry[1][1]);
		}

		void VisualYaw()
		{
			g_visDone = false;
			g_visCheck.reset();
			auto* pc = RE::PlayerCharacter::GetSingleton();
			auto* root = pc ? pc->Get3D(false) : nullptr;
			if (!root || g_visChecked) {
				return;
			}
			const auto& R = root->local.rotate;
			const float yaw = pc->data.angle.z;
			if (std::fabs(std::sin(yaw)) < 0.3f) {
				return;
			}
			const float c1 = std::fabs(Wrap(RootYaw(R, 1) - yaw)), c2 = std::fabs(Wrap(RootYaw(R, 2) - yaw));
			if ((std::min)(c1, c2) > 0.05f) {
				return;
			}
			g_visChecked = true;
			g_visConv = c1 <= c2 ? 1 : 2;
			logger::info("ANIMVIS root forward = {} (yaw err col {:.1f} row {:.1f} deg)", g_visConv == 1 ? "column" : "row", c1 * 57.3f, c2 * 57.3f);
		}

		bool RotYaw(RE::NiAVObject* a_node, float a_d, bool a_aboutParent)
		{
			if (!a_node || g_visConv == 0 || std::fabs(a_d) < 0.002f) {
				return false;
			}
			const float   c = std::cos(a_d), s = std::sin(a_d);
			RE::NiMatrix3 M;
			M.entry[0][0] = c, M.entry[0][1] = s, M.entry[0][2] = 0.0f;
			M.entry[1][0] = -s, M.entry[1][1] = c, M.entry[1][2] = 0.0f;
			M.entry[2][0] = 0.0f, M.entry[2][1] = 0.0f, M.entry[2][2] = 1.0f;
			auto& t = a_node->local;
			t.rotate = g_visConv == 1 ? M * t.rotate : t.rotate * M.Transpose();
			if (a_aboutParent) {
				t.translate = M * t.translate;
			}
			return true;
		}

		bool VisRotate(RE::NiAVObject* a_node) { return RotYaw(a_node, g_visOff, true); }

		void SyncRootYaw(RE::NiAVObject* a_root)
		{
			auto* pc = RE::PlayerCharacter::GetSingleton();
			if (a_root && pc && g_visConv != 0 && std::fabs(g_visOff) >= 0.002f) {
				if (RotYaw(a_root, Wrap(pc->data.angle.z - RootYaw(a_root->local.rotate, g_visConv)), false)) {
					++g_visSync;
				}
			}
		}

		void VisNote(RE::NiAVObject* a_node)
		{
			g_visDone = true;
			g_visCheck.reset(a_node);
			g_visCheckYaw = RootYaw(a_node->world.rotate, g_visConv);
			++g_visSet;
		}

		void VisualYawFallback()
		{
			auto* pc = RE::PlayerCharacter::GetSingleton();
			auto* root = pc ? pc->Get3D(false) : nullptr;
			if (g_visDone || !root || g_visConv == 0 || std::fabs(g_visOff) < 0.002f || !PlayerMotion()) {
				return;
			}
			if (!RotYaw(root, Wrap(pc->data.angle.z + g_visOff - RootYaw(root->local.rotate, g_visConv)), false)) {
				return;
			}
			RE::NiUpdateData u{};
			root->Update(u);
			VisNote(root);
		}
	}

	namespace
	{
		struct Spring
		{
			V3  prev{}, prevVel{}, xv{};
			int warm = 2;
			int kind = -1;

			void Reset(int a_kind)
			{
				kind = a_kind;
				warm = 2;
				xv = g_camOff = {};
			}

			void Step(const V3& p, int a_kind, float dt)
			{
				if (a_kind != kind || dt <= 0.0f || dt > 0.25f || (p - prev).Length() > 300.0f) {
					Reset(a_kind);
				}
				const V3 v{ 0.0f, 0.0f, (p.z - prev.z) / dt };
				prev = p;
				if (warm > 0) {
					if (--warm == 0) {
						prevVel = v;
					}
					return;
				}
				g_vst.maxStep = (std::max)(g_vst.maxStep, std::fabs(v.z) * dt);
				xv = xv - (v - prevVel);
				prevVel = v;
				V3          x = g_camOff;
				const float y = 2.0f * 0.6931f / (std::max)(g_cfg.vertHalf, 0.01f) * (1.0f + x.Length() / (std::max)(g_cfg.camStiffDist, 1.0f));
				const V3    j1 = xv + x * y;
				const float e = std::exp(-y * dt);
				xv = (xv - j1 * (y * dt)) * e;
				x = (x + j1 * dt) * e;
				if (const float l = x.Length(); l > g_cfg.vertMax) {
					x = x * (g_cfg.vertMax / l);
					xv = {};
				}
				g_camOff = x;
				g_vst.maxOff = (std::max)(g_vst.maxOff, x.Length());
			}
		} g_spring;

		template <int N>
		struct CamUpdate
		{
			static void thunk(RE::TESCameraState* a_this, RE::BSTSmartPointer<RE::TESCameraState>& a_next)
			{
				func(a_this, a_next);
				auto* cam = RE::PlayerCamera::GetSingleton();
				auto* root = cam && cam->currentState.get() == a_this ? cam->cameraRoot.get() : nullptr;
				if (!root) {
					return;
				}
				auto*       pc = RE::PlayerCharacter::GetSingleton();
				const auto* cam3 = N == 0 ? static_cast<RE::ThirdPersonState*>(a_this)->thirdPersonCameraObj : nullptr;
				g_cam.on = false;
				if (!PlayerMotion() || !g_cfg.vertSpring || !pc || !pc->Get3D(false) || (N == 0 && (!cam3 || SmoothCamLoaded()))) {
					g_spring.Reset(-1);
					return;
				}
				static std::uint32_t tick = 0;
				if (tick != g_camTick) {
					tick = g_camTick;
					g_spring.Step(N == 0 ? V3(cam3->world.translate) : V3(root->local.translate), N, g_camDt);
				}
				{
					static std::uint32_t lastTick = 0;
					static V3            pp{}, pv{};
					static float         held = 0.0f;
					if (lastTick != g_camTick && g_camDt > 0.0f) {
						lastTick = g_camTick;
						const V3 v{ (pc->GetPositionX() - pp.x) / g_camDt, (pc->GetPositionY() - pp.y) / g_camDt, 0.0f };
						const float sp = v.Length();
						held = sp > 150.0f && sp < 1500.0f && (v - pv).Length() / g_camDt < 80.0f ? held + g_camDt : 0.0f;
						pp = pc->GetPosition();
						pv = v;
						if (held > 2.0f) {
							g_vst.steadyOff = (std::max)(g_vst.steadyOff, std::hypot(g_camOff.x, g_camOff.y));
							++g_vst.steadyN;
						}
					}
				}
				const V3 o{ 0.0f, 0.0f, g_camOff.z };
				if (o.Length() > 0.05f) {
					root->local.translate += o;
					RE::NiUpdateData u{};
					root->Update(u);
					++g_vst.sets;
				}
				g_cam.on = true;
				g_cam.state = a_this;
				g_cam.world = root->world.translate;

				static float logT = 0.0f;
				if (g_cfg.log && (logT += g_camDt) > 2.0f) {
					logT = 0.0f;
					logger::info("CAMSPRING last 2 s: pivot {}: biggest height step {:.1f}u/frame, biggest camera lag {:.1f}u, lag after 2 s of constant velocity {:.1f}u over {} frames (TDM directional movement {}), offsets applied {}, rewritten after the state (x/y/z) {}/{}/{}, render camera - root {:.1f}u",
						N == 0 ? "Camera3rd" : "eye", g_vst.maxStep, g_vst.maxOff, g_vst.steadyOff, g_vst.steadyN, TDM::Api() && TDM::Api()->GetDirectionalMovementState(), g_vst.sets, g_vst.lost[0], g_vst.lost[1], g_vst.lost[2], g_vst.wcOff);
					g_vst = {};
				}
			}
			static inline REL::Relocation<decltype(thunk)> func;
		};
	}

	void SetPlayerHeading(float a_heading)
	{
		auto* pc = RE::PlayerCharacter::GetSingleton();
		if (!pc) {
			return;
		}
		auto* cam = RE::PlayerCamera::GetSingleton();
		if (cam && cam->IsInThirdPerson()) {
			auto* tps = static_cast<RE::ThirdPersonState*>(cam->currentState.get());
			tps->freeRotationEnabled = true;
			tps->freeRotation.x = Wrap(tps->freeRotation.x + Wrap(pc->data.angle.z - a_heading));
			g_ownHeading = true;
		}
		pc->SetHeading(a_heading);
	}

	void Install()
	{
		g_mainThread = std::this_thread::get_id();
		g_cfg = Config::Get().anim;
		REL::Relocation<std::uintptr_t> chr{ RE::VTABLE_Character[0] };
		REL::Relocation<std::uintptr_t> plr{ RE::VTABLE_PlayerCharacter[0] };
		UpdateAnim<0>::func = chr.write_vfunc(0x7D, UpdateAnim<0>::thunk);
		UpdateAnim<1>::func = plr.write_vfunc(0x7D, UpdateAnim<1>::thunk);
		REL::Relocation<std::uintptr_t> spd{ RE::VTABLE_BSTAnimationGraphDataChannel_Actor_float_ActorSpeedChannel_[0] };
		SpeedChannel::func = spd.write_vfunc(1, SpeedChannel::thunk);
		REL::Relocation<std::uintptr_t> clip{ RE::VTABLE_hkbClipGenerator[0] };
		ClipUpdate::func = clip.write_vfunc(0x5, ClipUpdate::thunk);
		logger::info("Locomotion: hooked Character/PlayerCharacter::UpdateAnimation and hkbClipGenerator::Update");
		REL::Relocation<std::uintptr_t> mov{ RE::VTABLE_MovementHandler[0] };
		InputHook<2>::func = mov.write_vfunc((REL::Module::IsSE() ? REL::VersionShift(0x02, 2, SKSE::RUNTIME_SSE_1_7_99) : 0x02), InputHook<2>::thunk<RE::ThumbstickEvent>);
		InputHook<4>::func = mov.write_vfunc((REL::Module::IsSE() ? REL::VersionShift(0x04, 2, SKSE::RUNTIME_SSE_1_7_99) : 0x04), InputHook<4>::thunk<RE::ButtonEvent>);
		if (auto* dh = RE::TESDataHandler::GetSingleton(); dh && dh->LookupModByName("DSerCombatGameplayOverhaul.esp")) {
			REL::Relocation<std::uintptr_t> snk{ RE::VTABLE_SneakHandler[0] };
			SneakHook::func = snk.write_vfunc((REL::Module::IsSE() ? REL::VersionShift(0x04, 2, SKSE::RUNTIME_SSE_1_7_99) : 0x04), SneakHook::thunk);
			logger::info("Locomotion: Combat Gameplay Overhaul found, sneak press ends the input ramp (no dodge while stopping)");
		}
		REL::Relocation<std::uintptr_t> tps{ RE::VTABLE_ThirdPersonState[0] };
		FreeRotGet::func = tps.write_vfunc(0x0C, FreeRotGet::thunk);
		FreeRotSet::func = tps.write_vfunc(0x0D, FreeRotSet::thunk);
		CamUpdate<0>::func = tps.write_vfunc(0x03, CamUpdate<0>::thunk);
		REL::Relocation<std::uintptr_t> fps{ RE::VTABLE_FirstPersonState[0] };
		CamUpdate<1>::func = fps.write_vfunc(0x03, CamUpdate<1>::thunk);
		InstallJobHook();
		if (auto* ui = RE::UI::GetSingleton()) {
			ui->AddEventSink<RE::MenuOpenCloseEvent>(&g_menuSink);
		}
	}

	namespace
	{
		void PosePass()
		{
			static int                                frames = 0, playerPasses = 0, npcPasses = 0, skipped = 0;
			static std::vector<std::shared_ptr<Slot>> list;
			g_preRenderFrame.store(g_frame.load());
			list.clear();
			{
				std::lock_guard lk(g_tableMutex);
				for (auto& [id, s] : g_table) {
					if (s->pr.hooked) {
						list.push_back(s);
					}
				}
			}
			std::size_t playerBones = 0;
			for (auto& s : list) {
				auto& pr = s->pr;
				auto* a = s->player ? RE::PlayerCharacter::GetSingleton() : RE::TESForm::LookupByID<RE::Actor>(s->id);
				if (!a || (s->player ? !PlayerLayer() : !g_cfg.npcs)) {
					continue;
				}
				std::lock_guard lk(pr.m);
				if (!pr.hooked || pr.root.get() != a->Get3D(false)) {
					continue;
				}
				if (pr.busy.load()) {
					++skipped;
					continue;
				}
				pr.inPass = true;
				RE::NiAVObject* visNode = nullptr;
				if (s->player) {
					if (PlayerMotion()) {
						SyncRootYaw(pr.root.get());
					}
				}
				for (std::size_t i = 0; i < pr.redir.size(); ++i) {
					auto* real = pr.redir[i].real;
					real->local = pr.pose[i];
					if (s->player && !visNode && std::string_view(real->name.c_str()) == "NPC Root [Root]" && PlayerMotion() && VisRotate(real)) {
						visNode = real;
					}
				}
				Refresh(pr.root.get());
				Post(a, *s, 0.0f);
				RE::NiUpdateData u{};
				pr.root->Update(u);
				if (visNode) {
					VisNote(visNode);
				}
				if (s->player && g_vp.arm > 0 && s->b.head) {
					for (auto& r : pr.redir) {
						if (std::string_view(r.real->name.c_str()) == "NPC Root [Root]") {
							g_vp.npcRootNode.reset(r.real);
						}
					}
					g_vp.headNode.reset(s->b.head);
					g_vp.heading = a->data.angle.z;
					g_vp.root = RootYaw(pr.root->world.rotate, g_visConv);
					g_vp.npcRoot = g_vp.npcRootNode ? RootYaw(g_vp.npcRootNode->world.rotate, g_visConv) : 0.0f;
					ToQuat(s->b.head->world.rotate, g_vp.head);
					g_vp.pass = true;
				}
				pr.inPass = false;
				if (s->player) {
					++playerPasses;
					playerBones = pr.redir.size();
				} else {
					++npcPasses;
				}
			}
			if (++frames >= 300) {
				if (g_cfg.log) {
					logger::info("ANIMPOSE over {} frames: player passes {} ({} bones), NPC passes {} ({} redirected now), skipped (job running) {}, animation job ran during a pass {} times, player edited {} frames",
						frames, playerPasses, playerBones, npcPasses, list.size() - (playerBones ? 1 : 0), skipped, g_passOverlap.exchange(0), g_pose.edited.exchange(0));
				}
				g_pose.job = 0;
				g_pose.main = 0;
				g_pose.again = 0;
				frames = playerPasses = npcPasses = skipped = 0;
			}
		}
	}

	void PreRender()
	{
		auto*      pc = RE::PlayerCharacter::GetSingleton();
		auto*      cam = RE::PlayerCamera::GetSingleton();
		auto*      root = cam ? cam->cameraRoot.get() : nullptr;
		if (g_cam.on && root && cam->currentState.get() == g_cam.state && Active()) {
			const V3 w = root->world.translate;
			V3       lost{};
			for (int i = 0; i < 3; ++i) {
				if (std::fabs(w[i] - g_cam.world[i]) > 0.05f) {
					lost[i] = g_camOff[i];
					++g_vst.lost[i];
				}
			}
			if (lost.Length() > 0.0f) {
				root->local.translate += lost;
				RE::NiUpdateData u{};
				root->Update(u);
			}
			if (auto* wc = RE::Main::WorldRootCamera()) {
				const V3 d = wc->world.translate - root->world.translate;
				g_vst.wcOff = (std::max)(g_vst.wcOff, d.Length());
			}
		}
		g_cam.on = false;

		if (g_vp.arm > 0 && g_vp.pass && g_vp.headNode && pc) {
			auto qAng = [](const float (&a)[4], const float (&b)[4]) {
				const float d = std::fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
				return 2.0f * std::acos((std::min)(d, 1.0f)) * 57.3f;
			};
			float h[4];
			ToQuat(g_vp.headNode->world.rotate, h);
			auto* r3 = pc->Get3D(false);
			constexpr float kD = 57.3f;
			logger::info("ANIMVISFRAME {} after the change: heading {:.0f} | pass: root {:.0f} NPC Root {:.0f} | render: heading {:.0f} root {:.0f} NPC Root {:.0f} | head moved pass->render {:.1f} deg, since last frame {:.1f} deg (visible yaw {:.0f})",
				7 - g_vp.arm, g_vp.heading * kD, g_vp.root * kD, g_vp.npcRoot * kD, pc->data.angle.z * kD, r3 ? RootYaw(r3->world.rotate, g_visConv) * kD : 0.0f,
				g_vp.npcRootNode ? RootYaw(g_vp.npcRootNode->world.rotate, g_visConv) * kD : 0.0f, qAng(g_vp.head, h), g_vp.havePrev ? qAng(g_vp.prevHead, h) : 0.0f,
				g_visAng * kD);
			std::copy(h, h + 4, g_vp.prevHead);
			g_vp.havePrev = true;
			g_vp.pass = false;
			++g_vp.lines;
			if (--g_vp.arm == 0) {
				g_vp.havePrev = false;
			}
		}

		if (g_visCheck) {
			if (std::fabs(Wrap(RootYaw(g_visCheck->world.rotate, g_visConv) - g_visCheckYaw)) > 0.02f) {
				++g_visLost;
			}
			static int n = 0;
			if (++n >= 300) {
				if (g_cfg.log) {
					logger::info("ANIMVIS last {} frames: body turned {} on {} (offset now {:.0f} deg), overwritten before render {}, 3D root behind the heading (synced) {}",
						n, g_visSet, g_visCheck->name.c_str(), g_visOff * 57.3f, g_visLost, g_visSync);
				}
				n = g_visSet = g_visLost = g_visSync = 0;
			}
		}

		{
			static std::vector<std::pair<RE::NiAVObject*, RE::NiTransform>> last, nodes;
			auto*                                                         r3 = pc && Active() && ((g_cfg.vertSpring && PlayerMotion()) || (PlayerLayer() && PoseFeatures(true))) ? pc->Get3D(false) : nullptr;
			nodes.clear();
			if (r3) {
				CollectPrev(r3, nodes);
				if (last.size() == nodes.size()) {
					for (std::size_t i = 0; i < nodes.size(); ++i) {
						if (last[i].first == nodes[i].first) {
							nodes[i].first->previousWorld = last[i].second;
						}
					}
				}
				for (auto& [n, t] : nodes) {
					t = n->world;
				}
			}
			last.swap(nodes);
		}
	}

	void PostRender() {}

	void Reload()
	{
		g_cfg = Config::Get().anim;
	}

	Config::Anim& Settings() { return g_cfg; }

	void SetClimb(RE::FormID a_id, const ClimbPose* a_pose)
	{
		if (auto slot = Find(a_id)) {
			std::lock_guard lk(slot->m);
			if (a_pose) {
				slot->climbPose = *a_pose;
				slot->climbFrame = g_frame.load();
			} else {
				slot->climbPose = ClimbPose{};
			}
		}
	}

	bool IsDriven(RE::FormID a_id)
	{
		if (auto slot = Find(a_id)) {
			std::lock_guard lk(slot->m);
			return g_frame.load() - slot->drivenFrame <= 2;
		}
		return false;
	}

	void NoteDriven(RE::FormID a_id, float a_speed)
	{
		if (auto slot = Find(a_id)) {
			std::lock_guard lk(slot->m);
			slot->driven = a_speed;
			slot->drivenFrame = g_frame.load();
		}
	}

	void Reset()
	{
		{
			std::lock_guard lk(g_tableMutex);
			g_table.clear();
		}
		for (auto& e : g_chars) {
			e.ch.store(nullptr);
		}
		g_probed = 0;
		Mantle::Reset();
	}

	Stats GetStats() { return g_stats; }

	void OnFrame(float a_delta)
	{
		if (a_delta > 0.0f && a_delta <= 0.5f) {
			Mantle::OnFrame(a_delta);
			InputStep(a_delta);
			VisualYaw();
		}
		g_camDt = a_delta;
		++g_camTick;
		if (!Active() || a_delta <= 0.0f || a_delta > 0.5f) {
			return;
		}
		const float now = g_time.load() + a_delta;
		g_time.store(now);
		const auto frame = g_frame.fetch_add(1) + 1;

		auto* player = RE::PlayerCharacter::GetSingleton();
		auto* lists = RE::ProcessLists::GetSingleton();
		if (!player || !lists || !player->Get3D(false)) {
			return;
		}

		struct Cand
		{
			float           d2;
			RE::ActorHandle h;
		};
		static std::vector<Cand> cands;
		cands.clear();
		if (g_cfg.npcs) {
			const V3 pp = player->GetPosition();
			const float r2 = g_cfg.radius * g_cfg.radius;
			for (auto& h : lists->highActorHandles) {
				auto  ptr = h.get();
				auto* a = ptr.get();
				if (!a || a == player || a->IsDead() || !a->Is3DLoaded()) {
					continue;
				}
				const V3 d = a->GetPosition() - pp;
				const float d2 = d.x * d.x + d.y * d.y + d.z * d.z;
				if (d2 < r2) {
					cands.push_back({ d2, h });
				}
			}
			std::sort(cands.begin(), cands.end(), [](const Cand& x, const Cand& y) { return x.d2 < y.d2; });
			if (cands.size() > static_cast<std::size_t>(g_cfg.maxActors)) {
				cands.resize(g_cfg.maxActors);
			}
		}

		int          rays = g_cfg.maxRays;
		std::uint32_t active = 0;
		auto         run = [&](RE::Actor* a, bool a_player) {
            std::shared_ptr<Slot> slot = Find(a->GetFormID());
            if (!slot) {
                slot = std::make_shared<Slot>();
                slot->id = a->GetFormID();
                std::lock_guard lk(g_tableMutex);
                g_table[slot->id] = slot;
                if (g_cfg.log && g_probed < 3) {
                    ++g_probed;
                    ProbeGraph(a, a_player ? "player" : "npc");
                }
            }
            SampleActor(a, *slot, now, a_delta, a_player, rays);
            std::lock_guard lk(slot->m);
            if (slot->gate > 0.5f) {
                ++active;
            }
		};
		if (PlayerLayer()) {
			run(player, true);
		}
		for (auto& cd : cands) {
			auto  ptr = cd.h.get();
			auto* a = ptr.get();
			if (a) {
				run(a, false);
			}
		}
		g_activeActors.store(active);
		if (g_cfg.player || g_cfg.npcs) {
			PosePass();
		}
		VisualYawFallback();

		if ((frame & 63) == 0) {
			std::lock_guard lk(g_tableMutex);
			for (auto it = g_table.begin(); it != g_table.end();) {
				if (frame - it->second->sampleFrame > 180 && !it->second->pr.busy.load()) {
					it = g_table.erase(it);
				} else {
					++it;
				}
			}
			for (auto& e : g_chars) {
				if (e.ch.load() && frame - e.frame.load() > 180) {
					e.ch.store(nullptr);
				}
			}
		}
		LogStats(now);
	}
}
