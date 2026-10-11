#include "SwingFix.h"

#include "Config.h"
#include "JumpNav.h"
#include "Mantle.h"
#include "SpiderClimb.h"
#include "Tuning.h"

namespace SwingFix
{
	namespace
	{
		struct Memo
		{
			double inRangeSince = -1.0, lastSwing = -9.0, nextCheck = 0.0, nextDiag = 0.0, lastT = -1.0;
			float  px = 0, py = 0;
			int    wstate = -1;
			RE::ATTACK_STATE_ENUM prevAtk = RE::ATTACK_STATE_ENUM::kNone;
			int    swings = 0;
		};

		std::unordered_map<RE::FormID, Memo> g_memo;
		double                               g_time = 0.0, g_statT = 0.0;
		int                                  g_swings = 0, g_logs = 0, g_diag = 0;

		bool Melee(RE::Actor* a_actor)
		{
			auto* r = a_actor->GetEquippedObject(false);
			auto* l = a_actor->GetEquippedObject(true);
			if (l && l->Is(RE::FormType::Spell)) {
				return !r && !a_actor->HasKeywordString("ActorTypeNPC");
			}
			if (!r) {
				return true;
			}
			auto* w = r->As<RE::TESObjectWEAP>();
			return w && w->IsMelee();
		}

		std::unordered_map<RE::FormID, std::string>        g_raceEvent;
		std::unordered_map<RE::FormID, std::vector<float>> g_ratios;
		int                                                g_diag2 = 0, g_wlogs = 0;

		float Learned(RE::FormID a_race, float a_default)
		{
			auto it = g_ratios.find(a_race);
			if (it == g_ratios.end() || it->second.size() < 2) {
				return a_default;
			}
			auto v = it->second;
			std::sort(v.begin(), v.end());
			return v[v.size() / 2] * 1.1f;
		}

		const char* AttackEvent(RE::Actor* a_actor)
		{
			auto* race = a_actor->GetRace();
			if (!race || a_actor->GetEquippedObject(false)) {
				return "attackStart";
			}
			auto [it, fresh] = g_raceEvent.try_emplace(race->GetFormID(), "attackStart");
			if (fresh) {
				auto* map = race->attackDataMap.get();
				if (map && map->attackDataMap.empty() && map->defaultDataRace && map->defaultDataRace->attackDataMap) {
					map = map->defaultDataRace->attackDataMap.get();
				}
				std::string all, best;
				if (map) {
					for (auto& [name, data] : map->attackDataMap) {
						if (!data) {
							continue;
						}
						const auto f = data->data.flags.underlying();
						all += std::format("{}[{:x}] ", name.c_str(), f);
						constexpr std::uint32_t skip = 2 | 4 | 8 | 16 | 32;
						if (!(f & skip) && !data->IsLeftAttack() && (best.empty() || std::string_view(name.c_str()) == "attackStart")) {
							best = name.c_str();
						}
					}
				}
				if (!best.empty()) {
					it->second = best;
				}
				logger::info("SWINGEVENT race {:08X} '{}' chosen={} events: {}", race->GetFormID(), race->GetName(), it->second, all);
			}
			return it->second.c_str();
		}

		bool Unarmed(RE::Actor* a_actor) { return !a_actor->GetEquippedObject(false); }
	}

	void Reset() { g_memo.clear(); }

	void OnFrame(float a_dt)
	{
		const auto& c = Config::Get().combat;
		auto*       lists = RE::ProcessLists::GetSingleton();
		auto*       player = RE::PlayerCharacter::GetSingleton();
		if (!c.swing || Tuning::GetMode() != Tuning::Mode::kFluid || !lists || !player || a_dt <= 0.0f || a_dt > 0.5f) {
			return;
		}
		g_time += a_dt;
		for (auto& h : lists->highActorHandles) {
			auto  ptr = h.get();
			auto* a = ptr.get();
			if (!a || a == player || !a->IsInCombat() || a->IsDead() || !a->Is3DLoaded()) {
				continue;
			}
			auto& m = g_memo[a->GetFormID()];
			if (g_time < m.nextCheck) {
				continue;
			}
			const float moved = std::hypot(a->GetPositionX() - m.px, a->GetPositionY() - m.py);
			const float still = m.lastT > 0.0 ? moved / static_cast<float>((std::max)(g_time - m.lastT, 0.01)) : 1e9f;
			m.px = a->GetPositionX(), m.py = a->GetPositionY(), m.lastT = g_time;
			m.nextCheck = g_time + 0.1;
			auto*       tgt = a->GetActorRuntimeData().currentCombatTarget.get().get();
			const auto* st = a->AsActorState();
			if (c.log && g_wlogs < 300 && st && a->GetEquippedObject(false) && a->GetEquippedObject(false)->As<RE::TESObjectWEAP>()) {
				auto*      root = a->Get3D();
				auto*      wn = root ? root->GetObjectByName(RE::BSFixedString("WEAPON")) : nullptr;
				auto*      node = wn ? wn->AsNode() : nullptr;
				const int  kids = node ? static_cast<int>(node->GetChildren().size()) : -1;
				int        hid = wn && wn->GetFlags().any(RE::NiAVObject::Flag::kHidden) ? 1 : 0;
				if (node && kids > 0 && node->GetChildren()[0] && node->GetChildren()[0]->GetFlags().any(RE::NiAVObject::Flag::kHidden)) {
					hid |= 2;
				}
				const int ws = (st->IsWeaponDrawn() ? 1 : 0) | (hid << 1) | ((kids + 1) << 4);
				if (ws != m.wstate) {
					if (m.wstate != -1) {
						++g_wlogs;
						RE::NiPoint3 vel;
						a->GetLinearVelocity(vel);
						logger::info("WEAPONVIS {:08X} '{}' drawn={} hidden={} kids={} speed={:.0f} jump={} mantle={} atkState={} weapon={:08X}", a->GetFormID(), a->GetName(), st->IsWeaponDrawn(), hid, kids,
							std::hypot(vel.x, vel.y), JumpNav::TraceState(a->GetFormID()), Mantle::Active(a->GetFormID()), (int)st->GetAttackState(), a->GetEquippedObject(false)->GetFormID());
					}
					m.wstate = ws;
				}
			}
			const auto  atkNow = st ? st->GetAttackState() : RE::ATTACK_STATE_ENUM::kNone;
			if (tgt && Unarmed(a) && a->GetRace() && atkNow != RE::ATTACK_STATE_ENUM::kNone && m.prevAtk == RE::ATTACK_STATE_ENUM::kNone && g_time - m.lastSwing > 1.5) {
				const auto  to = tgt->GetPosition() - a->GetPosition();
				const float r = a->GetReach();
				if (r > 1.0f) {
					auto& v = g_ratios[a->GetRace()->GetFormID()];
					v.push_back((std::max)(std::hypot(to.x, to.y) - tgt->GetBoundRadius(), 0.0f) / r);
					if (v.size() > 16) {
						v.erase(v.begin());
					}
					if (c.log && g_diag2++ < 100) {
						logger::info("SWINGRANGE '{}' race '{}' natural attack start dh={:.0f} reach={:.0f} ratio={:.2f} n={} learned={:.2f}", a->GetName(), a->GetRace()->GetName(),
							std::hypot(to.x, to.y), r, v.back(), v.size(), Learned(a->GetRace()->GetFormID(), 0.0f));
					}
				}
			}
			m.prevAtk = atkNow;
			bool        ok = tgt && st && !tgt->IsDead() && (st->IsWeaponDrawn() || Unarmed(a)) && st->GetAttackState() == RE::ATTACK_STATE_ENUM::kNone &&
			          st->GetKnockState() == RE::KNOCK_STATE_ENUM::kNormal && st->GetSitSleepState() == RE::SIT_SLEEP_STATE::kNormal && !st->IsFlying() &&
			          !st->IsSwimming() && !a->IsBlocking() && !a->IsAttacking() && !a->IsInRagdollState() && !a->IsOnMount() && !a->IsInKillMove() &&
			          !JumpNav::InFlight(a->GetFormID()) && !Mantle::Active(a->GetFormID()) && Melee(a);
			float dh = 0, dz = 0, ang = 0;
			if (ok) {
				const auto  to = tgt->GetPosition() - a->GetPosition();
				dh = std::hypot(to.x, to.y);
				dz = to.z;
				const float yaw = std::atan2(to.x, to.y), d = yaw - a->GetHeading(false);
				ang = std::fabs(std::remainder(d, 2.0f * 3.14159265f)) * 57.29578f;
				const bool  creature = Unarmed(a);
				const float reach = a->GetReach() * (creature ? Learned(a->GetRace() ? a->GetRace()->GetFormID() : 0, c.swingReachUnarmed) : c.swingReach) + tgt->GetBoundRadius();
				if (SpiderClimb::Active(a->GetFormID())) {
					ok = to.Length() <= reach + 20.0f && still < 30.0f;
				} else {
					ok = dh <= reach && (!creature || still < 30.0f) && (creature ? dz >= -a->GetHeight() && dz <= c.swingRise * a->GetHeight() : (dz >= -0.5f * a->GetHeight() && dz <= c.swingRise * a->GetHeight())) && ang <= c.swingAngleDeg;
				}
			}
			if (!ok) {
				m.inRangeSince = -1.0;
				if (c.log && tgt && g_time >= m.nextDiag && g_diag < 200) {
					const auto  to = tgt->GetPosition() - a->GetPosition();
					const float dh2 = std::hypot(to.x, to.y);
					if (dh2 < 250.0f) {
						m.nextDiag = g_time + 2.0;
						++g_diag;
						logger::info("SWINGWAIT {:08X} '{}' dh={:.0f} reach={:.0f} rad={:.0f} h={:.0f} dz={:.0f} drawn={} atk={} knock={} sit={} fly={} blk={} atking={} rag={} jump={} mantle={} melee={} r={} l={}",
							a->GetFormID(), a->GetName(), dh2, a->GetReach(), tgt->GetBoundRadius(), a->GetHeight(), to.z, st->IsWeaponDrawn(), (int)st->GetAttackState(), (int)st->GetKnockState(),
							(int)st->GetSitSleepState(), st->IsFlying(), a->IsBlocking(), a->IsAttacking(), a->IsInRagdollState(), JumpNav::InFlight(a->GetFormID()), Mantle::Active(a->GetFormID()),
							Melee(a), (void*)a->GetEquippedObject(false), (void*)a->GetEquippedObject(true));
					}
				}
				continue;
			}
			if (m.inRangeSince < 0.0) {
				m.inRangeSince = g_time;
			}
			if (g_time - m.inRangeSince >= c.swingIdleSec && g_time - m.lastSwing >= c.swingCooldown) {
				bool ready = false;
				const bool hadReady = a->GetGraphVariableBool("IsAttackReady", ready) && ready;
				if (!hadReady) {
					a->SetGraphVariableBool("IsAttackReady", true);
				}
				const bool sent = a->NotifyAnimationGraph(AttackEvent(a));
				m.lastSwing = g_time;
				++m.swings;
				++g_swings;
				if (c.log && g_logs++ < 150) {
					logger::info("[{}] SWING {:08X} '{}' dh={:.0f} dz={:.0f} ang={:.0f} reach={:.0f} waited={:.2f}s swings={} event={} ready={} accepted={}", Tuning::ModeLabel(), a->GetFormID(), a->GetName(),
						dh, dz, ang, a->GetReach(), g_time - m.inRangeSince, m.swings, AttackEvent(a), hadReady, sent);
				}
			}
		}
		if (c.log && g_time - g_statT >= 10.0) {
			g_statT = g_time;
			if (g_swings) {
				logger::info("[{}] SWINGSTAT swings={} (10 s)", Tuning::ModeLabel(), g_swings);
			}
			g_swings = 0;
		}
	}
}
