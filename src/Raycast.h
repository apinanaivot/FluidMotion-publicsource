#pragma once

namespace Raycast
{
	struct Hit
	{
		RE::NiPoint3       point;
		RE::NiPoint3       normal{ 0, 0, 1 };
		RE::TESObjectREFR* ref = nullptr;
		RE::COL_LAYER      layer = RE::COL_LAYER::kUnidentified;
	};

	inline bool SolidLayer(RE::COL_LAYER a_l)
	{
		using L = RE::COL_LAYER;
		switch (a_l) {
		case L::kTrigger:
		case L::kNonCollidable:
		case L::kCloudTrap:
		case L::kPortal:
		case L::kAcousticSpace:
		case L::kActorZone:
		case L::kProjectileZone:
		case L::kGasTrap:
		case L::kInvisibleWall:
		case L::kWard:
		case L::kAvoidBox:
		case L::kCollisionBox:
		case L::kCameraSphere:
		case L::kDoorDetection:
		case L::kWater:
		case L::kTrap:
			return false;
		default:
			return true;
		}
	}

	inline int g_skipped[64]{};

	struct Caster
	{
		RE::bhkWorld* world = nullptr;
		std::uint32_t filter = 0;
		mutable int   used = 0;

		bool Init(RE::Actor* a_actor)
		{
			auto* cell = a_actor->GetParentCell();
			world = cell ? cell->GetbhkWorld() : nullptr;
			if (!world) {
				return false;
			}
			RE::CFilter f{};
			a_actor->GetCollisionFilterInfo(f);
			filter = (f.filter & 0xFFFF0000) | static_cast<std::uint32_t>(RE::COL_LAYER::kCharController);
			return true;
		}

		bool Cast(const RE::NiPoint3& a_from, const RE::NiPoint3& a_to, Hit& a_out) const
		{
			++used;
			const float     s = RE::bhkWorld::GetWorldScale();
			RE::bhkPickData pick;
			pick.rayInput.from = RE::hkVector4(a_from.x * s, a_from.y * s, a_from.z * s, 0.0f);
			pick.rayInput.to = RE::hkVector4(a_to.x * s, a_to.y * s, a_to.z * s, 0.0f);
			pick.rayInput.filterInfo.filter = filter;
			world->PickObject(pick);
			if (!pick.rayOutput.HasHit()) {
				return false;
			}
			a_out.point = a_from + (a_to - a_from) * pick.rayOutput.hitFraction;
			const auto* n = reinterpret_cast<const float*>(&pick.rayOutput.normal);
			a_out.normal = { n[0], n[1], n[2] };
			a_out.ref = pick.rayOutput.rootCollidable ?
				RE::TESHavokUtilities::FindCollidableRef(*pick.rayOutput.rootCollidable) : nullptr;
			a_out.layer = pick.rayOutput.rootCollidable ? pick.rayOutput.rootCollidable->GetCollisionLayer() : RE::COL_LAYER::kUnidentified;
			return true;
		}

		bool CastSolid(const RE::NiPoint3& a_from, const RE::NiPoint3& a_to, Hit& a_out) const
		{
			RE::NiPoint3 from = a_from;
			for (int i = 0; i < 4; ++i) {
				if (!Cast(from, a_to, a_out)) {
					return false;
				}
				if (SolidLayer(a_out.layer) || IsActorRefStatic(a_out.ref)) {
					return true;
				}
				++g_skipped[static_cast<int>(a_out.layer) & 63];
				const RE::NiPoint3 d = a_to - from;
				const float        len = d.Length();
				if (len < 3.0f) {
					return false;
				}
				from = a_out.point + d * (1.5f / len);
			}
			return false;
		}

		static bool IsActorRefStatic(RE::TESObjectREFR* a_ref) { return a_ref && a_ref->Is(RE::FormType::ActorCharacter); }
	};

	inline bool IsActorRef(RE::TESObjectREFR* a_ref) { return a_ref && a_ref->Is(RE::FormType::ActorCharacter); }
}
