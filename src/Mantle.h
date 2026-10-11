#pragma once

#include "Raycast.h"

namespace Mantle
{
	struct Plan
	{
		RE::NiPoint3 edge, stand, fwd;
		float        topZ = 0, H = 0;
		bool         hang = false;
	};

	bool PlanClimb(const Raycast::Caster& a_ray, const RE::NiPoint3& a_pos, const RE::NiPoint3& a_dir, float a_height, float a_maxWall,
		bool a_air, Plan& a_out, float a_window = 30.0f);
	bool Start(RE::Actor* a_actor, const Plan& a_plan);
	bool TryStart(RE::Actor* a_actor, bool a_air = false, const RE::NiPoint3* a_dir = nullptr, float a_maxWall = 75.0f, float a_window = 30.0f);
	bool CanClimb(RE::Actor* a_actor);
	void OnFrame(float a_delta);
	bool Active(RE::FormID a_id);
	void Reset();
}
