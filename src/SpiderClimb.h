#pragma once

namespace SpiderClimb
{
	struct Pose
	{
		RE::NiPoint3 normal{ 0, 0, 1 };
		RE::NiPoint3 fwd{ 0, 1, 0 };
		RE::NiPoint3 anchor{};
		float        offset = 0;
	};

	void OnFrame(float a_delta);
	void Reset();
	bool Enabled();
	bool Active(RE::FormID a_id);
	bool Get(RE::FormID a_id, Pose& a_out);

	bool GoTo(RE::Actor* a_actor, const RE::NiPoint3& a_goal, bool a_hold);
	bool Pursue(RE::Actor* a_actor, RE::Actor* a_target);
	void Stop(RE::FormID a_id);
}
