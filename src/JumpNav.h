#pragma once

namespace JumpNav
{
	enum class Trigger : std::uint8_t
	{
		kNone = 0,
		kShortcut = 1,
		kPursuit = 2,
		kFollow = 3,
		kRoute = 4,
	};

	struct Stats
	{
		std::uint32_t evals = 0;
		std::uint32_t launched[5]{};
		std::uint32_t routes = 0, routesFailed = 0, routesDone = 0;
		std::uint32_t landed = 0;
		std::uint32_t aborted = 0;
		std::uint32_t rejRange = 0, rejProfile = 0, rejRise = 0, rejSweep = 0, rejLanding = 0, rejOccupied = 0;
	};

	void  OnFrame(float a_delta);
	void  Reset();
	bool  OnNavmesh(RE::Actor* a_actor, RE::Actor* a_target);
	bool  Routing(RE::FormID a_id);
	bool  InFlight(RE::FormID a_id);
	int   TraceState(RE::FormID a_id);
	bool  InHurry(RE::Actor* a_actor);
	Stats GetStats();

	struct DebugArc
	{
		std::vector<RE::NiPoint3> points;
		float                     age = 0;
		bool                      launched = false;
		std::string               label;
	};
	void GetDebugArcs(std::vector<DebugArc>& a_out);

	inline std::atomic_bool enabled{ false };
}
