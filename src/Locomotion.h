#pragma once

#include "Config.h"

namespace Locomotion
{
	struct Stats
	{
		std::uint32_t actors = 0;
		float         postUs = 0;
		float         raysPerSec = 0;
		std::uint32_t speedOverrides = 0;
	};

	void  Install();
	void  Reload();
	void  OnFrame(float a_delta);
	void  Reset();
	struct ClimbPose
	{
		int          clip = 0;
		int          rig = 0;
		float        w = 0;
		float        t = 0;
		float        contact = 0;
		RE::NiPoint3 hand[2];
	};
	void  SetClimb(RE::FormID a_id, const ClimbPose* a_pose);
	bool  IsDriven(RE::FormID a_id);
	void  NoteDriven(RE::FormID a_id, float a_speed);
	void  SetPlayerHeading(float a_heading);
	void  PreRender();
	void  PostRender();
	Stats GetStats();
	Config::Anim& Settings();
}
