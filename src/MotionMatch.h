#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace MotionMatch
{
	constexpr int kFeat = 27;

	struct Db
	{
		std::uint32_t             nb = 0, nf = 0;
		float                     dt = 1.0f / 30.0f, ratio = 0.74f, ankleH = 5.8f;
		std::vector<std::string>  bones;
		float                     mean[kFeat]{}, scale[kFeat]{};
		std::vector<float>        feat;
		std::vector<std::uint8_t> flags;
		std::vector<float>        comZ, speed;
		std::vector<float>        yawRate;
		std::vector<std::int16_t> q;
	};

	const Db* Get();

	struct Input
	{
		float        vx = 0, vy = 0;
		float        omega = 0;
		bool         haveGoal = false;
		float        gx = 0, gy = 0, goalT = 0.3f;
		float        face[3]{};
		bool         force = false;
		bool         preferVanilla = false;
		bool         haveFeet = false;
		float        foot[2][3]{};
		const float* graphQ = nullptr;
		float        graphZ = 0;
	};

	struct Params
	{
		float searchHz = 8.0f, halflife = 0.12f, margin = 0.12f, rateMin = 0.85f, rateMax = 1.2f, vanillaBias = 0.5f, cadence = 1.0f;
	};

	struct State
	{
		bool               playing = false, vanilla = false;
		float              f = 0;
		float              yawRate = 0;
		float              searchT = 0, offZ = 0, outZ = 0, rate = 1.0f, sinceSwitch = 9.0f, stride = 1.0f;
		std::vector<float> off, out;
		int                switches = 0, searches = 0, frame = -1;
		float              lastCost = 0, curCost = 0;
	};

	void Step(const Db& a_db, State& a_s, const Input& a_in, const Params& a_p, float a_dt);
}
