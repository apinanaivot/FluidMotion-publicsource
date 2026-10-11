#pragma once

namespace EndBrake
{
	enum Decision : std::int8_t
	{
		kNone = 0,
		kOff = 1,
		kNoFollower = 2,
		kUntouched = 3,
		kEndDecel = 4,
		kFault = 5,
	};

	void Install();
	bool IsInstalled();

	inline std::atomic_bool   enabled{ false };
	inline std::atomic<float> endDecel{ 2000.0f };
}
