#pragma once

namespace CornerLimit
{
	struct Sample
	{
		float angVel;
		float angAcc;
		float dec;
		float omega;
		float vanilla;
		float used;
	};

	void Install();
	bool IsInstalled();

	inline std::atomic_bool   enabled{ false };
	inline std::atomic<float> omegaScale{ 2.0f };
}
