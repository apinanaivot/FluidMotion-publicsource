#pragma once

namespace TurnFix
{
	void Install();
	bool IsInstalled();
	void Apply(bool a_fluid);

	inline std::atomic<float> turnAngle{ 120.0f };
}
