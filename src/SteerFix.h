#pragma once

namespace SteerFix
{
	void Install();
	bool IsInstalled();
	void Apply(bool a_fluid);

	inline std::atomic<float> lookTime{ 0.5f };
}
