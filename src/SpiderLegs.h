#pragma once

namespace SpiderLegs
{
	void OnFrame(float a_delta);
	void Reset();
	bool Has(RE::FormID a_id);
	void Post(RE::Actor* a_actor, int a_src = 0);
	void OnPresent();
	void Install();
	void ApplyMovement(bool a_fluid);
}
