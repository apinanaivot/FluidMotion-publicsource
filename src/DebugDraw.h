#pragma once

namespace DebugDraw
{
	inline std::atomic_bool  showNavmesh{ false };
	inline std::atomic_bool  showPaths{ false };
	inline std::atomic<float> radius{ 2500.0f };

	void Update();
	bool Active();
	void __stdcall Draw();
}
