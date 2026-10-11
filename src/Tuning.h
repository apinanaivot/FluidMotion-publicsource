#pragma once

namespace Tuning
{
	enum class Mode
	{
		kVanilla,
		kFluid
	};

	void Init();
	void Reload();

	Mode        GetMode();
	void        SetMode(Mode a_mode);
	bool        GetPathfinding();
	void        SetPathfinding(bool a_on);
	void        Refresh();
	const char* ModeLabel();
}
