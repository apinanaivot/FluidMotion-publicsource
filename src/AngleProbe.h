#pragma once

namespace AngleProbe
{
	struct Sample
	{
		float desired, yaw, w0, f6c, wmax, alpha, cmd, f50;
	};
	void Install();
}
