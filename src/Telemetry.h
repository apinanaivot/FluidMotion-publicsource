#pragma once

namespace Telemetry
{
	void Start();
	void OnLoad();
	void EndAllEpisodes();

	struct LiveActor
	{
		RE::FormID  id;
		std::string name;
		std::string package;
		float       speed;
		float       playerDist;
		bool        inEpisode;
		float       episodeTime;
		int         stalls;
		float       fullSpeedShare;
	};

	struct EpisodeSummary
	{
		std::string label;
		std::string name;
		std::string reason;
		float       duration;
		float       slowShare;
		float       fullSpeedShare;
		float       efficiency;
		float       avgSpeed;
		float       p50Speed;
		float       p90Speed;
		int         stalls;
		int         packageSwitches;
		std::string end;
	};

	void SetTraceEnabled(bool a_enabled);
	bool IsTraceEnabled();

	void GetSnapshot(std::vector<LiveActor>& a_live, std::vector<EpisodeSummary>& a_recent);
}
