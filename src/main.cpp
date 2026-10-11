#include "AngleProbe.h"
#include "Config.h"
#include "CrashLog.h"
#include "DebugDraw.h"
#include "FrameHook.h"
#include "CornerLimit.h"
#include "EndBrake.h"
#include "GameSettings.h"
#include "JumpNav.h"
#include "Locomotion.h"
#include "SpiderClimb.h"
#include "SpiderLegs.h"
#include "Menu.h"
#include "PathProbe.h"
#include "PathRequestProbe.h"
#include "SteerFix.h"
#include "SwingFix.h"
#include "Telemetry.h"
#include "Tuning.h"
#include "TurnFix.h"

namespace
{
	void OnMessage(SKSE::MessagingInterface::Message* a_msg)
	{
		switch (a_msg->type) {
		case SKSE::MessagingInterface::kDataLoaded:
			Config::Load();
			DebugDraw::showNavmesh = Config::Get().debug.showNavmesh;
			DebugDraw::showPaths = Config::Get().debug.showPaths;
			DebugDraw::radius = Config::Get().debug.radius;
			GameSettings::DumpMatching();
			SteerFix::Install();
			CornerLimit::Install();
			TurnFix::Install();
			EndBrake::Install();
			Locomotion::Install();
			SpiderLegs::Install();
			Locomotion::Reload();
			FrameHook::Install();
			AngleProbe::Install();
			PathRequestProbe::Install();
			Tuning::Init();
			Telemetry::Start();
			Menu::Register();
			PathProbe::Install();
			break;
		case SKSE::MessagingInterface::kPostLoadGame:
		case SKSE::MessagingInterface::kNewGame:
			Config::Load();
			Tuning::Reload();
			Locomotion::Reset();
			SpiderLegs::Reset();
			SpiderClimb::Reset();
			Locomotion::Reload();
			Telemetry::OnLoad();
			JumpNav::Reset();
			SwingFix::Reset();
			break;
		default:
			break;
		}
	}
}

SKSEPluginLoad(const SKSE::LoadInterface* a_skse)
{
	if (auto dir = logger::log_directory()) {
		std::error_code ec;
		const auto      cur = *dir / "FluidMotion.log";
		if (std::filesystem::file_size(cur, ec) > 20000) {
			std::filesystem::copy_file(cur, *dir / "FluidMotion_prev.log", std::filesystem::copy_options::overwrite_existing, ec);
		}
	}
	SKSE::Init(a_skse, { .logPattern = "[%H:%M:%S.%e] [%l] %v", .trampoline = true, .trampolineSize = 2048 });
	spdlog::default_logger()->flush_on(spdlog::level::info);

	CrashLog::Install();
	const auto ver = a_skse->RuntimeVersion();
	logger::info("Fluid Motion: Pathfinding Overhaul loaded, game runtime {}", ver.string());

	SKSE::GetMessagingInterface()->RegisterListener(OnMessage);
	return true;
}
