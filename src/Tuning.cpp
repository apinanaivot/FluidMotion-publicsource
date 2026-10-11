#include "Tuning.h"
#include "Config.h"
#include "CornerLimit.h"
#include "EndBrake.h"
#include "GameSettings.h"
#include "JumpNav.h"
#include "SpiderLegs.h"
#include "SteerFix.h"
#include "Telemetry.h"
#include "TurnFix.h"

namespace Tuning
{
	namespace
	{
		constexpr auto kIniPath = L"Data/SKSE/Plugins/FluidMotion.ini";
		constexpr auto kSection = "FluidMotion";

		std::atomic<Mode> g_mode{ Mode::kFluid };
		std::atomic_bool  g_pathfinding{ true };

		struct Setting
		{
			const char*         key;
			std::atomic<float>* value;
			float               min, max;
		};
		const Setting kSettings[] = {
			{ "fSteerLookTime", &SteerFix::lookTime, 0.0f, 2.0f },
			{ "fCornerTurnRateScale", &CornerLimit::omegaScale, 0.5f, 8.0f },
			{ "fTurnInPlaceAngle", &TurnFix::turnAngle, 15.0f, 180.0f },
			{ "fEndDecel", &EndBrake::endDecel, 354.0f, 5000.0f },
		};

		void LoadIni()
		{
			CSimpleIniA ini;
			ini.SetUnicode();
			ini.LoadFile(kIniPath);
			g_mode = ini.GetBoolValue(kSection, "bEnabled", true) ? Mode::kFluid : Mode::kVanilla;
			g_pathfinding = ini.GetBoolValue(kSection, "bPathfinding", true);
			for (auto& s : kSettings) {
				const auto v = static_cast<float>(ini.GetDoubleValue(kSection, s.key, s.value->load()));
				s.value->store(std::clamp(v, s.min, s.max));
			}
		}

		void Apply()
		{
			const bool fluid = g_mode == Mode::kFluid;
			const bool path = fluid && g_pathfinding;
			SteerFix::Apply(path);
			CornerLimit::enabled = path && CornerLimit::IsInstalled();
			TurnFix::Apply(path);
			EndBrake::enabled = path && EndBrake::IsInstalled();
			JumpNav::enabled = path && Config::Get().jump.enabled;
			SpiderLegs::ApplyMovement(fluid && Config::Get().spider.enabled && Config::Get().spider.omni);
			GameSettings::RestoreAll();
			if (fluid) {
				for (auto& [name, raw] : Config::Get().gmstOverrides) {
					GameSettings::SetRaw(name, raw);
				}
			}
		}
	}

	void Init() { Reload(); }

	void Reload()
	{
		LoadIni();
		Apply();
		logger::info("Mode: {}", ModeLabel());
	}

	Mode GetMode() { return g_mode; }

	bool GetPathfinding() { return g_pathfinding; }

	void SetPathfinding(bool a_on)
	{
		g_pathfinding = a_on;
		Refresh();
	}

	void Refresh()
	{
		SKSE::GetTaskInterface()->AddTask([] { Apply(); });
	}

	const char* ModeLabel() { return g_mode == Mode::kFluid ? "fluid" : "vanilla"; }

	void SetMode(Mode a_mode)
	{
		SKSE::GetTaskInterface()->AddTask([a_mode] {
			if (g_mode == a_mode) {
				return;
			}
			Telemetry::EndAllEpisodes();
			g_mode = a_mode;
			Apply();
			logger::info("Mode switched to {}", ModeLabel());
		});
	}
}
