#include "Menu.h"
#include "Config.h"
#include "DebugDraw.h"
#include "Locomotion.h"
#include "SKSEMenuFramework.h"
#include "Telemetry.h"
#include "Tuning.h"

namespace Menu
{
	namespace
	{
		namespace UI = ImGuiMCP;

		bool g_dirty = false;

		void Save()
		{
			constexpr auto path = L"Data/SKSE/Plugins/FluidMotion.ini";
			CSimpleIniA    ini;
			ini.SetUnicode();
			ini.LoadFile(path);
			auto b = [&](const char* s, const char* k, bool v) { ini.SetBoolValue(s, k, v); };
			auto f = [&](const char* s, const char* k, float v) { ini.SetValue(s, k, std::format("{:g}", v).c_str()); };
			const auto& c = Locomotion::Settings();
			b("FluidMotion", "bEnabled", Tuning::GetMode() == Tuning::Mode::kFluid);
			b("FluidMotion", "bPathfinding", Tuning::GetPathfinding());
			b("SpiderClimb", "bEnabled", Config::Get().climb.enabled);
			b("Spider", "bEnabled", Config::Get().spider.enabled);
			b("Anim", "bEnabled", c.enabled);
			b("Anim", "bPlayer", c.player);
			b("Anim", "bPlayerMotion", c.playerMotion);
			b("Anim", "bNpcs", c.npcs);
			b("Anim", "bSpeedSync", c.speedSync);
			b("Anim", "bDuck", c.duck);
			b("Anim", "bSlopeLean", c.slopeLean);
			b("Anim", "bTurnLean", c.turnLean);
			b("Anim", "bHeadLead", c.headLead);
			b("Anim", "bLandCrouch", c.landCrouch);
			b("Anim", "bStopSettle", c.stopSettle);
			b("Anim", "bMotionMatch", c.motionMatch);
			b("Anim", "bPlayerHeading", c.playerHeading);
			b("Anim", "bCombatFreeCamera", c.combatFreeCamera);
			b("Anim", "bAttackTurn", c.attackTurn);
			b("Anim", "bPlayerClimb", c.playerClimb);
			b("Anim", "bPlayerAccel", c.playerAccelOn);
			f("Anim", "fPlayerAccel", c.playerAccel);
			b("Anim", "bFirstPersonMotion", c.firstPersonMotion);
			b("Anim", "bVertSpring", c.vertSpring);
			f("Anim", "fVertHalf", c.vertHalf);
			f("Anim", "fCamStiffDist", c.camStiffDist);
			b("Debug", "bShowNavmesh", DebugDraw::showNavmesh);
			b("Debug", "bShowPaths", DebugDraw::showPaths);
			f("Debug", "fRadius", DebugDraw::radius);
			if (ini.SaveFile(path) < 0) {
				logger::error("Menu: could not write FluidMotion.ini");
			} else {
				logger::info("Menu: settings saved to FluidMotion.ini");
			}
		}

		void SaveIfChanged()
		{
			if (g_dirty && !UI::IsAnyItemActive()) {
				g_dirty = false;
				SKSE::GetTaskInterface()->AddTask(Save);
			}
		}

		void Tip(const char* a_text)
		{
			if (UI::BeginItemTooltip()) {
				UI::PushTextWrapPos(UI::GetFontSize() * 30.0f);
				UI::TextUnformatted(a_text);
				UI::PopTextWrapPos();
				UI::EndTooltip();
			}
		}

		bool Check(const char* a_label, bool* a_value, const char* a_tip)
		{
			const bool changed = UI::Checkbox(a_label, a_value);
			Tip(a_tip);
			g_dirty |= changed;
			return changed;
		}

		void Slider(const char* a_label, float* a_value, float a_min, float a_max, const char* a_fmt, const char* a_tip)
		{
			UI::SetNextItemWidth(UI::GetFontSize() * 12.0f);
			g_dirty |= UI::SliderFloat(a_label, a_value, a_min, a_max, a_fmt);
			Tip(a_tip);
		}

		bool Radio(const char* a_label, int* a_value, int a_button, const char* a_tip)
		{
			const bool changed = UI::RadioButton(a_label, a_value, a_button);
			Tip(a_tip);
			g_dirty |= changed;
			return changed;
		}

		void Heading(const char* a_text)
		{
			UI::Spacing();
			UI::SeparatorText(a_text);
		}

		bool BeginFluidPage()
		{
			const bool off = Tuning::GetMode() != Tuning::Mode::kFluid;
			if (off) {
				UI::TextDisabled("Fluid Motion is off (General page).");
			}
			UI::BeginDisabled(off);
			return off;
		}

		void __stdcall DrawGeneral()
		{
			const auto mode = Tuning::GetMode();
			int        m = mode == Tuning::Mode::kFluid ? 1 : 0;
			UI::TextUnformatted("Fluid Motion");
			static constexpr const char* kTip = "Turns every Fluid Motion feature off (pathing, jumps, animation, camera).";
			UI::SameLine();
			bool changed = Radio("Off (vanilla)", &m, 0, kTip);
			UI::SameLine();
			changed |= Radio("On", &m, 1, kTip);
			if (changed) {
				Tuning::SetMode(m ? Tuning::Mode::kFluid : Tuning::Mode::kVanilla);
			}

			BeginFluidPage();
			Heading("Pathfinding (NPCs, creatures, followers)");
			bool path = Tuning::GetPathfinding();
			if (Check("New pathfinding", &path, "Smooth path following (steering, corners, turns, stops) and jump / off-navmesh routes.")) {
				Tuning::SetPathfinding(path);
			}
			Check("Spiders crawl on walls", &Config::Mutable().climb.enabled, "Spiders pursue targets over walls and ceilings. Off: they use the same pathfinding as other NPCs and stay on the ground.");
			UI::EndDisabled();
			SaveIfChanged();
		}

		void __stdcall DrawAnim()
		{
			BeginFluidPage();
			auto& c = Locomotion::Settings();
			{
				static constexpr const char* kTip = "Turns off everything on this page and the Player page, climbing and spider legs too.";
				int                          a = c.enabled ? 1 : 0;
				UI::TextUnformatted("Animation layer");
				UI::SameLine();
				bool changed = Radio("Off##anim", &a, 0, kTip);
				UI::SameLine();
				changed |= Radio("On##anim", &a, 1, kTip);
				if (changed) {
					c.enabled = a != 0;
				}
			}
			UI::BeginDisabled(!c.enabled);
			Check("Apply to the player", &c.player, "Off: the player is left completely vanilla (no pose effects, and nothing on the Player page).");
			Check("Apply to NPCs", &c.npcs, "Nearby NPCs and creatures get the animation layer.");
			if (Check("Spider leg IK", &Config::Mutable().spider.enabled, "Spider legs are placed on the ground by IK (and spiders move sideways / backwards).")) {
				Tuning::Refresh();
			}
			if (UI::TreeNode("Individual effects")) {
				Check("Speed sync", &c.speedSync, "Animation follows real speed: no running in place at a wall.");
				Check("Duck", &c.duck, "Bend down under low ceilings.");
				Check("Slope lean", &c.slopeLean, "Lean into uphill and downhill slopes.");
				Check("Turn lean", &c.turnLean, "Bank into turns while running.");
				Check("Head leads", &c.headLead, "The head looks into the turn first.");
				Check("Landing crouch", &c.landCrouch, "Crouch after a long fall.");
				Check("Stop settle", &c.stopSettle, "Small dip when braking from a run.");
				UI::TreePop();
			}
			UI::EndDisabled();
			UI::EndDisabled();
			SaveIfChanged();
		}

		void __stdcall DrawPlayer()
		{
			BeginFluidPage();
			auto& c = Locomotion::Settings();
			if (!c.enabled || !c.player) {
				UI::TextDisabled("The animation layer is off for the player (Animation page).");
			}
			UI::BeginDisabled(!c.enabled || !c.player);
			Check("Player movement and camera changes", &c.playerMotion,
				"Off: vanilla player movement, turning and camera (no acceleration, no facing the movement direction, no camera spring, no motion matching, no ledge climbing); True Directional Movement and other movement mods work as without this mod. The pose effects of the Animation page still apply to the player.");
			UI::BeginDisabled(!c.playerMotion);

			Heading("Motion matching");
			Check("Motion matching", &c.motionMatch, "Legs and hips come from a mocap database while you walk or run.");

			Heading("Movement");
			Check("Directional movement", &c.playerHeading, "Third person: the body faces where you run independent of the camera. Replaces True Directional Movement's directional movement when that is installed.");
			UI::BeginDisabled(!c.playerHeading);
			UI::Indent();
			Check("Also with a weapon drawn", &c.combatFreeCamera, "Directional movement with a weapon drawn too: the body turns and runs forward wherever you go. Off: with a weapon drawn the body keeps facing the camera and uses the richer sideways and backwards strafing animations (sheathed it stays directional, also in combat).");
			UI::BeginDisabled(!c.combatFreeCamera);
			Check("Attacking and blocking turn the body to the camera", &c.attackTurn, "Off: the body keeps facing the movement direction while you attack or block.");
			UI::EndDisabled();
			UI::Unindent();
			UI::EndDisabled();
			Check("Ledge climbing", &c.playerClimb, "Grab and pull up onto ledges after a jump. Off: you never climb (NPCs still do).");
			Check("Acceleration", &c.playerAccelOn, "Your run speed builds up and drops smoothly. Off: vanilla instant starts and stops.");
			UI::BeginDisabled(!c.playerAccelOn);
			UI::Indent();
			Slider("Acceleration time (s)", &c.playerAccel, 0.0f, 0.5f, "%.2f", "How slowly your run speed builds up and drops.");
			Check("Also in first person", &c.firstPersonMotion, "Off: first person uses vanilla acceleration.");
			UI::Unindent();
			UI::EndDisabled();

			Heading("Camera");
			Check("Camera spring", &c.vertSpring, "Smooths the camera height (steps, bumps, landings).");
			if (GetModuleHandleA("SmoothCam.dll")) {
				UI::TextDisabled("SmoothCam is installed and used for the third-person camera. The spring only smooths height in first person.");
			}
			UI::BeginDisabled(!c.vertSpring);
			UI::Indent();
			if (UI::TreeNode("Spring tuning")) {
				Slider("Half-life (s)", &c.vertHalf, 0.03f, 0.8f, "%.2f", "Time for the camera lag to halve. Lower is snappier.");
				Slider("Stiffens at (u)", &c.camStiffDist, 5.0f, 150.0f, "%.0f", "The further the camera lags in height, the stiffer; twice as stiff at this distance.");
				UI::TreePop();
			}
			UI::Unindent();
			UI::EndDisabled();

			UI::EndDisabled();
			UI::EndDisabled();
			UI::EndDisabled();
			SaveIfChanged();
		}

		void __stdcall DrawDiagnostics()
		{
			UI::TextDisabled("Debugging only.");
			bool trace = Telemetry::IsTraceEnabled();
			if (Check("Record trace CSV", &trace, "Writes every sample of tracked actors to FluidMotion_trace_<time>.csv next to the log.")) {
				Telemetry::SetTraceEnabled(trace);
			}
			bool nav = DebugDraw::showNavmesh;
			if (Check("Navmesh wireframe", &nav, "Draws navmesh triangles, open edges and ledge links around you.")) {
				DebugDraw::showNavmesh = nav;
			}
			bool paths = DebugDraw::showPaths;
			if (Check("Paths and jumps", &paths, "Draws actor paths, planned jump arcs and rejected jumps.")) {
				DebugDraw::showPaths = paths;
			}
			float r = DebugDraw::radius;
			Slider("Wireframe radius (u)", &r, 500.0f, 6000.0f, "%.0f", "How far around you the wireframe is drawn.");
			DebugDraw::radius = r;
			SaveIfChanged();
		}
	}

	void Register()
	{
		if (!SKSEMenuFramework::IsInstalled()) {
			logger::error("SKSE Menu Framework is not installed: the Fluid Motion menu and the debug overlay are unavailable (settings stay in FluidMotion.ini)");
			return;
		}
		SKSEMenuFramework::SetSection("Fluid Motion");
		SKSEMenuFramework::AddSectionItem("General", DrawGeneral);
		SKSEMenuFramework::AddSectionItem("Animation", DrawAnim);
		SKSEMenuFramework::AddSectionItem("Player", DrawPlayer);
		SKSEMenuFramework::AddSectionItem("Diagnostics", DrawDiagnostics);
		SKSEMenuFramework::AddHudElement(DebugDraw::Draw);
		logger::info("SKSE Menu Framework: Fluid Motion pages registered");
	}
}
