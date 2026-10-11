#include "FrameHook.h"
#include "DebugDraw.h"
#include "JumpNav.h"
#include "Locomotion.h"
#include "SpiderClimb.h"
#include "SpiderLegs.h"
#include "SwingFix.h"

namespace FrameHook
{
	namespace
	{
		using Clock = std::chrono::steady_clock;

		struct Stat
		{
			Clock::time_point start = Clock::now(), lastFrame = Clock::now();
			double            ours = 0, maxOurs = 0, maxFrame = 0;
			int               frames = 0;
		};

		struct PlayerUpdate
		{
			static void thunk(RE::PlayerCharacter* a_this, float a_delta)
			{
				static Stat s;
				const auto  t0 = Clock::now();
				func(a_this, a_delta);
				const auto t1 = Clock::now();
				float      dt = RE::GetSecondsSinceLastFrame();
				auto*      ui = RE::UI::GetSingleton();
				auto*      main = RE::Main::GetSingleton();
				const bool paused = (ui && ui->GameIsPaused()) || (main && main->GetRuntimeData().freezeTime);
				if (paused) {
					dt = 0.0f;
				}
				static float logT = 0.0f;
				static bool  wasPaused = false;
				if (paused != wasPaused || ((logT += a_delta) > 5.0f && !paused && std::fabs(dt - a_delta) > 0.2f * a_delta)) {
					logT = 0.0f;
					wasPaused = paused;
					logger::info("FRAMEDT player {:.4f} world {:.4f} paused {}", a_delta, RE::GetSecondsSinceLastFrame(), paused);
				}
				JumpNav::OnFrame(dt);
				SwingFix::OnFrame(dt);
				Locomotion::OnFrame(dt);
				SpiderClimb::OnFrame(dt);
				SpiderLegs::OnFrame(dt);
				DebugDraw::Update();
				const auto t2 = Clock::now();

				const double ours = std::chrono::duration<double, std::milli>(t2 - t1).count();
				const double frame = std::chrono::duration<double, std::milli>(t0 - s.lastFrame).count();
				s.lastFrame = t0;
				s.ours += ours;
				s.maxOurs = (std::max)(s.maxOurs, ours);
				s.maxFrame = (std::max)(s.maxFrame, frame);
				++s.frames;
				if (t2 - s.start >= std::chrono::seconds(10)) {
					const double span = std::chrono::duration<double>(t2 - s.start).count();
					logger::info("FRAMESTAT fps={:.1f} ours avg={:.3f}ms max={:.2f}ms longestFrame={:.1f}ms", s.frames / span, s.ours / s.frames,
						s.maxOurs, s.maxFrame);
					s = Stat{};
				}
			}
			static inline REL::Relocation<decltype(thunk)> func;
		};
	}

	void Install()
	{
		REL::Relocation<std::uintptr_t> vtbl{ RE::VTABLE_PlayerCharacter[0] };
		PlayerUpdate::func = vtbl.write_vfunc(0xAD, PlayerUpdate::thunk);
		logger::info("FrameHook: hooked PlayerCharacter::Update");
	}
}
