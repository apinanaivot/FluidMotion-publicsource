#include "AngleProbe.h"
#include "Locomotion.h"
#include "PathProbe.h"

namespace AngleProbe
{
	namespace
	{
		using Update_t = std::uintptr_t (*)(std::uintptr_t, std::uintptr_t, std::uintptr_t, std::uintptr_t);
		Update_t g_update = nullptr;

		bool ReadInputs(std::uintptr_t a_this, std::uintptr_t a_ctx, std::uintptr_t& a_link, Sample& a_s)
		{
			__try {
				a_link = *reinterpret_cast<std::uintptr_t*>(a_this - 8);
				const auto rec = *reinterpret_cast<std::uintptr_t*>(a_ctx);
				if (!a_link || !rec) {
					return false;
				}
				a_s.desired = *reinterpret_cast<float*>(rec + 0x58);
				a_s.f50 = *reinterpret_cast<float*>(rec + 0x50);
				a_s.f6c = *reinterpret_cast<float*>(rec + 0x6C);
				a_s.wmax = *reinterpret_cast<float*>(rec + 0x7C);
				a_s.alpha = *reinterpret_cast<float*>(rec + 0x80);
				const auto vtbl = *reinterpret_cast<std::uintptr_t**>(a_link);
				float rot[3]{};
				reinterpret_cast<void (*)(std::uintptr_t, float*)>(vtbl[0x20 / 8])(a_link, rot);
				a_s.yaw = rot[2];
				a_s.w0 = reinterpret_cast<float (*)(std::uintptr_t)>(vtbl[0x30 / 8])(a_link);
				return true;
			} __except (EXCEPTION_EXECUTE_HANDLER) {
				return false;
			}
		}

		bool ReadOutput(std::uintptr_t a_ctx, float& a_cmd)
		{
			__try {
				const auto out = *reinterpret_cast<std::uintptr_t*>(a_ctx + 8);
				a_cmd = out ? *reinterpret_cast<float*>(out + 0x18) : 0.0f;
				return out != 0;
			} __except (EXCEPTION_EXECUTE_HANDLER) {
				return false;
			}
		}

		void SilenceIfDriven(std::uintptr_t a_this, std::uintptr_t a_ctx)
		{
			__try {
				const auto link = *reinterpret_cast<std::uintptr_t*>(a_this - 8);
				const auto out = a_ctx ? *reinterpret_cast<std::uintptr_t*>(a_ctx + 8) : 0;
				if (!link || !out) {
					return;
				}
				if (const auto id = PathProbe::ActorForKey(link); id && Locomotion::IsDriven(id)) {
					*reinterpret_cast<float*>(out + 0x18) = 0.0f;
				}
			} __except (EXCEPTION_EXECUTE_HANDLER) {
			}
		}

		std::uintptr_t UpdateHook(std::uintptr_t a_this, std::uintptr_t a_ctx, std::uintptr_t a_r8, std::uintptr_t a_r9)
		{
			if (!PathProbe::recording.load(std::memory_order_relaxed)) {
				const auto r = g_update(a_this, a_ctx, a_r8, a_r9);
				SilenceIfDriven(a_this, a_ctx);
				return r;
			}
			std::uintptr_t link = 0;
			Sample         s{};
			const bool     have = a_ctx && ReadInputs(a_this, a_ctx, link, s);
			const auto     result = g_update(a_this, a_ctx, a_r8, a_r9);
			if (have) {
				SilenceIfDriven(a_this, a_ctx);
			}
			if (have && ReadOutput(a_ctx, s.cmd)) {
				PathProbe::RecordAngle(link, s);
			}
			return result;
		}
	}

	void Install()
	{
		REL::Relocation<std::uintptr_t> vtbl{ RE::VTABLE_MovementHandlerAgentAngleController[1] };
		const auto expected = REL::Module::get().base() + 0x122b000;
		const auto current = *reinterpret_cast<std::uintptr_t*>(vtbl.address() + 2 * sizeof(void*));
		if (current != expected) {
			logger::error("AngleProbe: angle controller vtable not as expected - NOT installed");
			return;
		}
		g_update = reinterpret_cast<Update_t>(vtbl.write_vfunc(2, &UpdateHook));
		logger::info("AngleProbe: hooked heading controller update (image+0x122b000)");
	}
}
