#include "TurnFix.h"

namespace TurnFix
{
	namespace
	{
		constexpr float kVanillaAngle = 15.0f;

		constexpr std::uintptr_t kTurnInsn = 0x11d1429;
		constexpr std::uint8_t   kTurnOpcode[] = { 0xF3, 0x0F, 0x10, 0x05 };

		float* g_threshold = nullptr;
	}

	void Install()
	{
		const auto insn = REL::Module::get().base() + kTurnInsn;
		const auto target = insn + 8 + *reinterpret_cast<const std::int32_t*>(insn + 4);
		if (std::memcmp(reinterpret_cast<const void*>(insn), kTurnOpcode, sizeof(kTurnOpcode)) != 0 ||
			*reinterpret_cast<const float*>(target) != kVanillaAngle) {
			logger::error("TurnFix: NeedsTurn not as expected (different game version or another mod) - NOT installed");
			return;
		}
		auto* threshold = SKSE::GetTrampoline().allocate<float>();
		*threshold = kVanillaAngle;
		const auto disp = reinterpret_cast<std::intptr_t>(threshold) - static_cast<std::intptr_t>(insn + 8);
		if (disp != static_cast<std::int32_t>(disp)) {
			logger::error("TurnFix: threshold out of rel32 range - NOT installed");
			return;
		}
		REL::safe_write(insn + 4, static_cast<std::int32_t>(disp));
		g_threshold = threshold;
		logger::info("TurnFix: turn-in-place threshold patched (image+{:#x})", kTurnInsn);
	}

	bool IsInstalled() { return g_threshold != nullptr; }

	void Apply(bool a_fluid)
	{
		if (g_threshold) {
			*g_threshold = a_fluid ? turnAngle.load() : kVanillaAngle;
		}
	}
}
