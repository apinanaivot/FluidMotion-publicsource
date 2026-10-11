#include "CornerLimit.h"
#include "PathProbe.h"

namespace CornerLimit
{
	namespace
	{
		constexpr std::size_t    kVtable = 3;
		constexpr std::size_t    kSlot = 0xB;
		constexpr std::ptrdiff_t kSubOffset = 0x28;

		constexpr std::uint8_t kExpected[] = { 0x40, 0x53, 0x48, 0x81, 0xEC, 0x90, 0x00, 0x00, 0x00, 0x0F, 0x29, 0xB4,
			0x24, 0x80, 0x00, 0x00, 0x00, 0x48, 0x8B, 0xD9, 0x8B, 0x89, 0xF0, 0x00, 0x00, 0x00 };

		using Limit_t = bool (*)(std::uintptr_t, float, float, float, float, float*);
		Limit_t g_limit = nullptr;
		bool    g_installed = false;

		constexpr std::ptrdiff_t kProgress = 0x100;
		constexpr std::ptrdiff_t kState = 0x118;
		constexpr std::ptrdiff_t kPath = 0x128;
		constexpr float          kLookAhead = 4000.0f;
		constexpr REL::ID        kPathDistance{ 90977 };

		std::optional<float> FilletLimit(std::uintptr_t a_base, float a_dec, float a_omega, bool& a_limited)
		{
			a_limited = false;
			const auto state = *reinterpret_cast<std::uint32_t*>(a_base + kState);
			auto*      path = *reinterpret_cast<RE::BSPathingSolution**>(a_base + kPath);
			const float progress = *reinterpret_cast<float*>(a_base + kProgress);
			if ((state != 4 && state != 0xD) || !path || !(progress >= 0.0f) || a_dec <= 0.0f || a_omega <= 0.0f) {
				return std::nullopt;
			}
			const auto& nodes = path->currentPathingNodes;
			const auto  n = nodes.size();
			using PathDistance_t = float (*)(RE::BSPathingSolution*, float, float);
			static REL::Relocation<PathDistance_t> pathDistance{ kPathDistance };

			float limit = (std::numeric_limits<float>::max)();
			float d = 0.0f;
			for (auto k = static_cast<std::uint32_t>(progress) + 1; k + 1 < n; ++k) {
				d += k == static_cast<std::uint32_t>(progress) + 1 ? pathDistance(path, progress, static_cast<float>(k)) :
				                                                     pathDistance(path, static_cast<float>(k - 1), static_cast<float>(k));
				if (d > kLookAhead) {
					break;
				}
				if (nodes[k].actionAtNode) {
					return std::nullopt;
				}
				const auto& a = nodes[k - 1].location.location;
				const auto& b = nodes[k].location.location;
				const auto& c = nodes[k + 1].location.location;
				const float x1 = b.x - a.x, y1 = b.y - a.y, x2 = c.x - b.x, y2 = c.y - b.y;
				const float l1 = std::sqrt(x1 * x1 + y1 * y1), l2 = std::sqrt(x2 * x2 + y2 * y2);
				if (l1 < 1.0f || l2 < 1.0f) {
					continue;
				}
				const float th = std::acos(std::clamp((x1 * x2 + y1 * y2) / (l1 * l2), -1.0f, 1.0f));
				if (th < 0.01f) {
					continue;
				}
				const float r = (std::min)(l1, l2) * 0.5f / std::tan(th * 0.5f);
				const float vc = r * a_omega;
				limit = (std::min)(limit, std::sqrt(vc * vc + 2.0f * a_dec * (std::max)(d, 0.0f)));
			}
			if (limit < 1.0e5f) {
				a_limited = true;
			}
			return limit;
		}

		bool LimitHook(std::uintptr_t a_this, float a_angVel, float a_angAcc, float a_dec, float a_omega, float* a_out)
		{
			float      vanilla = -1.0f;
			const bool hasVanilla = g_limit(a_this, a_angVel, a_angAcc, a_dec, a_omega, &vanilla);
			bool       result = hasVanilla;
			float      used = hasVanilla ? vanilla : -1.0f;
			if (enabled.load(std::memory_order_relaxed)) {
				bool limited = false;
				std::optional<float> fillet;
				__try {
					fillet = FilletLimit(a_this - kSubOffset, a_dec, a_omega * omegaScale.load(std::memory_order_relaxed), limited);
				} __except (EXCEPTION_EXECUTE_HANDLER) {
					fillet = std::nullopt;
				}
				if (fillet) {
					result = limited;
					used = limited ? *fillet : -1.0f;
				}
			}
			if (result) {
				*a_out = used;
			}
			PathProbe::RecordCorner(a_this - kSubOffset,
				{ a_angVel, a_angAcc, a_dec, a_omega, hasVanilla ? vanilla : -1.0f, used });
			return result;
		}
	}

	void Install()
	{
		REL::Relocation<std::uintptr_t> vtbl{ RE::VTABLE_MovementAgentPathFollowerStandard[kVtable] };
		const auto fn = *reinterpret_cast<std::uintptr_t*>(vtbl.address() + kSlot * sizeof(void*));
		if (std::memcmp(reinterpret_cast<const void*>(fn), kExpected, sizeof(kExpected)) != 0) {
			logger::error("CornerLimit: path follower corner-speed limiter not as expected (different game version or another mod) - NOT installed");
			return;
		}
		g_limit = reinterpret_cast<Limit_t>(vtbl.write_vfunc(kSlot, &LimitHook));
		g_installed = true;
		logger::info("CornerLimit: hooked path follower corner-speed limiter (image+{:#x})", fn - REL::Module::get().base());
	}

	bool IsInstalled() { return g_installed; }
}
