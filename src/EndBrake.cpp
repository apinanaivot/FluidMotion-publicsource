#include "EndBrake.h"
#include "PathProbe.h"

namespace EndBrake
{
	namespace
	{
		constexpr REL::ID kPathDistance{ 90977 };

		constexpr std::ptrdiff_t kRecSpeed = 0x4C;
		constexpr std::ptrdiff_t kRecFlag = 0x5C;
		constexpr std::ptrdiff_t kRecDist = 0x60;
		constexpr std::ptrdiff_t kRecEnd = 0x64;
		constexpr std::ptrdiff_t kRecAcc = 0x70;
		constexpr std::ptrdiff_t kRecDec = 0x74;

		constexpr std::ptrdiff_t kProgress = 0x100;
		constexpr std::ptrdiff_t kState = 0x118;
		constexpr std::ptrdiff_t kPath = 0x128;

		bool g_installed = false;

		using Update_t = std::uintptr_t (*)(std::uintptr_t, std::uintptr_t, std::uintptr_t, std::uintptr_t);
		Update_t g_update = nullptr;

		Decision DecideImpl(std::uintptr_t a_this, std::uintptr_t a_rec, float& a_nodeDist)
		{
			if (!*reinterpret_cast<std::uint8_t*>(a_rec + kRecFlag)) {
				return kNone;
			}
			const auto link = *reinterpret_cast<std::uintptr_t*>(a_this - 8);
			const auto req = PathProbe::GetRequestByLink(link);
			if (!req || !req->flag || GetTickCount64() - req->ms > 250) {
				return kNoFollower;
			}
			const auto  f = req->follower;
			const auto  state = *reinterpret_cast<std::uint32_t*>(f + kState);
			auto*       path = *reinterpret_cast<RE::BSPathingSolution**>(f + kPath);
			const float progress = *reinterpret_cast<float*>(f + kProgress);
			if (PathProbe::recording.load(std::memory_order_relaxed) && (state == 4 || state == 0xD) && path && progress >= 0.0f) {
				auto k = static_cast<std::uint32_t>(progress) + 1;
				if (static_cast<float>(k) - progress < 0.01f) {
					++k;
				}
				if (k < path->currentPathingNodes.size()) {
					using PathDistance_t = float (*)(RE::BSPathingSolution*, float, float);
					static REL::Relocation<PathDistance_t> pathDistance{ kPathDistance };
					a_nodeDist = pathDistance(path, progress, static_cast<float>(k));
				}
			}
			if (!enabled.load(std::memory_order_relaxed)) {
				return kOff;
			}
			const float dist = *reinterpret_cast<float*>(a_rec + kRecDist);
			const float end = *reinterpret_cast<float*>(a_rec + kRecEnd);
			if (req->dist > dist + 1.0f) {
				return kUntouched;
			}
			auto&       dec = *reinterpret_cast<float*>(a_rec + kRecDec);
			const float target = *reinterpret_cast<float*>(a_rec + kRecSpeed);
			const float want = endDecel.load(std::memory_order_relaxed);
			if (dec > 0.0f && dec < want && end * end + 2.0f * dec * dist < target * target) {
				dec = want;
				return kEndDecel;
			}
			return kUntouched;
		}

		std::uintptr_t UpdateHook(std::uintptr_t a_this, std::uintptr_t a_recPtr, std::uintptr_t a_r8, std::uintptr_t a_r9)
		{
			PathProbe::Translation tr{};
			tr.nodeDist = -1.0f;
			bool           haveRec = false;
			std::uintptr_t link = 0;
			Decision       why = kFault;
			__try {
				const auto rec = a_recPtr ? *reinterpret_cast<std::uintptr_t*>(a_recPtr) : 0;
				if (rec) {
					link = *reinterpret_cast<std::uintptr_t*>(a_this - 8);
					tr.speed = *reinterpret_cast<float*>(rec + kRecSpeed);
					tr.dist = *reinterpret_cast<float*>(rec + kRecDist);
					tr.end = *reinterpret_cast<float*>(rec + kRecEnd);
					tr.acc = *reinterpret_cast<float*>(rec + kRecAcc);
					tr.dec = *reinterpret_cast<float*>(rec + kRecDec);
					tr.flag = *reinterpret_cast<std::uint8_t*>(rec + kRecFlag);
					haveRec = true;
					why = DecideImpl(a_this, rec, tr.nodeDist);
				}
			} __except (EXCEPTION_EXECUTE_HANDLER) {
				why = kFault;
			}
			if (haveRec && link) {
				tr.fix = why;
				PathProbe::RecordTranslation(link, tr);
			}
			return g_update(a_this, a_recPtr, a_r8, a_r9);
		}
	}

	void Install()
	{
		REL::Relocation<std::uintptr_t> vtbl{ RE::VTABLE_MovementHandlerAgentTranslationController[1] };
		const auto expected = REL::ID{ 92284 }.address();
		const auto current = *reinterpret_cast<std::uintptr_t*>(vtbl.address() + 2 * sizeof(void*));
		if (current != expected) {
			logger::error("EndBrake: translation controller vtable not as expected (different game version or another mod) - NOT installed");
			return;
		}
		g_update = reinterpret_cast<Update_t>(vtbl.write_vfunc(2, &UpdateHook));
		g_installed = true;
		logger::info("EndBrake: hooked translation controller update (image+{:#x})", expected - REL::Module::get().base());
	}

	bool IsInstalled() { return g_installed; }
}
