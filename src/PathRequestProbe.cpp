#include "PathRequestProbe.h"

namespace PathRequestProbe
{
	namespace
	{
		constexpr std::uintptr_t kCalls[] = { 0x46c7c8, 0x6f4e49, 0x709841, 0x709db0, 0x70a2d0, 0x7159c7, 0x715e9e, 0x72df3a };
		constexpr std::uintptr_t kCtor = 0x4f4e80;
		std::atomic<std::uint32_t> g_count[std::size(kCalls)]{};
		std::uint32_t              g_logged[std::size(kCalls)]{};

		template <int N>
		struct Thunk
		{
			static std::uintptr_t thunk(std::uintptr_t a_this)
			{
				g_count[N].fetch_add(1, std::memory_order_relaxed);
				return func(a_this);
			}
			static inline REL::Relocation<decltype(thunk)> func;
		};

		template <int N>
		bool Hook(std::uintptr_t a_base)
		{
			const auto site = a_base + kCalls[N];
			if (*reinterpret_cast<const std::uint8_t*>(site) != 0xE8 ||
				site + 5 + *reinterpret_cast<const std::int32_t*>(site + 1) != a_base + kCtor) {
				return false;
			}
			Thunk<N>::func = SKSE::GetTrampoline().write_call<5>(site, Thunk<N>::thunk);
			return true;
		}
	}

	void Install()
	{
		const auto base = REL::Module::get().base();
		const bool ok = Hook<0>(base) & Hook<1>(base) & Hook<2>(base) & Hook<3>(base) &
		                Hook<4>(base) & Hook<5>(base) & Hook<6>(base) & Hook<7>(base);
		logger::info("PathRequestProbe: flee path request counters {}", ok ? "installed" : "partly installed (unexpected bytes)");
	}

	void LogIfDue()
	{
		static auto last = std::chrono::steady_clock::now();
		const auto  now = std::chrono::steady_clock::now();
		if (now - last < std::chrono::seconds(20)) {
			return;
		}
		last = now;
		std::string   line;
		std::uint32_t total = 0;
		for (std::size_t i = 0; i < std::size(kCalls); ++i) {
			const auto cur = g_count[i].load(std::memory_order_relaxed);
			total += cur - g_logged[i];
			line += std::format(" {:#x}={}", 0x140000000 + kCalls[i], cur - g_logged[i]);
			g_logged[i] = cur;
		}
		if (total) {
			logger::info("FleeReq (last 20s, PathingRequestFlee built at call site):{}", line);
		}
	}
}
