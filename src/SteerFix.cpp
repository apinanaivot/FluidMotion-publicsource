#include "SteerFix.h"

#include <xbyak/xbyak.h>

namespace SteerFix
{
	namespace
	{
		constexpr std::uintptr_t kPatch = 0x12466bc;
		constexpr std::uint8_t   kOriginal[] = { 0xF3, 0x0F, 0x10, 0x4D, 0xDF, 0x0F, 0x28, 0xD7, 0xF3, 0x0F, 0x59, 0xC6 };

		std::atomic<float> g_active{ 0.0f };
		bool               g_installed = false;

		struct Track
		{
			float         x, y;
			std::uint64_t ticks;
			float         speed;
		};
		std::mutex                                  g_lock;
		std::unordered_map<std::uintptr_t, Track>   g_tracks;

		double TicksPerSecond()
		{
			LARGE_INTEGER f;
			QueryPerformanceFrequency(&f);
			return static_cast<double>(f.QuadPart);
		}

		float Scale(float a_s, std::uintptr_t a_ctx)
		{
			const float look = g_active.load(std::memory_order_relaxed);
			if (look <= 0.0f || !a_ctx) {
				return 2.0f * a_s;
			}
			static const double tps = TicksPerSecond();
			const auto*         pos = reinterpret_cast<const float*>(a_ctx);
			const auto          key = *reinterpret_cast<const std::uintptr_t*>(a_ctx + 0x28);
			LARGE_INTEGER       now;
			QueryPerformanceCounter(&now);
			float speed = 0.0f;
			{
				std::scoped_lock l(g_lock);
				if (g_tracks.size() > 4096) {
					g_tracks.clear();
				}
				auto [it, fresh] = g_tracks.try_emplace(key, Track{ pos[0], pos[1], static_cast<std::uint64_t>(now.QuadPart), 0.0f });
				auto& t = it->second;
				if (!fresh) {
					const double dt = (now.QuadPart - t.ticks) / tps;
					if (dt > 0.008 && dt < 0.25) {
						const float dx = pos[0] - t.x, dy = pos[1] - t.y;
						t.speed = static_cast<float>(std::sqrt(dx * dx + dy * dy) / dt);
						t.x = pos[0];
						t.y = pos[1];
						t.ticks = now.QuadPart;
					} else if (dt >= 0.25) {
						t = Track{ pos[0], pos[1], static_cast<std::uint64_t>(now.QuadPart), 0.0f };
					}
				}
				speed = t.speed;
			}
			return 2.0f * (std::max)(a_s, speed * look);
		}

		struct Cave : Xbyak::CodeGenerator
		{
			Cave(std::uintptr_t a_fn, std::uintptr_t a_return)
			{
				sub(rsp, 0x20);
				mov(rdx, rdi);
				mov(rax, a_fn);
				call(rax);
				add(rsp, 0x20);
				movss(xmm1, ptr[rbp - 0x21]);
				movaps(xmm2, xmm7);
				mov(rax, a_return);
				jmp(rax);
			}
		};
	}

	void Install()
	{
		const auto site = REL::Module::get().base() + kPatch;
		if (std::memcmp(reinterpret_cast<const void*>(site), kOriginal, sizeof(kOriginal)) != 0) {
			logger::error("SteerFix: path look-ahead not as expected (different game version or another mod) - NOT installed");
			return;
		}
		Cave cave(reinterpret_cast<std::uintptr_t>(&Scale), site + sizeof(kOriginal));
		cave.ready();
		const auto code = SKSE::GetTrampoline().allocate(cave.getSize());
		std::memcpy(code, cave.getCode(), cave.getSize());
		std::uint8_t patch[sizeof(kOriginal)];
		std::memset(patch, 0x90, sizeof(patch));
		patch[0] = 0xE9;
		const auto rel = reinterpret_cast<std::intptr_t>(code) - static_cast<std::intptr_t>(site + 5);
		if (rel != static_cast<std::int32_t>(rel)) {
			logger::error("SteerFix: cave out of rel32 range - NOT installed");
			return;
		}
		const auto rel32 = static_cast<std::int32_t>(rel);
		std::memcpy(patch + 1, &rel32, sizeof(rel32));
		REL::safe_write(site, patch, sizeof(patch));
		g_installed = true;
		logger::info("SteerFix: path steering correction patched (image+{:#x})", kPatch);
	}

	bool IsInstalled() { return g_installed; }

	void Apply(bool a_fluid) { g_active = a_fluid && g_installed ? lookTime.load() : 0.0f; }
}
