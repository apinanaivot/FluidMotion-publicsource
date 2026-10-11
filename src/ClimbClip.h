#pragma once

#include <fstream>
#include <memory>

namespace ClimbClip
{
	struct Frame
	{
		RE::NiPoint3 root;
		float        comZ = 0;
		RE::NiPoint3 hand[2];
	};

	struct Clip
	{
		std::vector<std::string> bones;
		std::vector<Frame>       frames;
		std::vector<float>       q;
		float                    dt = 1.0f / 60.0f, release = 0.5f;
		RE::NiPoint3             mid0;

		float Duration() const { return static_cast<float>(frames.size() - 1) * dt; }
		float Height() const { return frames.back().root.z; }

		void At(float a_t, std::size_t& a_f0, std::size_t& a_f1, float& a_u) const
		{
			const float f = std::clamp(a_t / dt, 0.0f, static_cast<float>(frames.size() - 1));
			a_f0 = static_cast<std::size_t>(f);
			a_f1 = (std::min)(a_f0 + 1, frames.size() - 1);
			a_u = f - static_cast<float>(a_f0);
		}
		Frame Sample(float a_t) const
		{
			std::size_t i, j;
			float       u;
			At(a_t, i, j, u);
			const Frame &A = frames[i], &B = frames[j];
			Frame       r;
			r.root = A.root + (B.root - A.root) * u;
			r.comZ = A.comZ + (B.comZ - A.comZ) * u;
			for (int h = 0; h < 2; ++h) {
				r.hand[h] = A.hand[h] + (B.hand[h] - A.hand[h]) * u;
			}
			return r;
		}
		void Rot(float a_t, std::size_t a_k, float (&a_out)[4]) const
		{
			std::size_t i, j;
			float       u;
			At(a_t, i, j, u);
			const float* A = &q[(i * bones.size() + a_k) * 4];
			const float* B = &q[(j * bones.size() + a_k) * 4];
			const float  s = (A[0] * B[0] + A[1] * B[1] + A[2] * B[2] + A[3] * B[3]) < 0.0f ? -1.0f : 1.0f;
			float        n = 0;
			for (int c = 0; c < 4; ++c) {
				a_out[c] = A[c] + (B[c] * s - A[c]) * u;
				n += a_out[c] * a_out[c];
			}
			n = 1.0f / std::sqrt(n);
			for (float& v : a_out) {
				v *= n;
			}
		}
	};

	enum Kind { kVault = 0, kHang = 1, kJumpStart = 2, kJumpFall = 3, kJumpLand = 4 };

	inline const Clip* Get(int a_kind = kVault, int a_rig = 0)
	{
		static std::unique_ptr<Clip> clips[10];
		static bool                  tried[10]{};
		static const char* const     giantFiles[3] = { "Data\\SKSE\\Plugins\\FluidMotion_jump_giant_start.bin", "Data\\SKSE\\Plugins\\FluidMotion_jump_giant_fall.bin",
			"Data\\SKSE\\Plugins\\FluidMotion_jump_giant_land.bin" };
		if (a_rig == 1) {
			if (a_kind < kJumpStart) {
				return nullptr;
			}
			a_kind += 5;
		}
		static const char* const     files[5] = { "Data\\SKSE\\Plugins\\FluidMotion_climb.bin", "Data\\SKSE\\Plugins\\FluidMotion_hang.bin",
			"Data\\SKSE\\Plugins\\FluidMotion_jump_start.bin", "Data\\SKSE\\Plugins\\FluidMotion_jump_fall.bin", "Data\\SKSE\\Plugins\\FluidMotion_jump_land.bin" };
		auto&                        clip = clips[a_kind];
		if (tried[a_kind]) {
			return clip.get();
		}
		tried[a_kind] = true;
		const char*   path = a_kind >= 5 ? giantFiles[a_kind - 5 - kJumpStart] : files[a_kind];
		std::ifstream in(path, std::ios::binary);
		auto          rd = [&](void* p, std::size_t n) { in.read(static_cast<char*>(p), static_cast<std::streamsize>(n)); };
		char          magic[4]{};
		std::uint32_t ver = 0, nb = 0, nf = 0;
		rd(magic, 4);
		rd(&ver, 4);
		rd(&nb, 4);
		if (!in || std::string_view(magic, 4) != "FMCL" || ver != 1 || nb == 0 || nb > 256) {
			logger::warn("ClimbClip: {} missing or invalid - that climb is off", path);
			return nullptr;
		}
		auto c = std::make_unique<Clip>();
		for (std::uint32_t i = 0; i < nb; ++i) {
			std::uint8_t n = 0;
			rd(&n, 1);
			std::string s(n, '\0');
			rd(s.data(), n);
			c->bones.push_back(std::move(s));
		}
		rd(&c->dt, 4);
		rd(&nf, 4);
		rd(&c->release, 4);
		rd(&c->mid0, 12);
		if (!in || nf < 2 || nf > 20000 || !(c->dt > 0.0f)) {
			return nullptr;
		}
		c->frames.resize(nf);
		c->q.resize(static_cast<std::size_t>(nf) * nb * 4);
		for (std::uint32_t f = 0; f < nf; ++f) {
			float v[10];
			rd(v, sizeof(v));
			auto& fr = c->frames[f];
			fr.root = { v[0], v[1], v[2] };
			fr.comZ = v[3];
			fr.hand[0] = { v[4], v[5], v[6] };
			fr.hand[1] = { v[7], v[8], v[9] };
			rd(&c->q[static_cast<std::size_t>(f) * nb * 4], nb * 16);
		}
		if (!in) {
			logger::warn("ClimbClip: {} truncated", path);
			return nullptr;
		}
		logger::info("ClimbClip: {} bones, {} frames, {:.2f}s, ledge {:.0f}u, release {:.2f}s", nb, nf, c->Duration(), c->Height(), c->release);
		clip = std::move(c);
		return clip.get();
	}
}
