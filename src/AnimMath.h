#pragma once

namespace AnimMath
{
	using V3 = RE::NiPoint3;
	using M3 = RE::NiMatrix3;

	constexpr float kPi = 3.14159265f;

	inline float Clamp(float a_v, float a_lo, float a_hi) { return a_v < a_lo ? a_lo : (a_v > a_hi ? a_hi : a_v); }
	inline float Smooth(float a_v, float a_lo, float a_hi)
	{
		const float t = Clamp((a_v - a_lo) / (a_hi - a_lo), 0.0f, 1.0f);
		return t * t * (3.0f - 2.0f * t);
	}
	inline float Approach(float a_x, float a_target, float a_tc, float a_dt)
	{
		return a_x + (a_target - a_x) * (1.0f - std::exp(-a_dt / a_tc));
	}
	inline float Wrap(float a_angle)
	{
		if (!std::isfinite(a_angle)) {
			return 0.0f;
		}
		return std::remainder(a_angle, 2.0f * kPi);
	}
	inline V3 Norm(const V3& a_v, const V3& a_fallback)
	{
		const float l = a_v.Length();
		return l > 1e-5f ? a_v * (1.0f / l) : a_fallback;
	}

	inline M3 AxisAngle(const V3& u, float a)
	{
		const float c = std::cos(a), s = std::sin(a), t = 1.0f - c;
		M3          m;
		m.entry[0][0] = t * u.x * u.x + c;
		m.entry[0][1] = t * u.x * u.y - s * u.z;
		m.entry[0][2] = t * u.x * u.z + s * u.y;
		m.entry[1][0] = t * u.x * u.y + s * u.z;
		m.entry[1][1] = t * u.y * u.y + c;
		m.entry[1][2] = t * u.y * u.z - s * u.x;
		m.entry[2][0] = t * u.x * u.z - s * u.y;
		m.entry[2][1] = t * u.y * u.z + s * u.x;
		m.entry[2][2] = t * u.z * u.z + c;
		return m;
	}

	inline M3 RotBetween(const V3& a_from, const V3& a_to)
	{
		const V3    axis = a_from.Cross(a_to);
		const float s = axis.Length(), c = a_from.Dot(a_to);
		if (s < 1e-6f) {
			if (c > 0.0f) {
				return M3{};
			}
			const V3 p = std::fabs(a_from.x) < 0.9f ? V3{ 1, 0, 0 } : V3{ 0, 1, 0 };
			return AxisAngle(Norm(a_from.Cross(p), { 0, 0, 1 }), kPi);
		}
		return AxisAngle(axis * (1.0f / s), std::atan2(s, c));
	}

	inline void ToQuat(const M3& m, float (&q)[4])
	{
		const auto& e = m.entry;
		const float tr = e[0][0] + e[1][1] + e[2][2];
		if (tr > 0.0f) {
			const float s = std::sqrt(tr + 1.0f) * 2.0f;
			q[0] = (e[2][1] - e[1][2]) / s, q[1] = (e[0][2] - e[2][0]) / s, q[2] = (e[1][0] - e[0][1]) / s, q[3] = 0.25f * s;
		} else if (e[0][0] > e[1][1] && e[0][0] > e[2][2]) {
			const float s = std::sqrt(1.0f + e[0][0] - e[1][1] - e[2][2]) * 2.0f;
			q[0] = 0.25f * s, q[1] = (e[0][1] + e[1][0]) / s, q[2] = (e[0][2] + e[2][0]) / s, q[3] = (e[2][1] - e[1][2]) / s;
		} else if (e[1][1] > e[2][2]) {
			const float s = std::sqrt(1.0f + e[1][1] - e[0][0] - e[2][2]) * 2.0f;
			q[0] = (e[0][1] + e[1][0]) / s, q[1] = 0.25f * s, q[2] = (e[1][2] + e[2][1]) / s, q[3] = (e[0][2] - e[2][0]) / s;
		} else {
			const float s = std::sqrt(1.0f + e[2][2] - e[0][0] - e[1][1]) * 2.0f;
			q[0] = (e[0][2] + e[2][0]) / s, q[1] = (e[1][2] + e[2][1]) / s, q[2] = 0.25f * s, q[3] = (e[1][0] - e[0][1]) / s;
		}
	}

	inline M3 FromQuat(const float (&q)[4])
	{
		const float x = q[0], y = q[1], z = q[2], w = q[3];
		M3          m;
		m.entry[0][0] = 1 - 2 * (y * y + z * z), m.entry[0][1] = 2 * (x * y - z * w), m.entry[0][2] = 2 * (x * z + y * w);
		m.entry[1][0] = 2 * (x * y + z * w), m.entry[1][1] = 1 - 2 * (x * x + z * z), m.entry[1][2] = 2 * (y * z - x * w);
		m.entry[2][0] = 2 * (x * z - y * w), m.entry[2][1] = 2 * (y * z + x * w), m.entry[2][2] = 1 - 2 * (x * x + y * y);
		return m;
	}

	inline void CollectPrev(RE::NiAVObject* a_node, std::vector<std::pair<RE::NiAVObject*, RE::NiTransform>>& a_out)
	{
		a_out.emplace_back(a_node, a_node->previousWorld);
		if (auto* n = a_node->AsNode()) {
			for (auto& c : n->GetChildren()) {
				if (c) {
					CollectPrev(c.get(), a_out);
				}
			}
		}
	}

	inline void ComposeWorld(RE::NiAVObject* a_node)
	{
		if (a_node->parent) {
			a_node->world = a_node->parent->world * a_node->local;
		}
		if (auto* n = a_node->AsNode()) {
			for (auto& c : n->GetChildren()) {
				if (c) {
					ComposeWorld(c.get());
				}
			}
		}
		a_node->UpdateWorldBound();
	}

	inline void Refresh(RE::NiAVObject* a_node) { ComposeWorld(a_node); }

	inline void RefreshXform(RE::NiAVObject* a_node) { Refresh(a_node); }

	inline void RotateWorld(RE::NiAVObject* a_bone, const M3& a_dR)
	{
		const M3& p = a_bone->parent->world.rotate;
		a_bone->local.rotate = p.Transpose() * a_dR * p * a_bone->local.rotate;
	}

	inline void MoveWorld(RE::NiAVObject* a_bone, const V3& a_pos)
	{
		const auto* p = a_bone->parent;
		const float s = p->world.scale > 0.0f ? p->world.scale : 1.0f;
		a_bone->local.translate = (p->world.rotate.Transpose() * (a_pos - p->world.translate)) * (1.0f / s);
	}

	inline void SolveTwoBone(RE::NiAVObject* a, RE::NiAVObject* b, RE::NiAVObject* c, const V3& a_target, const V3& a_fallbackPole)
	{
		const V3    A = a->world.translate, B = b->world.translate, C = c->world.translate;
		const float l1 = (B - A).Length(), l2 = (C - B).Length();
		const V3    AT = a_target - A;
		const float d = AT.Length();
		if (l1 < 1e-3f || l2 < 1e-3f || d < 1e-3f) {
			return;
		}
		const V3    dir = AT * (1.0f / d);
		const float dc = Clamp(d, std::fabs(l1 - l2) + 0.5f, l1 + l2 - 0.2f);
		const float along = (l1 * l1 - l2 * l2 + dc * dc) / (2.0f * dc);
		const float h = std::sqrt((std::max)(l1 * l1 - along * along, 0.0f));
		const V3    AB = B - A;
		const V3    pole = Norm(AB - dir * AB.Dot(dir), a_fallbackPole);
		const V3    newB = A + dir * along + pole * h;

		RotateWorld(a, RotBetween(Norm(B - A, dir), Norm(newB - A, dir)));
		Refresh(a);
		const V3 B2 = b->world.translate, C2 = c->world.translate;
		RotateWorld(b, RotBetween(Norm(C2 - B2, dir), Norm(A + dir * dc - B2, dir)));
		Refresh(b);
	}

	inline void SolveTwoBonePole(RE::NiAVObject* a, RE::NiAVObject* b, RE::NiAVObject* c, const V3& a_target, const V3& a_pole)
	{
		const V3    A = a->world.translate, B = b->world.translate, C = c->world.translate;
		const float l1 = (B - A).Length(), l2 = (C - B).Length();
		const V3    AT = a_target - A;
		const float d = AT.Length();
		if (l1 < 1e-3f || l2 < 1e-3f || d < 1e-3f) {
			return;
		}
		const V3    dir = AT * (1.0f / d);
		const float dc = Clamp(d, std::fabs(l1 - l2) + 0.5f, l1 + l2 - 0.2f);
		const float along = (l1 * l1 - l2 * l2 + dc * dc) / (2.0f * dc);
		const float h = std::sqrt((std::max)(l1 * l1 - along * along, 0.0f));
		const V3    pole = Norm(a_pole - dir * a_pole.Dot(dir), Norm((B - A) - dir * (B - A).Dot(dir), { 0, 0, 1 }));
		const V3    newB = A + dir * along + pole * h;

		RotateWorld(a, RotBetween(Norm(B - A, dir), Norm(newB - A, dir)));
		Refresh(a);
		const V3 B2 = b->world.translate, C2 = c->world.translate;
		RotateWorld(b, RotBetween(Norm(C2 - B2, dir), Norm(A + dir * dc - B2, dir)));
		Refresh(b);
	}
}
