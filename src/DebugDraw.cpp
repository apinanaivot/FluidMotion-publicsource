#include "DebugDraw.h"
#include "JumpNav.h"
#include "PathProbe.h"

#include "SKSEMenuFramework.h"

namespace DebugDraw
{
	namespace
	{
		using Vec = RE::NiPoint3;

		struct Line
		{
			Vec           a, b;
			std::uint32_t color;
			float         thick;
		};
		struct Text
		{
			Vec           p;
			std::uint32_t color;
			std::string   s;
		};
		struct Scene
		{
			std::vector<Line>        lines;
			std::vector<Text>        texts;
			std::vector<std::string> hud;
		};

		constexpr std::uint32_t Rgba(int r, int g, int b, int a = 255) { return IM_COL32(r, g, b, a); }

		const std::uint32_t kInterior = Rgba(80, 200, 220, 70);
		const std::uint32_t kBoundary = Rgba(255, 150, 40, 200);
		const std::uint32_t kPortal = Rgba(90, 120, 255, 150);
		const std::uint32_t kLedgeDown = Rgba(255, 40, 40, 255);
		const std::uint32_t kLedgeUp = Rgba(255, 60, 255, 255);

		std::mutex                               g_lock;
		std::shared_ptr<const std::vector<Line>> g_navLines;
		Scene                                    g_scene;
		std::string                              g_navSummary;

		double                                   g_time = 0;
		std::chrono::steady_clock::time_point    g_last = std::chrono::steady_clock::now();
		double                                   g_nextNav = 0;
		double                                   g_nextRefresh = 0;
		std::unordered_set<RE::FormID>           g_loggedMeshes;

		float Dist2D(const Vec& a, const Vec& b) { return std::hypot(a.x - b.x, a.y - b.y); }

		struct NavCounts
		{
			int meshes = 0, tris = 0, boundary = 0, portals = 0, ledgeUp = 0, ledgeDown = 0, other = 0;
		};

		Vec Centroid(const RE::BSNavmesh& a_mesh, std::uint16_t a_tri, bool& a_ok)
		{
			a_ok = false;
			if (a_tri >= a_mesh.triangles.size()) {
				return {};
			}
			const auto& t = a_mesh.triangles[a_tri];
			Vec         c{};
			for (auto v : t.vertices) {
				if (v >= a_mesh.vertices.size()) {
					return {};
				}
				c += a_mesh.vertices[v].location;
			}
			a_ok = true;
			return c / 3.0f;
		}

		void AddMesh(RE::NavMesh* a_mesh, const Vec& a_center, float a_radius, std::vector<Line>& a_out, NavCounts& a_counts, NavCounts& a_new)
		{
			using TF = RE::BSNavmeshTriangle::TriangleFlag;
			using ET = RE::EDGE_EXTRA_INFO_TYPE;
			constexpr TF kLink[3] = { TF::kEdge0_Link, TF::kEdge1_Link, TF::kEdge2_Link };

			const auto& verts = a_mesh->vertices;
			const auto& tris = a_mesh->triangles;
			const auto& extra = a_mesh->extraEdgeInfo;
			const bool  first = g_loggedMeshes.insert(a_mesh->GetFormID()).second;
			NavCounts   c;
			c.meshes = 1;

			for (std::uint32_t ti = 0; ti < tris.size(); ++ti) {
				const auto& t = tris[ti];
				if (t.triangleFlags.all(TF::kDeleted)) {
					continue;
				}
				++c.tris;
				for (int e = 0; e < 3; ++e) {
					const auto i0 = t.vertices[e], i1 = t.vertices[(e + 1) % 3];
					if (i0 >= verts.size() || i1 >= verts.size()) {
						continue;
					}
					const Vec&  va = verts[i0].location;
					const Vec&  vb = verts[i1].location;
					const bool  inRange = Dist2D((va + vb) * 0.5f, a_center) <= a_radius;
					const auto  nb = t.triangles[e];
					if (t.triangleFlags.all(kLink[e])) {
						const auto* info = nb < extra.size() ? &extra[nb] : nullptr;
						const auto  type = info ? static_cast<ET>(info->type.underlying()) : ET::kInvalid;
						const bool  ledge = type == ET::kLedgeUp || type == ET::kLedgeDown;
						if (type == ET::kPortal) {
							++c.portals;
						} else if (type == ET::kLedgeUp) {
							++c.ledgeUp;
						} else if (type == ET::kLedgeDown) {
							++c.ledgeDown;
						} else {
							++c.other;
						}
						if (!inRange) {
							continue;
						}
						if (ledge) {
							const auto col = type == ET::kLedgeUp ? kLedgeUp : kLedgeDown;
							a_out.push_back({ va, vb, col, 3.0f });
							auto* other = info->portal.otherMeshID ? RE::TESForm::LookupByID<RE::NavMesh>(info->portal.otherMeshID) : a_mesh;
							bool  ok = false;
							if (other) {
								const Vec target = Centroid(*other, info->portal.triangle, ok);
								if (ok) {
									a_out.push_back({ (va + vb) * 0.5f, target, col, 2.0f });
								}
							}
						} else {
							a_out.push_back({ va, vb, kPortal, 1.0f });
						}
					} else if (nb == 0xFFFF) {
						++c.boundary;
						if (inRange) {
							a_out.push_back({ va, vb, kBoundary, 1.5f });
						}
					} else if (nb > ti && inRange) {
						a_out.push_back({ va, vb, kInterior, 1.0f });
					}
				}
			}
			auto add = [&](NavCounts& a) {
				a.meshes += c.meshes;
				a.tris += c.tris;
				a.boundary += c.boundary;
				a.portals += c.portals;
				a.ledgeUp += c.ledgeUp;
				a.ledgeDown += c.ledgeDown;
				a.other += c.other;
			};
			add(a_counts);
			if (first) {
				add(a_new);
			}
		}

		void RebuildNav(const Vec& a_center)
		{
			auto      lines = std::make_shared<std::vector<Line>>();
			NavCounts all, fresh;
			std::unordered_set<RE::NavMesh*> seen;
			if (auto* tes = RE::TES::GetSingleton()) {
				tes->ForEachCell([&](RE::TESObjectCELL* a_cell) {
					auto* arr = a_cell ? a_cell->GetRuntimeData().navMeshes : nullptr;
					if (!arr) {
						return;
					}
					for (auto& nm : arr->navMeshes) {
						if (auto* m = nm.get(); m && seen.insert(m).second) {
							AddMesh(m, a_center, radius, *lines, all, fresh);
						}
					}
				});
			}
			if (fresh.meshes > 0) {
				logger::info("NAVMESH new meshes={} tris={} boundaryEdges={} portals={} ledgeUp={} ledgeDown={} otherLinks={}",
					fresh.meshes, fresh.tris, fresh.boundary, fresh.portals, fresh.ledgeUp, fresh.ledgeDown, fresh.other);
			}
			auto summary = std::format("navmesh: {} meshes, {} tris, ledge links up {} / down {}, {} lines drawn",
				all.meshes, all.tris, all.ledgeUp, all.ledgeDown, lines->size());
			std::scoped_lock l(g_lock);
			g_navLines = std::move(lines);
			g_navSummary = std::move(summary);
		}

		void AddCross(Scene& a_s, const Vec& a_p, float a_size, std::uint32_t a_col, float a_thick)
		{
			a_s.lines.push_back({ a_p - Vec{ a_size, 0, 0 }, a_p + Vec{ a_size, 0, 0 }, a_col, a_thick });
			a_s.lines.push_back({ a_p - Vec{ 0, a_size, 0 }, a_p + Vec{ 0, a_size, 0 }, a_col, a_thick });
			a_s.lines.push_back({ a_p, a_p + Vec{ 0, 0, a_size * 2 }, a_col, a_thick });
		}

		void CapturePaths(Scene& a_s, RE::PlayerCharacter* a_player)
		{
			auto* lists = RE::ProcessLists::GetSingleton();
			if (!lists) {
				return;
			}
			std::vector<RE::Actor*> actors;
			for (auto& h : lists->highActorHandles) {
				if (auto a = h.get().get(); a && a != a_player) {
					actors.push_back(a);
				}
			}
			if (g_time >= g_nextRefresh) {
				g_nextRefresh = g_time + 0.25;
				PathProbe::Refresh(actors);
			}
			const Vec  ppos = a_player->GetPosition();
			static PathProbe::PathGeometry geo;
			for (auto* a : actors) {
				if (a->IsDead() || !a->Is3DLoaded() || Dist2D(a->GetPosition(), ppos) > radius * 1.5f) {
					continue;
				}
				auto       pr = PathProbe::GetPathRef(a);
				if (!pr || !pr->path || !PathProbe::ReadPath(pr->path, geo) || geo.count < 2) {
					continue;
				}
				const auto col = Rgba(255, 230, 80, 130);
				const float th = 1.5f;
				const int   from = std::clamp(static_cast<int>(pr->progress), 0, static_cast<int>(geo.count) - 1);
				Vec         prev = a->GetPosition();
				for (int i = from + 1; i < static_cast<int>(geo.count); ++i) {
					const auto& n = geo.nodes[i];
					a_s.lines.push_back({ prev, n.pos, col, th });
					AddCross(a_s, n.pos, n.action ? 25.0f : 10.0f, n.action ? Rgba(80, 255, 255) : col, 1.0f);
					prev = n.pos;
				}
			}

			std::vector<JumpNav::DebugArc> arcs;
			JumpNav::GetDebugArcs(arcs);
			for (auto& arc : arcs) {
				const float fade = arc.launched ? 1.0f - arc.age / 15.0f : 1.0f - arc.age / 4.0f;
				const int   alpha = std::clamp(static_cast<int>(255 * fade), 40, 255);
				const auto  col = arc.launched ? Rgba(60, 255, 90, alpha) : Rgba(255, 60, 60, alpha);
				for (std::size_t i = 1; i < arc.points.size(); ++i) {
					a_s.lines.push_back({ arc.points[i - 1], arc.points[i], col, arc.launched ? 3.0f : 1.5f });
				}
				if (!arc.points.empty()) {
					a_s.texts.push_back({ arc.points[arc.points.size() / 2] + Vec{ 0, 0, 20 }, col, arc.label });
				}
			}
		}

		bool Project(const float (&a_m)[4][4], const RE::NiRect<float>& a_port, const Vec& a_p, const ImGuiMCP::ImVec2& a_size, ImGuiMCP::ImVec2& a_out)
		{
			float x = 0, y = 0, z = 0;
			if (!RE::NiCamera::WorldPtToScreenPt3(a_m, a_port, a_p, x, y, z, 1e-5f) || z < 0.0f) {
				return false;
			}
			a_out = { x * a_size.x, (1.0f - y) * a_size.y };
			return true;
		}
	}

	bool Active() { return showNavmesh || showPaths; }

	void Update()
	{
		const auto now = std::chrono::steady_clock::now();
		g_time += std::chrono::duration<double>(now - g_last).count();
		g_last = now;

		auto* player = RE::PlayerCharacter::GetSingleton();
		if (!Active() || !player || !player->Is3DLoaded()) {
			std::scoped_lock l(g_lock);
			g_scene = {};
			g_navLines.reset();
			return;
		}
		if (showNavmesh && g_time >= g_nextNav) {
			g_nextNav = g_time + 0.5;
			RebuildNav(player->GetPosition());
		}

		Scene s;
		if (showPaths) {
			CapturePaths(s, player);
		}
		{
			std::scoped_lock l(g_lock);
			if (showNavmesh && !g_navSummary.empty()) {
				s.hud.push_back(g_navSummary);
			}
			if (!showNavmesh) {
				g_navLines.reset();
			}
		}
		std::scoped_lock l(g_lock);
		g_scene = std::move(s);
	}

	void __stdcall Draw()
	{
		if (!Active()) {
			return;
		}
		auto* cam = RE::Main::WorldRootCamera();
		if (!cam) {
			return;
		}
		std::shared_ptr<const std::vector<Line>> nav;
		Scene                                    scene;
		{
			std::scoped_lock l(g_lock);
			nav = g_navLines;
			scene = g_scene;
		}
		const auto& m = cam->GetRuntimeData().worldToCam;
		const auto& port = cam->GetRuntimeData2().port;
		const auto  size = ImGuiMCP::GetIO()->DisplaySize;
		auto*       dl = ImGuiMCP::GetBackgroundDrawList();

		auto drawLines = [&](const std::vector<Line>& a_lines) {
			for (auto& ln : a_lines) {
				ImGuiMCP::ImVec2 a, b;
				if (Project(m, port, ln.a, size, a) && Project(m, port, ln.b, size, b)) {
					ImGuiMCP::ImDrawListManager::AddLine(dl, a, b, ln.color, ln.thick);
				}
			}
		};
		if (nav) {
			drawLines(*nav);
		}
		drawLines(scene.lines);
		for (auto& t : scene.texts) {
			ImGuiMCP::ImVec2 p;
			if (Project(m, port, t.p, size, p)) {
				ImGuiMCP::ImDrawListManager::AddText(dl, { p.x + 1, p.y + 1 }, IM_COL32(0, 0, 0, 200), t.s.c_str());
				ImGuiMCP::ImDrawListManager::AddText(dl, p, t.color, t.s.c_str());
			}
		}
		float y = 10.0f;
		for (auto& h : scene.hud) {
			ImGuiMCP::ImDrawListManager::AddText(dl, { 11, y + 1 }, IM_COL32(0, 0, 0, 220), h.c_str());
			ImGuiMCP::ImDrawListManager::AddText(dl, { 10, y }, IM_COL32(255, 255, 255, 255), h.c_str());
			y += ImGuiMCP::GetTextLineHeightWithSpacing();
		}
	}
}
