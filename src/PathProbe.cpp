#include "PathProbe.h"

#include <xbyak/xbyak.h>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>

namespace PathProbe
{
	namespace
	{
		constexpr std::ptrdiff_t kUpdateThisOffset = 0x18;

		std::mutex                                   g_lock;
		std::unordered_set<std::uintptr_t>           g_seen;
		struct Link
		{
			std::uintptr_t base;
			std::ptrdiff_t off;
			std::uintptr_t key;
		};
		std::unordered_set<std::uintptr_t>      g_owner;
		std::unordered_map<RE::FormID, Link>    g_follower;
		std::unordered_map<std::uintptr_t, RE::FormID> g_keyActor;
		std::unordered_set<std::uintptr_t>           g_unmatched;

		struct Request
		{
			float        speed, dist, end;
			std::uint8_t flag;
		};
		constexpr std::ptrdiff_t                       kFollowStateOffset = 0x80;
		std::mutex                                     g_reqLock;
		std::unordered_map<std::uintptr_t, Request>    g_requests;
		std::unordered_map<std::uintptr_t, LinkRequest> g_byLink;
		std::unordered_map<std::uintptr_t, Translation> g_tc;
		std::unordered_map<std::uintptr_t, CornerLimit::Sample> g_cl;
		std::unordered_map<std::uintptr_t, AngleProbe::Sample>  g_ac;

		using StateUpdate_t = std::uint32_t (*)(std::uintptr_t, std::uintptr_t, std::uintptr_t);
		StateUpdate_t g_stateUpdate = nullptr;

		std::uint32_t StateUpdateHook(std::uintptr_t a_this, std::uintptr_t a_ctx, std::uintptr_t a_out)
		{
			const auto result = g_stateUpdate(a_this, a_ctx, a_out);
			if (result == 0 && a_out) {
				Request r{
					*reinterpret_cast<float*>(a_out + 0x10),
					*reinterpret_cast<float*>(a_out + 0x48),
					*reinterpret_cast<float*>(a_out + 0x4C),
					*reinterpret_cast<std::uint8_t*>(a_out + 0x50)
				};
				const auto base = a_this - kFollowStateOffset;
				const auto link = *reinterpret_cast<std::uintptr_t*>(base + 0x10);
				std::scoped_lock l(g_reqLock);
				if (recording.load(std::memory_order_relaxed)) {
					if (g_requests.size() > 4096) {
						g_requests.clear();
					}
					g_requests[base] = r;
				}
				if (link) {
					if (g_byLink.size() > 4096) {
						g_byLink.clear();
					}
					g_byLink[link] = LinkRequest{ base, r.dist, r.end, r.flag, GetTickCount64() };
				}
			}
			return result;
		}

		void __cdecl OnUpdate(std::uintptr_t a_this)
		{
			std::scoped_lock l(g_lock);
			g_seen.insert(a_this - kUpdateThisOffset);
		}

		struct Thunk : Xbyak::CodeGenerator
		{
			Thunk(std::uintptr_t a_original)
			{
				push(rcx);
				push(rdx);
				push(r8);
				push(r9);
				sub(rsp, 0x68);
				movdqu(ptr[rsp + 0x20], xmm0);
				movdqu(ptr[rsp + 0x30], xmm1);
				movdqu(ptr[rsp + 0x40], xmm2);
				movdqu(ptr[rsp + 0x50], xmm3);
				mov(rax, reinterpret_cast<std::uintptr_t>(&OnUpdate));
				call(rax);
				movdqu(xmm0, ptr[rsp + 0x20]);
				movdqu(xmm1, ptr[rsp + 0x30]);
				movdqu(xmm2, ptr[rsp + 0x40]);
				movdqu(xmm3, ptr[rsp + 0x50]);
				add(rsp, 0x68);
				pop(r9);
				pop(r8);
				pop(rdx);
				pop(rcx);
				mov(rax, a_original);
				jmp(rax);
			}
		};

		template <class T>
		bool SafeRead(std::uintptr_t a_addr, T& a_out)
		{
			__try {
				a_out = *reinterpret_cast<const T*>(a_addr);
				return true;
			} __except (EXCEPTION_EXECUTE_HANDLER) {
				return false;
			}
		}
	}

	void Install()
	{
		REL::Relocation<std::uintptr_t> vtbl{ RE::VTABLE_MovementAgentPathFollowerStandard[1] };
		const auto original = *reinterpret_cast<std::uintptr_t*>(vtbl.address() + 2 * sizeof(void*));
		static Thunk thunk{ original };
		thunk.ready();
		DWORD old = 0;
		VirtualProtect(const_cast<std::uint8_t*>(thunk.getCode()), thunk.getSize(), PAGE_EXECUTE_READ, &old);
		vtbl.write_vfunc(2, reinterpret_cast<std::uintptr_t>(thunk.getCode()));
		logger::info("PathProbe: hooked path follower update (original at image+{:#x})",
			original - REL::Module::get().base());

		REL::Relocation<std::uintptr_t> stateVtbl{ RE::VTABLE_PathFollowerStateFollowPathToParameter[0] };
		g_stateUpdate = reinterpret_cast<StateUpdate_t>(stateVtbl.write_vfunc(1, &StateUpdateHook));
		logger::info("PathProbe: hooked FollowPathToParameter update (original at image+{:#x})",
			reinterpret_cast<std::uintptr_t>(g_stateUpdate) - REL::Module::get().base());
	}

	std::optional<LinkRequest> GetRequestByLink(std::uintptr_t a_link)
	{
		std::scoped_lock l(g_reqLock);
		if (auto it = g_byLink.find(a_link); it != g_byLink.end()) {
			return it->second;
		}
		return std::nullopt;
	}

	void RecordTranslation(std::uintptr_t a_link, const Translation& a_t)
	{
		if (!recording.load(std::memory_order_relaxed)) {
			return;
		}
		std::scoped_lock l(g_reqLock);
		if (g_tc.size() > 4096) {
			g_tc.clear();
		}
		g_tc[a_link] = a_t;
	}

	void RecordAngle(std::uintptr_t a_link, const AngleProbe::Sample& a_s)
	{
		if (!recording.load(std::memory_order_relaxed)) {
			return;
		}
		std::scoped_lock l(g_reqLock);
		if (g_ac.size() > 4096) {
			g_ac.clear();
		}
		g_ac[a_link] = a_s;
	}

	void RecordCorner(std::uintptr_t a_follower, const CornerLimit::Sample& a_s)
	{
		if (!recording.load(std::memory_order_relaxed)) {
			return;
		}
		std::scoped_lock l(g_reqLock);
		if (g_cl.size() > 4096) {
			g_cl.clear();
		}
		g_cl[a_follower] = a_s;
	}

	void Refresh(const std::vector<RE::Actor*>& a_actors)
	{
		std::unordered_set<std::uintptr_t> seen;
		{
			std::scoped_lock l(g_lock);
			seen.swap(g_seen);
		}

		std::unordered_map<std::uintptr_t, RE::FormID> keys;
		for (auto* a : a_actors) {
			const auto id = a->GetFormID();
			keys[reinterpret_cast<std::uintptr_t>(a)] = id;
			keys[reinterpret_cast<std::uintptr_t>(a->AsActorState())] = id;
			if (auto* mc = a->GetActorRuntimeData().movementController.get()) {
				keys[reinterpret_cast<std::uintptr_t>(mc)] = id;
			}
		}

		static int calls = 0;
		if (++calls % 20 == 0) {
			g_unmatched.clear();
		}
		for (auto base : seen) {
			if (g_owner.contains(base) || g_unmatched.contains(base)) {
				continue;
			}
			bool found = false;
			for (std::ptrdiff_t off = 0; off < 0x200 && !found; off += 8) {
				std::uintptr_t v = 0;
				if (SafeRead(base + off, v)) {
					if (auto it = keys.find(v); it != keys.end()) {
						g_owner.insert(base);
						g_follower[it->second] = Link{ base, off, v };
						{
							std::scoped_lock l(g_lock);
							if (g_keyActor.size() > 4096) {
								g_keyActor.clear();
							}
							g_keyActor[v] = it->second;
						}
						found = true;
						static bool logged = false;
						if (!logged) {
							logged = true;
							logger::info("PathProbe: follower->actor link found at follower+{:#x}", off);
						}
					}
				}
			}
			if (!found && g_unmatched.size() < 4096) {
				g_unmatched.insert(base);
			}
		}
	}

	bool ReadPath(std::uintptr_t a_path, PathGeometry& a_out)
	{
		if (!a_path) {
			return false;
		}
		__try {
			const auto data = *reinterpret_cast<const std::uintptr_t*>(a_path + 0x90);
			const auto total = *reinterpret_cast<const std::uint32_t*>(a_path + 0xA0);
			if (!data || total == 0 || total > 4096) {
				return false;
			}
			a_out.total = total;
			a_out.count = (std::min)(total, PathGeometry::kMaxNodes);
			a_out.incomplete = *reinterpret_cast<const bool*>(a_path + 0xA8);
			for (std::uint32_t i = 0; i < a_out.count; ++i) {
				const auto node = data + i * 0x48;
				auto&      n = a_out.nodes[i];
				n.pos = *reinterpret_cast<const RE::NiPoint3*>(node);
				n.action = *reinterpret_cast<const std::uintptr_t*>(node + 0x30) != 0;
				n.tangent = *reinterpret_cast<const RE::NiPoint3*>(node + 0x38);
			}
			return true;
		} __except (EXCEPTION_EXECUTE_HANDLER) {
			return false;
		}
	}

	std::optional<State> Get(RE::Actor* a_actor)
	{
		auto it = g_follower.find(a_actor->GetFormID());
		if (it == g_follower.end()) {
			return std::nullopt;
		}
		const auto [base, off, key] = it->second;
		std::uintptr_t check = 0;
		if (!SafeRead(base + off, check) || check != key) {
			g_follower.erase(it);
			g_owner.erase(base);
			return std::nullopt;
		}
		State s;
		if (!SafeRead(base + 0x128, s.path)) {
			return std::nullopt;
		}
		if (s.path) {
			SafeRead(s.path + 0xA0, s.points);
		}
		SafeRead(base + 0x100, s.progress);
		SafeRead(base + 0x118, s.state);
		SafeRead(base + 0x158, s.f158);
		SafeRead(base + 0xE0, s.raw);
		{
			std::scoped_lock l(g_reqLock);
			if (auto r = g_requests.find(base); r != g_requests.end()) {
				s.hasRequest = true;
				s.rqSpeed = r->second.speed;
				s.rqDist = r->second.dist;
				s.rqEnd = r->second.end;
				s.rqFlag = r->second.flag;
				g_requests.erase(r);
			}
			if (auto c = g_cl.find(base); c != g_cl.end()) {
				s.hasCl = true;
				s.cl = c->second;
				g_cl.erase(c);
			}
			std::uintptr_t link = 0;
			if (SafeRead(base + 0x10, link) && link) {
				if (auto t = g_tc.find(link); t != g_tc.end()) {
					const auto& v = t->second;
					s.hasTc = true;
					s.tcSpeed = v.speed;
					s.tcDist = v.dist;
					s.tcEnd = v.end;
					s.tcAcc = v.acc;
					s.tcDec = v.dec;
					s.tcNode = v.nodeDist;
					s.tcFlag = v.flag;
					s.tcFix = v.fix;
					g_tc.erase(t);
				}
				if (auto a = g_ac.find(link); a != g_ac.end()) {
					s.hasAc = true;
					s.ac = a->second;
					g_ac.erase(a);
				}
			}
		}
		return s;
	}

	RE::FormID ActorForKey(std::uintptr_t a_key)
	{
		std::scoped_lock l(g_lock);
		const auto       it = g_keyActor.find(a_key);
		return it == g_keyActor.end() ? 0 : it->second;
	}

	std::optional<PathRef> GetPathRef(RE::Actor* a_actor)
	{
		auto it = g_follower.find(a_actor->GetFormID());
		if (it == g_follower.end()) {
			return std::nullopt;
		}
		const auto [base, off, key] = it->second;
		std::uintptr_t check = 0;
		PathRef        r;
		if (!SafeRead(base + off, check) || check != key || !SafeRead(base + 0x128, r.path) ||
			!SafeRead(base + 0x100, r.progress) || !SafeRead(base + 0x118, r.state)) {
			return std::nullopt;
		}
		return r;
	}
}
