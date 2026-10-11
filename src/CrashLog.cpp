#include "CrashLog.h"

#include <Windows.h>

namespace CrashLog
{
	namespace
	{
		std::string Where(const void* a_addr)
		{
			HMODULE h = nullptr;
			if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
					static_cast<LPCSTR>(a_addr), &h) || !h) {
				return std::format("{:p}", a_addr);
			}
			char path[MAX_PATH]{};
			GetModuleFileNameA(h, path, MAX_PATH);
			const char* name = std::strrchr(path, '\\');
			return std::format("{}+0x{:X}", name ? name + 1 : path, reinterpret_cast<std::uintptr_t>(a_addr) - reinterpret_cast<std::uintptr_t>(h));
		}

		LONG CALLBACK Handler(EXCEPTION_POINTERS* a_info)
		{
			static std::atomic_int count{ 0 };
			const auto*            rec = a_info->ExceptionRecord;
			if (rec->ExceptionCode != EXCEPTION_ACCESS_VIOLATION) {
				return EXCEPTION_CONTINUE_SEARCH;
			}
			static std::atomic<void*> seen[32]{};
			for (auto& s : seen) {
				void* cur = s.load();
				if (cur == rec->ExceptionAddress) {
					return EXCEPTION_CONTINUE_SEARCH;
				}
				if (!cur && s.compare_exchange_strong(cur, rec->ExceptionAddress)) {
					break;
				}
			}
			if (count.fetch_add(1) >= 32) {
				return EXCEPTION_CONTINUE_SEARCH;
			}
			const auto* c = a_info->ContextRecord;
			logger::error("CRASHLOG access violation at {} (thread {}), accessing {:p} ({}), last step '{}'", Where(rec->ExceptionAddress),
				GetCurrentThreadId(), reinterpret_cast<void*>(rec->ExceptionInformation[1]), rec->ExceptionInformation[0] ? "write" : "read", crumb);
			logger::error("CRASHLOG rax={:X} rbx={:X} rcx={:X} rdx={:X} rsi={:X} rdi={:X} r8={:X} r9={:X} rsp={:X}", c->Rax, c->Rbx, c->Rcx, c->Rdx,
				c->Rsi, c->Rdi, c->R8, c->R9, c->Rsp);
			const auto* sp = reinterpret_cast<const std::uintptr_t*>(c->Rsp);
			int         shown = 0;
			for (int i = 0; i < 400 && shown < 24; ++i) {
				std::uintptr_t v = 0;
				if (!ReadProcessMemory(GetCurrentProcess(), sp + i, &v, sizeof(v), nullptr)) {
					break;
				}
				MEMORY_BASIC_INFORMATION mbi{};
				if (v > 0x10000 && VirtualQuery(reinterpret_cast<void*>(v), &mbi, sizeof(mbi)) && mbi.Type == MEM_IMAGE &&
					(mbi.Protect & (PAGE_EXECUTE | PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY))) {
					logger::error("CRASHLOG   stack+{:X}: {}", i * 8, Where(reinterpret_cast<void*>(v)));
					++shown;
				}
			}
			return EXCEPTION_CONTINUE_SEARCH;
		}
	}

	void Install() { AddVectoredExceptionHandler(1, Handler); }
}
