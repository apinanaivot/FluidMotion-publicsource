#pragma once

namespace CrashLog
{
	inline thread_local const char* crumb = "";
	inline void                     Crumb(const char* a_what) { crumb = a_what; }
	void                            Install();
}
