#pragma once

namespace GameSettings
{
	void DumpMatching();

	std::optional<float> GetVanillaFloat(const char* a_name);
	bool                 SetFloat(const char* a_name, float a_value);
	bool                 SetRaw(const std::string& a_name, const std::string& a_raw);
	void                 RestoreAll();
}
