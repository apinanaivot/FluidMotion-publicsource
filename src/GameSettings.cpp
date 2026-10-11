#include "GameSettings.h"
#include "Config.h"

namespace GameSettings
{
	namespace
	{
		using Type = RE::Setting::Type;

		std::unordered_map<RE::Setting*, RE::Setting::Data> g_originals;

		bool ContainsI(std::string_view a_hay, std::string_view a_needle)
		{
			auto it = std::search(a_hay.begin(), a_hay.end(), a_needle.begin(), a_needle.end(),
				[](char a, char b) { return std::tolower(static_cast<unsigned char>(a)) == std::tolower(static_cast<unsigned char>(b)); });
			return it != a_hay.end();
		}

		std::string ToString(const RE::Setting& a_s)
		{
			switch (a_s.GetType()) {
			case Type::kBool:
				return a_s.data.b ? "1" : "0";
			case Type::kInteger:
				return std::to_string(a_s.data.i);
			case Type::kUnsignedInteger:
				return std::to_string(a_s.data.u);
			case Type::kFloat:
				return std::format("{:.4f}", a_s.data.f);
			case Type::kString:
				{
					std::string str = a_s.data.s ? a_s.data.s : "";
					std::ranges::replace(str, '\n', ' ');
					std::ranges::replace(str, '\r', ' ');
					return std::format("\"{}\"", str);
				}
			default:
				return "<?>";
			}
		}

		bool Assign(RE::Setting& a_s, const std::string& a_raw)
		{
			try {
				switch (a_s.GetType()) {
				case Type::kBool:
					a_s.data.b = a_raw == "1" || a_raw == "true" || a_raw == "True";
					return true;
				case Type::kInteger:
					a_s.data.i = std::stoi(a_raw);
					return true;
				case Type::kUnsignedInteger:
					a_s.data.u = static_cast<std::uint32_t>(std::stoul(a_raw));
					return true;
				case Type::kFloat:
					a_s.data.f = std::stof(a_raw);
					return true;
				default:
					return false;
				}
			} catch (const std::exception&) {
				return false;
			}
		}
	}

	void DumpMatching()
	{
		const auto& cfg = Config::Get().dump;
		auto*       gmst = RE::GameSettingCollection::GetSingleton();
		if (!cfg.enabled || !gmst) {
			return;
		}

		std::vector<std::pair<std::string, std::string>> rows;
		for (auto& item : gmst->settings) {
			auto* setting = item.second;
			if (!setting || !setting->GetName()) {
				continue;
			}
			std::string_view name = setting->GetName();
			for (auto& kw : cfg.keywords) {
				if (ContainsI(name, kw)) {
					rows.emplace_back(name, ToString(*setting));
					break;
				}
			}
		}
		std::ranges::sort(rows);

		logger::info("---- GMST dump: {} settings match keywords ----", rows.size());
		for (auto& [name, value] : rows) {
			logger::info("  {} = {}", name, value);
		}
		logger::info("---- end GMST dump ----");
	}

	std::optional<float> GetVanillaFloat(const char* a_name)
	{
		auto* gmst = RE::GameSettingCollection::GetSingleton();
		auto* setting = gmst ? gmst->GetSetting(a_name) : nullptr;
		if (!setting || setting->GetType() != Type::kFloat) {
			return std::nullopt;
		}
		if (auto it = g_originals.find(setting); it != g_originals.end()) {
			return it->second.f;
		}
		return setting->data.f;
	}

	bool SetFloat(const char* a_name, float a_value)
	{
		return SetRaw(a_name, std::format("{}", a_value));
	}

	bool SetRaw(const std::string& a_name, const std::string& a_raw)
	{
		auto* gmst = RE::GameSettingCollection::GetSingleton();
		auto* setting = gmst ? gmst->GetSetting(a_name.c_str()) : nullptr;
		if (!setting) {
			logger::warn("GMST '{}' does not exist", a_name);
			return false;
		}
		auto before = ToString(*setting);
		auto backup = setting->data;
		if (!Assign(*setting, a_raw)) {
			logger::warn("GMST: could not set '{}' to '{}'", a_name, a_raw);
			return false;
		}
		g_originals.try_emplace(setting, backup);
		if (before != ToString(*setting)) {
			logger::info("GMST {} {} -> {}", a_name, before, ToString(*setting));
		}
		return true;
	}

	void RestoreAll()
	{
		for (auto& [setting, original] : g_originals) {
			setting->data = original;
		}
		if (!g_originals.empty()) {
			logger::info("GMST: restored {} settings to vanilla", g_originals.size());
		}
		g_originals.clear();
	}
}
