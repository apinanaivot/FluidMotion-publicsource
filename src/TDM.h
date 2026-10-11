#pragma once

namespace TDM
{
	struct IVTDM1
	{
		virtual unsigned long        GetTDMThreadId() const noexcept = 0;
		virtual bool                 GetDirectionalMovementState() const noexcept = 0;
		virtual bool                 GetTargetLockState() const noexcept = 0;
		virtual RE::ActorHandle      GetCurrentTarget() const noexcept = 0;
		virtual std::uint8_t         RequestDisableDirectionalMovement(SKSE::PluginHandle a_handle) noexcept = 0;
		virtual std::uint8_t         RequestDisableHeadtracking(SKSE::PluginHandle a_handle) noexcept = 0;
		virtual SKSE::PluginHandle   GetDisableDirectionalMovementOwner() const noexcept = 0;
		virtual SKSE::PluginHandle   GetDisableHeadtrackingOwner() const noexcept = 0;
		virtual std::uint8_t         ReleaseDisableDirectionalMovement(SKSE::PluginHandle a_handle) noexcept = 0;
		virtual std::uint8_t         ReleaseDisableHeadtracking(SKSE::PluginHandle a_handle) noexcept = 0;
	};

	inline IVTDM1* Api()
	{
		static IVTDM1* api = nullptr;
		static bool    tried = false;
		if (!tried) {
			tried = true;
			if (auto mod = GetModuleHandleA("TrueDirectionalMovement.dll")) {
				if (auto req = reinterpret_cast<void* (*)(std::uint8_t)>(GetProcAddress(mod, "RequestPluginAPI"))) {
					api = static_cast<IVTDM1*>(req(0));
				}
			}
			logger::info("TDM: True Directional Movement {}", api ? "found, its directional movement is replaced while this mod is active (target lock, mounts and swimming stay with it)" : "not loaded");
		}
		return api;
	}

	inline bool TargetLocked()
	{
		auto* api = Api();
		return api && api->GetTargetLockState();
	}

	inline void Hold(bool a_want)
	{
		static bool held = false, failLogged = false;
		auto*       api = Api();
		if (!api || a_want == held) {
			return;
		}
		const auto h = SKSE::GetPluginHandle();
		if (a_want) {
			const auto r = api->RequestDisableDirectionalMovement(h);
			held = r == 0 || r == 3;
			if (!held && !failLogged) {
				failLogged = true;
				logger::warn("TDM: directional movement disable refused (result {}), the two cameras may fight", r);
			}
		} else {
			api->ReleaseDisableDirectionalMovement(h);
			held = false;
		}
	}
}
