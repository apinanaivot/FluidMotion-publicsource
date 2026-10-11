#pragma once

#include "AngleProbe.h"
#include "CornerLimit.h"

namespace PathProbe
{
	struct State
	{
		std::uintptr_t path = 0;
		std::uint32_t  points = 0;
		float          progress = 0.0f;
		std::uint32_t  state = 0;
		float          f158 = 0.0f;
		bool           hasRequest = false;
		float          rqSpeed = 0.0f;
		float          rqDist = 0.0f;
		float          rqEnd = 0.0f;
		std::uint8_t   rqFlag = 0;
		bool           hasTc = false;
		float          tcSpeed = 0.0f, tcDist = 0.0f, tcEnd = 0.0f, tcAcc = 0.0f, tcDec = 0.0f, tcNode = -1.0f;
		std::uint8_t   tcFlag = 0;
		std::int8_t    tcFix = 0;
		bool                  hasAc = false;
		AngleProbe::Sample    ac{};
		bool                  hasCl = false;
		CornerLimit::Sample   cl{};
		std::array<float, 32> raw{};
	};

	struct PathNode
	{
		RE::NiPoint3 pos;
		RE::NiPoint3 tangent;
		bool         action = false;
	};
	struct PathGeometry
	{
		static constexpr std::uint32_t kMaxNodes = 64;
		std::uint32_t                  count = 0;
		std::uint32_t                  total = 0;
		bool                           incomplete = false;
		std::array<PathNode, kMaxNodes> nodes{};
	};
	bool ReadPath(std::uintptr_t a_path, PathGeometry& a_out);

	struct LinkRequest
	{
		std::uintptr_t follower;
		float          dist;
		float          end;
		std::uint8_t   flag;
		std::uint64_t  ms;
	};
	std::optional<LinkRequest> GetRequestByLink(std::uintptr_t a_link);

	struct Translation
	{
		float        speed, dist, end, acc, dec;
		float        nodeDist;
		std::uint8_t flag;
		std::int8_t  fix;
	};
	inline std::atomic_bool recording{ false };
	void RecordTranslation(std::uintptr_t a_link, const Translation& a_t);
	void RecordAngle(std::uintptr_t a_link, const AngleProbe::Sample& a_s);
	void RecordCorner(std::uintptr_t a_follower, const CornerLimit::Sample& a_s);

	void Install();
	void Refresh(const std::vector<RE::Actor*>& a_actors);
	std::optional<State> Get(RE::Actor* a_actor);

	struct PathRef
	{
		std::uintptr_t path = 0;
		float          progress = 0.0f;
		std::uint32_t  state = 0;
	};
	std::optional<PathRef> GetPathRef(RE::Actor* a_actor);
	RE::FormID             ActorForKey(std::uintptr_t a_key);
}
