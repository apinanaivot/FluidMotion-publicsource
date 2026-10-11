#include "Config.h"

namespace Config
{
	namespace
	{
		Data g_data;

		std::vector<std::string> SplitList(std::string_view a_list)
		{
			std::vector<std::string> out;
			std::string              cur;
			for (char c : a_list) {
				if (c == ',' || c == ';') {
					if (!cur.empty()) {
						out.push_back(std::move(cur));
					}
					cur.clear();
				} else if (!std::isspace(static_cast<unsigned char>(c))) {
					cur.push_back(c);
				}
			}
			if (!cur.empty()) {
				out.push_back(std::move(cur));
			}
			return out;
		}
	}

	const Data& Get() { return g_data; }

	Data& Mutable() { return g_data; }

	void Load()
	{
		constexpr auto path = L"Data/SKSE/Plugins/FluidMotion.ini";

		CSimpleIniA ini;
		ini.SetUnicode();
		if (ini.LoadFile(path) < 0) {
			logger::warn("FluidMotion.ini not found, using defaults");
		}

		Data d;
		auto& t = d.telemetry;
		t.enabled = ini.GetBoolValue("Telemetry", "bEnabled", t.enabled);
		t.sampleIntervalMs = static_cast<int>(ini.GetLongValue("Telemetry", "iSampleIntervalMs", t.sampleIntervalMs));
		t.radius = static_cast<float>(ini.GetDoubleValue("Telemetry", "fRadius", t.radius));
		t.trackFleeing = ini.GetBoolValue("Telemetry", "bTrackFleeing", t.trackFleeing);
		t.trackCombat = ini.GetBoolValue("Telemetry", "bTrackCombat", t.trackCombat);
		t.trackRunning = ini.GetBoolValue("Telemetry", "bTrackRunning", t.trackRunning);
		t.fastSpeed = static_cast<float>(ini.GetDoubleValue("Telemetry", "fFastSpeed", t.fastSpeed));
		t.slowSpeed = static_cast<float>(ini.GetDoubleValue("Telemetry", "fSlowSpeed", t.slowSpeed));
		t.turnSegment = static_cast<float>(ini.GetDoubleValue("Telemetry", "fTurnSegment", t.turnSegment));
		t.sharpTurnDeg = static_cast<float>(ini.GetDoubleValue("Telemetry", "fSharpTurnDeg", t.sharpTurnDeg));
		t.episodeGraceSec = static_cast<float>(ini.GetDoubleValue("Telemetry", "fEpisodeGraceSec", t.episodeGraceSec));
		t.minEpisodeSec = static_cast<float>(ini.GetDoubleValue("Telemetry", "fMinEpisodeSec", t.minEpisodeSec));
		t.writeTraceCsv = ini.GetBoolValue("Telemetry", "bWriteTraceCsv", t.writeTraceCsv);
		t.sampleIntervalMs = std::clamp(t.sampleIntervalMs, 10, 1000);

		auto& j = d.jump;
		auto  getf = [&](const char* a_key, float& a_v) {
			a_v = static_cast<float>(ini.GetDoubleValue("Jump", a_key, a_v));
		};
		j.enabled = ini.GetBoolValue("Jump", "bEnabled", j.enabled);
		j.shortcut = ini.GetBoolValue("Jump", "bShortcut", j.shortcut);
		j.pursuit = ini.GetBoolValue("Jump", "bPursuit", j.pursuit);
		j.follow = ini.GetBoolValue("Jump", "bFollow", j.follow);
		j.route = ini.GetBoolValue("Jump", "bRoute", j.route);
		getf("fRouteMaxDist", j.routeMaxDist);
		getf("fRouteStuckSec", j.routeStuckSec);
		getf("fRouteStep", j.routeStep);
		j.routeMaxNodes = static_cast<int>(ini.GetLongValue("Jump", "iRouteMaxNodes", j.routeMaxNodes));
		getf("fGlideSpeed", j.glideSpeed);
		getf("fDriveTurnScale", j.driveTurnScale);
		j.raceMotion = ini.GetBoolValue("Jump", "bRaceMotion", j.raceMotion);
		j.routeStep = std::clamp(j.routeStep, 40.0f, 200.0f);
		j.routeMaxNodesNear = static_cast<int>(ini.GetLongValue("Jump", "iRouteMaxNodesNear", j.routeMaxNodesNear));
		j.routeNearCount = static_cast<int>(ini.GetLongValue("Jump", "iRouteNearCount", j.routeNearCount));
		j.jumpOnNavmesh = ini.GetBoolValue("Jump", "bJumpOnNavmesh", j.jumpOnNavmesh);
		j.routeMaxNodes = std::clamp(j.routeMaxNodes, 8, 6000);
		j.routeMaxNodesNear = std::clamp(j.routeMaxNodesNear, j.routeMaxNodes, 8000);
		getf("fGravity", j.gravity);
		getf("fUpPerHeight", j.upPerHeight);
		getf("fUpMin", j.upMin);
		getf("fUpMax", j.upMax);
		getf("fDownMax", j.downMax);
		getf("fDistPerHeight", j.distPerHeight);
		getf("fDistMin", j.distMin);
		getf("fDistMax", j.distMax);
		getf("fMaxActorHeight", j.maxActorHeight);
		getf("fMinSaving", j.minSaving);
		getf("fMinRatio", j.minRatio);
		getf("fMinSpeed", j.minSpeed);
		getf("fMaxSpeed", j.maxSpeed);
		getf("fClearance", j.clearance);
		getf("fCooldown", j.cooldown);
		getf("fTurnRateDeg", j.turnRateDeg);
		getf("fFollowRouteMin", j.followRouteMin);
		getf("fGlideAccel", j.glideAccel);
		getf("fDriveRise", j.driveRise);
		getf("fDriveFall", j.driveFall);
		getf("fPursuitStuckSec", j.pursuitStuckSec);
		getf("fFollowStuckSec", j.followStuckSec);
		getf("fPursuitMinRise", j.pursuitMinRise);
		getf("fPursuitMinDrop", j.pursuitMinDrop);
		j.driveMode = static_cast<int>(ini.GetLongValue("Jump", "iDriveMode", j.driveMode));
		j.excludeRaces = SplitList(ini.GetValue("Jump", "sExcludeRaces",
			"Dragon,Alduin,Mammoth,Horker,Mudcrab,Slaughterfish,Centurion,Sphere,Wisp,Netch,Fish,Torchbug,Moth,Butterfly,Dragonfly,Hawk,Crow,Chicken,Cow,Goat"));
		j.noRouteRaces = SplitList(ini.GetValue("Jump", "sNoRouteRaces", "Dragon,Alduin,Slaughterfish,Fish,Wisp,Netch,Torchbug,Moth,Butterfly,Dragonfly,Hawk,Crow"));
		j.climbRaces = SplitList(ini.GetValue("Jump", "sClimbRaces", "Draugr,Falmer,Hagraven,Forsworn,Skeleton"));
		j.steepRaces = SplitList(ini.GetValue("Jump", "sSteepRaces", "Bear,Spider,Chaurus,Sabre,Goat,Troll"));
		getf("fSteepRise", j.steepRise);
		getf("fSteepCost", j.steepCost);
		getf("fWadeDepth", j.wadeDepth);
		getf("fWadeCost", j.wadeCost);
		getf("fVertMismatch", j.vertMismatch);
		getf("fGiantHeight", j.giantHeight);
		getf("fGiantDistMax", j.giantDistMax);
		getf("fGiantUpPerHeight", j.giantUpPerHeight);
		getf("fGiantDistPerHeight", j.giantDistPerHeight);
		j.npcStartEvent = ini.GetValue("Jump", "sNpcStartEvent", j.npcStartEvent.c_str());
		j.npcLandEvent = ini.GetValue("Jump", "sNpcLandEvent", j.npcLandEvent.c_str());
		j.creatureStartEvent = ini.GetValue("Jump", "sCreatureStartEvent", "");
		j.creatureLandEvent = ini.GetValue("Jump", "sCreatureLandEvent", "");
		j.log = ini.GetBoolValue("Jump", "bLog", j.log);
		j.gravity = std::clamp(j.gravity, 300.0f, 3000.0f);
		j.minSpeed = std::clamp(j.minSpeed, 50.0f, 1500.0f);
		j.maxSpeed = (std::max)(j.maxSpeed, j.minSpeed);

		auto& cb = d.combat;
		cb.swing = ini.GetBoolValue("Combat", "bSwing", cb.swing);
		cb.swingIdleSec = static_cast<float>(ini.GetDoubleValue("Combat", "fSwingIdleSec", cb.swingIdleSec));
		cb.swingCooldown = static_cast<float>(ini.GetDoubleValue("Combat", "fSwingCooldown", cb.swingCooldown));
		cb.swingReach = static_cast<float>(ini.GetDoubleValue("Combat", "fSwingReach", cb.swingReach));
		cb.swingReachUnarmed = static_cast<float>(ini.GetDoubleValue("Combat", "fSwingReachUnarmed", cb.swingReachUnarmed));
		cb.swingRise = static_cast<float>(ini.GetDoubleValue("Combat", "fSwingRise", cb.swingRise));
		cb.swingAngleDeg =static_cast<float>(ini.GetDoubleValue("Combat", "fSwingAngleDeg", cb.swingAngleDeg));
		cb.log = ini.GetBoolValue("Combat", "bLog", cb.log);

		{
			auto& sp = d.spider;
			auto  spf = [&](const char* a_key, float& a_v) { a_v = static_cast<float>(ini.GetDoubleValue("Spider", a_key, a_v)); };
			sp.enabled = ini.GetBoolValue("Spider", "bEnabled", sp.enabled);
			sp.maxSpiders = std::clamp(static_cast<int>(ini.GetLongValue("Spider", "iMaxSpiders", sp.maxSpiders)), 1, 30);
			sp.maxPerFrame = std::clamp(static_cast<int>(ini.GetLongValue("Spider", "iMaxStartsPerFrame", sp.maxPerFrame)), 1, 8);
			spf("fRadius", sp.radius);
			spf("fStepDist", sp.stepDist);
			spf("fStepDistSpeed", sp.stepDistSpeed);
			spf("fStepDistMax", sp.stepDistMax);
			spf("fOvershoot", sp.overshoot);
			spf("fStepHeight", sp.stepHeight);
			spf("fSwingFrac", sp.swingFrac);
			spf("fSwingFracSlow", sp.swingFracSlow);
			spf("fSwingMin", sp.swingMin);
			spf("fSwingMax", sp.swingMax);
			spf("fSwingSlow", sp.swingSlow);
			spf("fFootSpeed", sp.footSpeed);
			spf("fSideStep", sp.sideStep);
			spf("fBodyTime", sp.bodyTime);
			spf("fMaxTiltDeg", sp.maxTiltDeg);
			spf("fFade", sp.fade);
			sp.recommit = ini.GetBoolValue("Spider", "bRecommit", sp.recommit);
			sp.omni = ini.GetBoolValue("Spider", "bOmni", sp.omni);
			spf("fMaxTurnDeg", sp.maxTurnDeg);
			spf("fSideSpeed", sp.sideSpeed);
			spf("fBackSpeed", sp.backSpeed);
			sp.races = SplitList(ini.GetValue("Spider", "sRaces", "Spider"));
			sp.log = ini.GetBoolValue("Spider", "bLog", sp.log);
		}

		{
			auto& cl = d.climb;
			auto  clf = [&](const char* a_key, float& a_v) { a_v = static_cast<float>(ini.GetDoubleValue("SpiderClimb", a_key, a_v)); };
			cl.enabled = ini.GetBoolValue("SpiderClimb", "bEnabled", cl.enabled);
			clf("fSpeed", cl.speed);
			clf("fAccel", cl.accel);
			clf("fTurnDeg", cl.turnDeg);
			clf("fEngage", cl.engage);
			clf("fMinRise", cl.minRise);
			clf("fStuckSec", cl.stuckSec);
			clf("fStopDist", cl.stopDist);
			clf("fHover", cl.hover);
			clf("fStand", cl.stand);
			clf("fMaxAngleDeg", cl.maxAngleDeg);
			clf("fSideFrac", cl.sideFrac);
			clf("fBackFrac", cl.backFrac);
			clf("fSepRadius", cl.sepRadius);
			clf("fMaxSeconds", cl.maxSeconds);
			clf("fCooldown", cl.cooldown);
			cl.log = ini.GetBoolValue("SpiderClimb", "bLog", cl.log);
		}

		auto& an = d.anim;
		auto  anf = [&](const char* a_key, float& a_v) {
			a_v = static_cast<float>(ini.GetDoubleValue("Anim", a_key, a_v));
		};
		auto anb = [&](const char* a_key, bool& a_v) { a_v = ini.GetBoolValue("Anim", a_key, a_v); };
		anb("bEnabled", an.enabled);
		anb("bPlayer", an.player);
		anb("bPlayerMotion", an.playerMotion);
		anb("bNpcs", an.npcs);
		anf("fRadius", an.radius);
		an.maxActors = std::clamp(static_cast<int>(ini.GetLongValue("Anim", "iMaxActors", an.maxActors)), 1, 100);
		anf("fCastHz", an.castHz);
		an.maxRays = std::clamp(static_cast<int>(ini.GetLongValue("Anim", "iMaxRays", an.maxRays)), 5, 500);
		anb("bSpeedSync", an.speedSync);
		anb("bStepOver", an.stepOver);
		anb("bDuck", an.duck);
		anb("bSlopeLean", an.slopeLean);
		anb("bTurnLean", an.turnLean);
		anb("bHeadLead", an.headLead);
		anb("bLandCrouch", an.landCrouch);
		anb("bStopSettle", an.stopSettle);
		anb("bTouchWalls", an.touchWalls);
		anb("bClimb", an.climb);
		anb("bJumpPose", an.jumpPose);
		anf("fClimbMin", an.climbMin);
		anf("fClimbMaxFrac", an.climbMaxFrac);
		anf("fClimbApproach", an.climbApproach);
		anf("fClimbTail", an.climbTail);
		anf("fClimbRate", an.climbRate);
		anb("bHang", an.hang);
		anb("bPlayerClimb", an.playerClimb);
		anf("fHangTail", an.hangTail);
		anf("fStepMax", an.stepMax);
		anf("fFootClear", an.footClear);
		anf("fLeadTime", an.leadTime);
		anf("fDuckMargin", an.duckMargin);
		anf("fDuckMax", an.duckMax);
		anf("fBankGain", an.bankGain);
		anf("fBankMaxDeg", an.bankMaxDeg);
		anf("fHeadLeadSec", an.headLeadSec);
		anf("fHeadMaxDeg", an.headMaxDeg);
		anf("fSlopeMinDeg", an.slopeMinDeg);
		anf("fSlopeGain", an.slopeGain);
		anf("fSlopeMaxDeg", an.slopeMaxDeg);
		anf("fSlopeBackGain", an.slopeBackGain);
		anf("fSlopeBackMaxDeg", an.slopeBackMaxDeg);
		anf("fDipDecel", an.dipDecel);
		anf("fDipRef", an.dipRef);
		anf("fDipDrop", an.dipDrop);
		anf("fDipPitchDeg", an.dipPitchDeg);
		anf("fStopScale", an.stopScale);
		anf("fLandMinFall", an.landMinFall);
		anf("fLandRef", an.landRef);
		anb("bMotionMatch", an.motionMatch);
		anb("bMMArms", an.mmArms);
		anf("fMMSearchHz", an.mmSearchHz);
		anf("fMMHalflife", an.mmHalflife);
		anf("fMMLeanCut", an.mmLeanCut);
		anf("fMMClavicle", an.mmClavicle);
		anf("fMMForearm", an.mmForearm);
		anf("fMMHand", an.mmHand);
		anf("fMMUpperArm", an.mmUpperArm);
		anf("fMMArmLateral", an.mmArmLateral);
		anf("fMMStopFade", an.mmStopFade);
		anf("fMMCadence", an.mmCadence);
		anf("fHeadLevel", an.headLevel);
		anf("fNeckMaxBendDeg", an.neckMaxBendDeg);
		anb("bVertSpring", an.vertSpring);
		anf("fVertHalf", an.vertHalf);
		anf("fVertMax", an.vertMax);
		anf("fCamStiffDist", an.camStiffDist);
		anf("fMMMargin", an.mmMargin);
		anb("bMMVanillaForward", an.mmVanillaFwd);
		anf("fMMVanillaBias", an.mmVanillaBias);
		anb("bPlayerAccel", an.playerAccelOn);
		anf("fPlayerAccel", an.playerAccel);
		anf("fPlayerTurnDeg", an.playerTurnDeg);
		anb("bPlayerHeading", an.playerHeading);
		anb("bCombatFreeCamera", an.combatFreeCamera);
		anb("bAttackTurn", an.attackTurn);
		anb("bFirstPersonMotion", an.firstPersonMotion);
		anf("fMMRateMin", an.mmRateMin);
		anf("fMMRateMax", an.mmRateMax);
		anb("bLog", an.log);
		an.castHz = std::clamp(an.castHz, 5.0f, 120.0f);

		d.debug.showNavmesh = ini.GetBoolValue("Debug", "bShowNavmesh", d.debug.showNavmesh);
		d.debug.showPaths = ini.GetBoolValue("Debug", "bShowPaths", d.debug.showPaths);
		d.debug.radius = std::clamp(static_cast<float>(ini.GetDoubleValue("Debug", "fRadius", d.debug.radius)), 500.0f, 8000.0f);

		d.dump.enabled = ini.GetBoolValue("GameSettingsDump", "bEnabled", true);
		d.dump.keywords = SplitList(ini.GetValue("GameSettingsDump", "sKeywords",
			"Flee,Path,Avoid,Turn,Move,Run,Walk,Steer,Navmesh,Pursue,Detour,Stuck,CombatAttack,CombatAim,CombatAngle,CombatBetween,CombatApproachTarget,CombatForward,CombatStrafe,CombatStep,CombatMelee"));

		CSimpleIniA::TNamesDepend keys;
		ini.GetAllKeys("GameSettings", keys);
		keys.sort(CSimpleIniA::Entry::LoadOrder());
		for (auto& key : keys) {
			if (auto value = ini.GetValue("GameSettings", key.pItem)) {
				d.gmstOverrides.emplace_back(key.pItem, value);
			}
		}

		g_data = std::move(d);
		logger::info("Config loaded ({} extra GMST overrides)", g_data.gmstOverrides.size());
	}
}
