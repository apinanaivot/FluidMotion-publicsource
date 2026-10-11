#pragma once

namespace Config
{
	struct Telemetry
	{
		bool  enabled = true;
		int   sampleIntervalMs = 33;
		float radius = 10000.0f;
		bool  trackFleeing = true;
		bool  trackCombat = true;
		bool  trackRunning = true;
		float fastSpeed = 150.0f;
		float slowSpeed = 40.0f;
		float turnSegment = 64.0f;
		float sharpTurnDeg = 60.0f;
		float episodeGraceSec = 3.0f;
		float minEpisodeSec = 2.0f;
		bool  writeTraceCsv = true;
	};

	struct GameSettingsDump
	{
		bool                     enabled = true;
		std::vector<std::string> keywords;
	};

	struct Jump
	{
		bool  enabled = true;
		bool  shortcut = true;
		bool  pursuit = false;
		bool  follow = true;
		bool  route = true;
		float routeMaxDist = 2500.0f;
		float routeStuckSec = 1.0f;
		float routeStep = 90.0f;
		int   routeMaxNodes = 500;
		int   routeMaxNodesNear = 3000;
		int   routeNearCount = 3;
		bool  jumpOnNavmesh = false;
		float glideSpeed = 400.0f;
		float gravity = 900.0f;
		float upPerHeight = 1.15f;
		float upMin = 60.0f;
		float upMax = 180.0f;
		float downMax = 450.0f;
		float distPerHeight = 2.0f;
		float distMin = 150.0f;
		float distMax = 350.0f;
		float maxActorHeight = 300.0f;
		float minSaving = 300.0f;
		float minRatio = 1.6f;
		float minSpeed = 250.0f;
		float maxSpeed = 600.0f;
		float clearance = 12.0f;
		float cooldown = 1.5f;
		float followRouteMin = 500.0f;
		float turnRateDeg = 420.0f;
		float driveRise = 220.0f;
		float driveFall = 500.0f;
		bool  raceMotion = true;
		float driveTurnScale = 1.0f;
		float glideAccel = 700.0f;
		float pursuitStuckSec = 1.0f;
		float followStuckSec = 2.0f;
		float pursuitMinRise = 70.0f;
		float pursuitMinDrop = 120.0f;
		int   driveMode = 0;
		std::vector<std::string> excludeRaces;
		std::vector<std::string> noRouteRaces;
		std::vector<std::string> climbRaces;
		std::vector<std::string> steepRaces;
		float steepRise = 130.0f;
		float steepCost = 1.4f;
		float wadeDepth = 0.45f;
		float wadeCost = 1.5f;
		float vertMismatch = 1.0f;
		float giantHeight = 220.0f;
		float giantDistMax = 700.0f;
		float giantUpPerHeight = 1.4f;
		float giantDistPerHeight = 2.6f;
		std::string npcStartEvent = "JumpDirectionalStart";
		std::string npcLandEvent = "JumpLand";
		std::string creatureStartEvent;
		std::string creatureLandEvent;
		bool        log = true;
	};

	struct Anim
	{
		bool  enabled = true;
		bool  player = true;
		bool  playerMotion = true;
		bool  npcs = true;
		bool  jumpPose = true;
		float radius = 1800.0f;
		int   maxActors = 24;
		float castHz = 25.0f;
		int   maxRays = 90;
		bool  speedSync = true;
		bool  stepOver = true;
		bool  duck = true;
		bool  slopeLean = true;
		bool  turnLean = true;
		bool  headLead = true;
		bool  landCrouch = true;
		bool  stopSettle = true;
		bool  touchWalls = false;
		bool  climb = true;
		float climbMin = 38.0f;
		float climbMaxFrac = 0.8f;
		float climbApproach = 0.1f;
		float climbTail = 0.35f;
		float climbRate = 1.2f;
		bool  playerClimb = true;
		bool  hang = true;
		float hangTail = 0.55f;
		float stepMax = 40.0f;
		float footClear = 3.0f;
		float leadTime = 0.15f;
		float duckMargin = 14.0f;
		float duckMax = 40.0f;
		float bankGain = 0.4f;
		float bankMaxDeg = 14.0f;
		float headLeadSec = 0.35f;
		float headMaxDeg = 30.0f;
		float slopeMinDeg = 8.0f;
		float slopeGain = 0.65f;
		float slopeMaxDeg = 22.0f;
		float slopeBackGain = 1.2f;
		float slopeBackMaxDeg = 20.0f;
		float dipDecel = 180.0f;
		float dipRef = 500.0f;
		float dipDrop = 14.0f;
		float dipPitchDeg = 10.0f;
		float stopScale = 0.5f;
		float landMinFall = 110.0f;
		float landRef = 350.0f;
		bool  motionMatch = true;
		bool  mmArms = false;
		float mmSearchHz = 8.0f;
		float mmStopFade = 0.12f;
		float mmCadence = 0.5f;
		float mmClavicle = 0.0f;
		float mmForearm = 0.35f;
		float mmUpperArm = 0.5f;
		float mmHand = 0.5f;
		float mmArmLateral = 1.0f;
		float mmLeanCut = 0.7f;
		float headLevel = 0.85f;
		float neckMaxBendDeg = 15.0f;
		bool  vertSpring = true;
		float vertHalf = 0.25f;
		float vertMax = 80.0f;
		float camStiffDist = 40.0f;
		float mmHalflife = 0.12f;
		float mmMargin = 0.2f;
		bool  mmVanillaFwd = true;
		float mmVanillaBias = 0.5f;
		bool  playerAccelOn = true;
		float playerAccel = 0.15f;
		float playerTurnDeg = 300.0f;
		bool  playerHeading = true;
		bool  attackTurn = true;
		bool  firstPersonMotion = false;
		bool  combatFreeCamera = false;
		float mmRateMin = 0.7f;
		float mmRateMax = 1.5f;
		bool  log = true;
	};

	struct Debug
	{
		bool  showNavmesh = false;
		bool  showPaths = true;
		float radius = 2500.0f;
	};

	struct Spider
	{
		bool  enabled = true;
		int   maxSpiders = 6;
		float radius = 2500.0f;
		float stepDist = 20.0f;
		float stepDistMax = 32.0f;
		float stepDistSpeed = 0.07f;
		float overshoot = 0.6f;
		float stepHeight = 12.0f;
		float swingFrac = 0.45f;
		float swingFracSlow = 0.33f;
		float swingMin = 0.08f;
		float swingMax = 0.80f;
		float swingSlow = 30.0f;
		float sideStep = 0.5f;
		float footSpeed = 0.0f;
		float bodyTime = 0.10f;
		float maxTiltDeg = 25.0f;
		float fade = 0.15f;
		int   maxPerFrame = 6;
		bool  recommit = true;
		bool  omni = true;
		float maxTurnDeg = 75.0f;
		float sideSpeed = 0.6f;
		float backSpeed = 0.5f;
		std::vector<std::string> races;
		bool  log = true;
	};

	struct Climb
	{
		bool  enabled = true;
		float speed = 230.0f;
		float accel = 600.0f;
		float turnDeg = 90.0f;
		float engage = 2500.0f;
		float minRise = 80.0f;
		float stuckSec = 1.5f;
		float stopDist = 105.0f;
		float hover = 30.0f;
		float stand = 34.0f;
		float maxAngleDeg = 100.0f;
		float sideFrac = 0.6f;
		float backFrac = 0.5f;
		float sepRadius = 60.0f;
		float maxSeconds = 40.0f;
		float cooldown = 3.0f;
		bool  log = true;
	};

	struct Combat
	{
		bool  swing = true;
		float swingIdleSec = 0.35f;
		float swingCooldown = 1.2f;
		float swingReach = 0.9f;
		float swingRise = 0.75f;
		float swingAngleDeg = 30.0f;
		float swingReachUnarmed = 2.2f;
		bool  log = true;
	};

	struct Data
	{
		Debug                                             debug;
		Telemetry                                         telemetry;
		Jump                                              jump;
		Anim                                              anim;
		Combat                                            combat;
		Spider                                            spider;
		Climb                                             climb;
		GameSettingsDump                                  dump;
		std::vector<std::pair<std::string, std::string>>  gmstOverrides;
	};

	const Data& Get();
	Data&       Mutable();
	void        Load();
}
