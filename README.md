# Fluid Motion: Pathfinding Overhaul

Download the built mod from Nexus Mods: https://www.nexusmods.com/skyrimspecialedition/mods/194680?tab=description

SKSE plugin for smoother NPC and creature movement in Skyrim Special Edition (version 0.1.1).

## Requirements
- SKSE64, Address Library for SKSE Plugins, SKSE Menu Framework

## Build
- Visual Studio 2022 (Desktop development with C++), xmake 3.x

```
git clone --recursive https://github.com/apinanaivot/FluidMotion-publicsource.git
xmake f -p windows -m releasedbg
xmake
```

The DLL is written under `build/`. Copy it with `dist/SKSE/Plugins/*` into `Data/SKSE/Plugins`.

## Motion matching
The motion matching code needs a
`FluidMotion_mm.bin` database built from the LaFAN1 motion capture dataset (Ubisoft), which is licensed
CC BY-NC-ND 4.0. Because of that licence the database and the tool that builds it are not part of this repository,
and the mode is off by default. Without the file the plugin skips it and everything else works as normal.

## Third-party
- `src/SKSEMenuFramework.h`: SKSE Menu Framework API header, MIT, from QTR-Modding/SKSE-Menu-Framework-3-Example.
- CommonLibSSE-NG (submodule).
- LaFAN1 (Ubisoft La Forge), CC BY-NC-ND 4.0: not included, see above.
