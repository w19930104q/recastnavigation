#include <cstring>

#include "DetourCommon.h"
#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "catch2/catch_amalgamated.hpp"

namespace
{

const unsigned short POLYFLAGS_GROUND = 1 << 0;
const unsigned short POLYFLAGS_WATER = 1 << 1;
const unsigned short POLYFLAGS_ALL = POLYFLAGS_GROUND | POLYFLAGS_WATER;

const unsigned char AREA_GROUND = 1;
const unsigned char AREA_WATER = 2;

// Builds a single-tile navmesh made of a 5x3 grid of 2x2 quads:
//
//   z
//   6 +--------+--------+--------+--------+--------+
//     | ground | water  | water  | water  | ground |
//   4 +--------+--------+--------+--------+--------+
//     | ground | ground (bridge)          | ground |
//   2 +--------+--------+--------+--------+--------+
//     | ground | water  | water  | water  | ground |
//   0 +--------+--------+--------+--------+--------+
//     0        2        4        6        8        10   x
//
// Ground units must detour through the middle-row bridge, while units for
// which water is walkable can cross the water rows directly.
struct TestNavmesh
{
	static const int COLS = 5;
	static const int ROWS = 3;
	static const float CELL; // world size of one quad

	dtNavMesh* nav = nullptr;

	TestNavmesh()
	{
		const int nvp = 6; // DT_VERTS_PER_POLYGON

		unsigned short verts[(COLS + 1) * (ROWS + 1) * 3];
		for (int x = 0; x <= COLS; ++x)
		{
			for (int z = 0; z <= ROWS; ++z)
			{
				const int i = vertIndex(x, z) * 3;
				verts[i + 0] = (unsigned short)(x * CELL);
				verts[i + 1] = 0;
				verts[i + 2] = (unsigned short)(z * CELL);
			}
		}

		const int polyCount = COLS * ROWS;
		unsigned short polys[polyCount * 2 * nvp];
		unsigned short polyFlags[polyCount];
		unsigned char polyAreas[polyCount];
		for (int cx = 0; cx < COLS; ++cx)
		{
			for (int cz = 0; cz < ROWS; ++cz)
			{
				const int p = cx * ROWS + cz;
				unsigned short* poly = &polys[p * nvp * 2];
				// Counter-clockwise around +y, consistent winding so shared edges match.
				poly[0] = (unsigned short)vertIndex(cx, cz);
				poly[1] = (unsigned short)vertIndex(cx, cz + 1);
				poly[2] = (unsigned short)vertIndex(cx + 1, cz + 1);
				poly[3] = (unsigned short)vertIndex(cx + 1, cz);
				for (int j = 4; j < nvp * 2; ++j)
					poly[j] = 0xffff;

				const bool water = (cx >= 1 && cx <= 3) && (cz != 1);
				polyFlags[p] = water ? POLYFLAGS_WATER : POLYFLAGS_GROUND;
				polyAreas[p] = water ? AREA_WATER : AREA_GROUND;
			}
		}

		// Fill the adjacency half of the poly data: neighbor poly index (0-based) per
		// edge, 0xffff for border edges. This is what rcBuildPolyMesh produces and what
		// dtCreateNavMeshData turns into inter-poly links.
		for (int p = 0; p < polyCount; ++p)
		{
			unsigned short* poly = &polys[p * nvp * 2];
			for (int j = 0; j < 4; ++j)
			{
				const unsigned short v0 = poly[j];
				const unsigned short v1 = poly[(j + 1) % 4];
				unsigned short nei = 0xffff;
				for (int q = 0; q < polyCount && nei == 0xffff; ++q)
				{
					if (q == p)
						continue;
					const unsigned short* other = &polys[q * nvp * 2];
					for (int k = 0; k < 4; ++k)
					{
						if (other[k] == v1 && other[(k + 1) % 4] == v0)
						{
							nei = (unsigned short)q;
							break;
						}
					}
				}
				poly[nvp + j] = nei;
			}
		}

		dtNavMeshCreateParams params;
		std::memset(&params, 0, sizeof(params));
		params.verts = verts;
		params.vertCount = (COLS + 1) * (ROWS + 1);
		params.polys = polys;
		params.polyFlags = polyFlags;
		params.polyAreas = polyAreas;
		params.polyCount = polyCount;
		params.nvp = nvp;
		params.walkableHeight = 2.0f;
		params.walkableRadius = 1.0f;
		params.walkableClimb = 0.5f;
		params.cs = 1.0f;
		params.ch = 1.0f;
		dtVset(params.bmin, 0.0f, 0.0f, 0.0f);
		dtVset(params.bmax, COLS * CELL, 2.0f, ROWS * CELL);
		params.buildBvTree = true;

		unsigned char* navData = nullptr;
		int navDataSize = 0;
		REQUIRE(dtCreateNavMeshData(&params, &navData, &navDataSize));
		REQUIRE(navData != nullptr);

		nav = dtAllocNavMesh();
		REQUIRE(nav != nullptr);
		// The navmesh takes ownership of navData.
		REQUIRE(dtStatusSucceed(nav->init(navData, navDataSize, DT_TILE_FREE_DATA)));
	}

	~TestNavmesh()
	{
		dtFreeNavMesh(nav);
	}

	static int vertIndex(int x, int z) { return x * (ROWS + 1) + z; }

	// Returns the flags of every polygon currently in the agent's corridor.
	bool corridorContainsFlags(const dtCrowdAgent* ag, unsigned short flags) const
	{
		const dtPolyRef* path = ag->corridor.getPath();
		const int npath = ag->corridor.getPathCount();
		for (int i = 0; i < npath; ++i)
		{
			const dtMeshTile* tile = nullptr;
			const dtPoly* poly = nullptr;
			if (dtStatusFailed(nav->getTileAndPolyByRef(path[i], &tile, &poly)))
				continue;
			if (poly->flags & flags)
				return true;
		}
		return false;
	}
};

const float TestNavmesh::CELL = 2.0f;

dtCrowdAgentParams makeAgentParams()
{
	dtCrowdAgentParams params;
	std::memset(&params, 0, sizeof(params));
	params.radius = 0.3f;
	params.height = 1.0f;
	params.maxAcceleration = 8.0f;
	params.maxSpeed = 3.0f;
	params.collisionQueryRange = params.radius * 12.0f;
	params.pathOptimizationRange = params.radius * 30.0f;
	params.separationWeight = 2.0f;
	params.updateFlags = DT_CROWD_ANTICIPATE_TURNS | DT_CROWD_OPTIMIZE_VIS | DT_CROWD_OPTIMIZE_TOPO;
	params.obstacleAvoidanceType = 0;
	params.queryFilterType = 0;
	return params;
}

} // namespace

TEST_CASE("dtCrowd per-agent query filters", "[crowd]")
{
	TestNavmesh mesh;
	dtCrowd* crowd = dtAllocCrowd();
	REQUIRE(crowd->init(2, 0.5f, mesh.nav));

	// Shared filter slot 0: ground units cannot enter water.
	crowd->getEditableFilter(0)->setIncludeFlags(POLYFLAGS_GROUND);

	dtCrowdAgentParams params = makeAgentParams();

	// Ground agent: added through the original API, uses shared filter slot 0.
	const float groundStart[3] = { 1.0f, 0.0f, 1.0f };
	const float groundGoal[3] = { 9.0f, 0.0f, 1.0f };
	const int groundIdx = crowd->addAgent(groundStart, &params);
	REQUIRE(groundIdx >= 0);

	// Flying agent: custom per-agent filter that treats water as walkable.
	dtQueryFilter flyFilter;
	flyFilter.setIncludeFlags(POLYFLAGS_ALL);
	flyFilter.setAreaCost(AREA_WATER, 1.0f);
	const float flyStart[3] = { 1.0f, 0.0f, 5.0f };
	const float flyGoal[3] = { 9.0f, 0.0f, 5.0f };
	const int flyIdx = crowd->addAgent(flyStart, &params, &flyFilter);
	REQUIRE(flyIdx >= 0);

	SECTION("filter assignment and backward compatibility")
	{
		// The agent without a custom filter uses the shared filter slot.
		CHECK(crowd->getAgentFilter(groundIdx) == crowd->getFilter(0));
		CHECK(crowd->getAgentFilter(groundIdx)->getIncludeFlags() == POLYFLAGS_GROUND);

		// The agent with a custom filter uses its own copy.
		const dtQueryFilter* effective = crowd->getAgentFilter(flyIdx);
		REQUIRE(effective != nullptr);
		CHECK(effective != crowd->getFilter(0));
		CHECK(effective->getIncludeFlags() == POLYFLAGS_ALL);
		CHECK(effective->getAreaCost(AREA_WATER) == Catch::Approx(1.0f));

		// The filter is copied: later changes to the caller's filter don't affect the agent.
		flyFilter.setIncludeFlags(POLYFLAGS_GROUND);
		CHECK(crowd->getAgentFilter(flyIdx)->getIncludeFlags() == POLYFLAGS_ALL);

		// setAgentFilter can replace and clear the custom filter.
		dtQueryFilter groundOnly;
		groundOnly.setIncludeFlags(POLYFLAGS_GROUND);
		crowd->setAgentFilter(flyIdx, &groundOnly);
		CHECK(crowd->getAgentFilter(flyIdx)->getIncludeFlags() == POLYFLAGS_GROUND);
		CHECK(crowd->getAgentFilter(flyIdx) != crowd->getFilter(0));
		crowd->setAgentFilter(flyIdx, nullptr);
		CHECK(crowd->getAgentFilter(flyIdx) == crowd->getFilter(0));

		// Invalid indices are handled gracefully.
		CHECK(crowd->getAgentFilter(-1) == nullptr);
		CHECK(crowd->getAgentFilter(crowd->getAgentCount()) == nullptr);
	}

	SECTION("ground and flying agents in the same crowd take different routes")
	{
		const dtNavMeshQuery* query = crowd->getNavMeshQuery();
		const float* ext = crowd->getQueryHalfExtents();

		dtPolyRef groundGoalRef = 0;
		float pt[3];
		REQUIRE(dtStatusSucceed(query->findNearestPoly(groundGoal, ext, crowd->getAgentFilter(groundIdx), &groundGoalRef, pt)));
		REQUIRE(groundGoalRef != 0);
		REQUIRE(crowd->requestMoveTarget(groundIdx, groundGoalRef, groundGoal));

		dtPolyRef flyGoalRef = 0;
		REQUIRE(dtStatusSucceed(query->findNearestPoly(flyGoal, ext, crowd->getAgentFilter(flyIdx), &flyGoalRef, pt)));
		REQUIRE(flyGoalRef != 0);
		REQUIRE(crowd->requestMoveTarget(flyIdx, flyGoalRef, flyGoal));

		bool groundEnteredWater = false;
		bool flyCrossedWater = false;

		const float dt = 1.0f / 30.0f;
		for (int iter = 0; iter < 600; ++iter)
		{
			crowd->update(dt, nullptr);

			const dtCrowdAgent* ground = crowd->getAgent(groundIdx);
			const dtCrowdAgent* fly = crowd->getAgent(flyIdx);

			// The corridor of each agent must only contain polygons that pass
			// the agent's own filter (no walls crossed, no illegal areas).
			const dtPolyRef* gpath = ground->corridor.getPath();
			for (int i = 0; i < ground->corridor.getPathCount(); ++i)
				CHECK(query->isValidPolyRef(gpath[i], crowd->getAgentFilter(groundIdx)));
			const dtPolyRef* fpath = fly->corridor.getPath();
			for (int i = 0; i < fly->corridor.getPathCount(); ++i)
				CHECK(query->isValidPolyRef(fpath[i], crowd->getAgentFilter(flyIdx)));

			groundEnteredWater |= mesh.corridorContainsFlags(ground, POLYFLAGS_WATER);
			flyCrossedWater |= mesh.corridorContainsFlags(fly, POLYFLAGS_WATER);

			// Both agents must stay on the navmesh surface, inside the tile bounds.
			for (const dtCrowdAgent* ag : { ground, fly })
			{
				CHECK(ag->npos[1] == Catch::Approx(0.0f).margin(0.01f));
				CHECK(ag->npos[0] >= -0.01f);
				CHECK(ag->npos[0] <= 10.01f);
				CHECK(ag->npos[2] >= -0.01f);
				CHECK(ag->npos[2] <= 6.01f);
			}
		}

		const dtCrowdAgent* ground = crowd->getAgent(groundIdx);
		const dtCrowdAgent* fly = crowd->getAgent(flyIdx);

		// Both agents reached their goals.
		CHECK(ground->targetState == DT_CROWDAGENT_TARGET_VALID);
		CHECK(fly->targetState == DT_CROWDAGENT_TARGET_VALID);
		CHECK(dtVdist2D(ground->npos, groundGoal) == Catch::Approx(0.0f).margin(0.5f));
		CHECK(dtVdist2D(fly->npos, flyGoal) == Catch::Approx(0.0f).margin(0.5f));

		// The ground agent never entered the water (it must have used the bridge),
		// while the flying agent crossed the water directly.
		CHECK_FALSE(groundEnteredWater);
		CHECK(flyCrossedWater);
	}

	dtFreeCrowd(crowd);
}
