#include <cstring>

#include "DetourCommon.h"
#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "catch2/catch_amalgamated.hpp"

// These tests build a minimal two-tile navmesh by hand:
//
//   tile A: single quad, x in [0,10],  z in [0,10], y = 0, tile grid (0,0)
//   tile B: single quad, x in [10,20], z in [0,10], y = 0, tile grid (1,0)
//
// Tile A owns a bidirectional off-mesh connection from (9,0,5) to (19,0,5)
// whose far end links into tile B once both tiles are loaded. This mimics a
// pair of adjacent tiles bridged by an off-mesh link, and allows tile B to be
// dynamically unloaded (simulating a tile cache rebuild) while agents of a
// dtCrowd are traversing the link. The link is deliberately long so that an
// agent landing at the far end cannot be recovered by findNearestPoly() onto
// tile A (the crowd placement half extents are ~1.2 world units).
namespace
{

static const float CONNECTION_START[3] = { 9.0f, 0.0f, 5.0f };
static const float CONNECTION_END[3] = { 19.0f, 0.0f, 5.0f };

unsigned char* buildTestTile(const float bminX, const int tileX, const bool withOffMeshCon, int* outDataSize)
{
	// One quad, 10x10 world units, surface at y = 0 (bmin.y = -1, ch = 1).
	static const unsigned short verts[] = {
		0, 1, 0,
		10, 1, 0,
		10, 1, 10,
		0, 1, 10,
	};
	static const unsigned short polys[] = {
		0, 1, 2, 3, 0xffff, 0xffff,						// vertex indices (nvp = 6)
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,	// no adjacency
	};
	static const unsigned short polyFlags[] = { 1 };
	static const unsigned char polyAreas[] = { 0 };

	static const float offMeshConVerts[] = {
		CONNECTION_START[0], CONNECTION_START[1], CONNECTION_START[2],
		CONNECTION_END[0], CONNECTION_END[1], CONNECTION_END[2],
	};
	static const float offMeshConRad[] = { 1.0f };
	static const unsigned short offMeshConFlags[] = { 1 };
	static const unsigned char offMeshConAreas[] = { 0 };
	static const unsigned char offMeshConDir[] = { DT_OFFMESH_CON_BIDIR };
	static const unsigned int offMeshConUserID[] = { 1 };

	dtNavMeshCreateParams params;
	memset(&params, 0, sizeof(params));
	params.verts = verts;
	params.vertCount = 4;
	params.polys = polys;
	params.polyFlags = polyFlags;
	params.polyAreas = polyAreas;
	params.polyCount = 1;
	params.nvp = 6;
	if (withOffMeshCon)
	{
		params.offMeshConVerts = offMeshConVerts;
		params.offMeshConRad = offMeshConRad;
		params.offMeshConFlags = offMeshConFlags;
		params.offMeshConAreas = offMeshConAreas;
		params.offMeshConDir = offMeshConDir;
		params.offMeshConUserID = offMeshConUserID;
		params.offMeshConCount = 1;
	}
	params.tileX = tileX;
	params.tileY = 0;
	params.tileLayer = 0;
	params.bmin[0] = bminX;
	params.bmin[1] = -1.0f;
	params.bmin[2] = 0.0f;
	params.bmax[0] = bminX + 10.0f;
	params.bmax[1] = 1.0f;
	params.bmax[2] = 10.0f;
	params.walkableHeight = 2.0f;
	params.walkableRadius = 0.5f;
	params.walkableClimb = 0.9f;
	params.cs = 1.0f;
	params.ch = 1.0f;
	params.buildBvTree = true;

	unsigned char* data = 0;
	int dataSize = 0;
	if (!dtCreateNavMeshData(&params, &data, &dataSize))
		return 0;

	*outDataSize = dataSize;
	return data;
}

struct OffMeshCrowdFixture
{
	static const int MAX_AGENTS = 8;

	dtNavMesh* navmesh = 0;
	dtNavMeshQuery* navquery = 0;
	dtCrowd* crowd = 0;
	dtPolyRef tileBRef = 0;
	dtQueryFilter filter;

	OffMeshCrowdFixture()
	{
		dtNavMeshParams nmparams;
		memset(&nmparams, 0, sizeof(nmparams));
		nmparams.orig[0] = 0.0f;
		nmparams.orig[1] = -1.0f;
		nmparams.orig[2] = 0.0f;
		nmparams.tileWidth = 10.0f;
		nmparams.tileHeight = 10.0f;
		nmparams.maxTiles = 32;
		nmparams.maxPolys = 16;

		navmesh = dtAllocNavMesh();
		REQUIRE(navmesh != 0);
		REQUIRE(dtStatusSucceed(navmesh->init(&nmparams)));

		addTileA();
		addTileB();

		navquery = dtAllocNavMeshQuery();
		REQUIRE(navquery != 0);
		REQUIRE(dtStatusSucceed(navquery->init(navmesh, 2048)));

		crowd = dtAllocCrowd();
		REQUIRE(crowd != 0);
		REQUIRE(crowd->init(MAX_AGENTS, 0.6f, navmesh));
	}

	~OffMeshCrowdFixture()
	{
		dtFreeCrowd(crowd);
		dtFreeNavMeshQuery(navquery);
		dtFreeNavMesh(navmesh);
	}

	void addTileA()
	{
		int dataSize = 0;
		unsigned char* data = buildTestTile(0.0f, 0, true, &dataSize);
		REQUIRE(data != 0);
		REQUIRE(dtStatusSucceed(navmesh->addTile(data, dataSize, DT_TILE_FREE_DATA, 0, 0)));
	}

	void addTileB()
	{
		int dataSize = 0;
		unsigned char* data = buildTestTile(10.0f, 1, false, &dataSize);
		REQUIRE(data != 0);
		REQUIRE(dtStatusSucceed(navmesh->addTile(data, dataSize, DT_TILE_FREE_DATA, 0, &tileBRef)));
		REQUIRE(tileBRef != 0);
	}

	void removeTileB()
	{
		REQUIRE(dtStatusSucceed(navmesh->removeTile(tileBRef, 0, 0)));
		tileBRef = 0;
	}

	int addAgent(const float* pos)
	{
		dtCrowdAgentParams ap;
		memset(&ap, 0, sizeof(ap));
		ap.radius = 0.5f;
		ap.height = 2.0f;
		ap.maxAcceleration = 8.0f;
		ap.maxSpeed = 3.5f;
		ap.collisionQueryRange = ap.radius * 12.0f;
		ap.pathOptimizationRange = ap.radius * 30.0f;
		ap.updateFlags = DT_CROWD_ANTICIPATE_TURNS | DT_CROWD_OPTIMIZE_VIS | DT_CROWD_OPTIMIZE_TOPO |
						 DT_CROWD_OBSTACLE_AVOIDANCE;
		ap.obstacleAvoidanceType = 0;
		ap.separationWeight = 2.0f;
		const int idx = crowd->addAgent(pos, &ap);
		REQUIRE(idx != -1);
		return idx;
	}

	dtPolyRef findTarget(const float* pos, float* targetPos)
	{
		const float halfExtents[3] = { 2.0f, 2.0f, 2.0f };
		dtPolyRef ref = 0;
		REQUIRE(dtStatusSucceed(navquery->findNearestPoly(pos, halfExtents, &filter, &ref, targetPos)));
		REQUIRE(ref != 0);
		return ref;
	}

	void requestTarget(const int idx, const float* pos)
	{
		float targetPos[3];
		const dtPolyRef targetRef = findTarget(pos, targetPos);
		REQUIRE(crowd->requestMoveTarget(idx, targetRef, targetPos));
	}

	void step(const int frames)
	{
		for (int i = 0; i < frames; ++i)
			crowd->update(1.0f / 30.0f, 0);
	}

	bool agentOnNavmesh(const int idx)
	{
		const dtCrowdAgent* ag = crowd->getAgent(idx);
		if (!ag->active)
			return false;
		if (ag->state == DT_CROWDAGENT_STATE_INVALID)
			return false;
		return navquery->isValidPolyRef(ag->corridor.getFirstPoly(), &filter);
	}
};

} // namespace

TEST_CASE("dtCrowd off-mesh connection traversal")
{
	OffMeshCrowdFixture f;

	SECTION("agent crosses the off-mesh connection when nothing changes")
	{
		const float startPos[3] = { 2.0f, 0.0f, 5.0f };
		const float goalPos[3] = { 18.0f, 0.0f, 5.0f };
		const int idx = f.addAgent(startPos);
		f.requestTarget(idx, goalPos);

		bool sawOffmesh = false;
		bool arrived = false;
		for (int i = 0; i < 600 && !arrived; ++i)
		{
			f.crowd->update(1.0f / 30.0f, 0);
			const dtCrowdAgent* ag = f.crowd->getAgent(idx);
			if (ag->state == DT_CROWDAGENT_STATE_OFFMESH)
				sawOffmesh = true;
			float diff[3];
			dtVsub(diff, ag->npos, goalPos);
			diff[1] = 0.0f;
			if (dtVlenSqr(diff) < dtSqr(1.0f) && ag->state == DT_CROWDAGENT_STATE_WALKING)
				arrived = true;
		}

		CHECK(sawOffmesh);
		REQUIRE(arrived);
		CHECK(f.crowd->getAgent(idx)->state == DT_CROWDAGENT_STATE_WALKING);
		CHECK(f.agentOnNavmesh(idx));
	}

	SECTION("agents fall back to walking when the target tile is unloaded mid-traversal")
	{
		const float startPositions[3][3] = {
			{ 2.0f, 0.0f, 3.0f },
			{ 2.0f, 0.0f, 5.0f },
			{ 2.0f, 0.0f, 7.0f },
		};
		const float goalPositions[3][3] = {
			{ 18.0f, 0.0f, 3.0f },
			{ 18.0f, 0.0f, 5.0f },
			{ 18.0f, 0.0f, 7.0f },
		};

		int idx[3];
		for (int i = 0; i < 3; ++i)
		{
			idx[i] = f.addAgent(startPositions[i]);
			f.requestTarget(idx[i], goalPositions[i]);
		}

		// Step until at least one agent is traversing the off-mesh connection.
		bool sawOffmesh = false;
		for (int i = 0; i < 600 && !sawOffmesh; ++i)
		{
			f.crowd->update(1.0f / 30.0f, 0);
			for (int j = 0; j < 3; ++j)
			{
				if (f.crowd->getAgent(idx[j])->state == DT_CROWDAGENT_STATE_OFFMESH)
					sawOffmesh = true;
			}
		}
		REQUIRE(sawOffmesh);

		// Simulate a tile cache rebuild unloading the link's target tile.
		f.removeTileB();

		// The agents must not remain stuck in the off-mesh state.
		f.step(10);
		for (int i = 0; i < 3; ++i)
		{
			const dtCrowdAgent* ag = f.crowd->getAgent(idx[i]);
			CAPTURE(i, ag->npos[0], ag->npos[1], ag->npos[2]);
			CHECK(ag->state == DT_CROWDAGENT_STATE_WALKING);
			CHECK(f.agentOnNavmesh(idx[i]));
		}

		// The agents must remain on the navmesh (no fall-through, no invalid state).
		f.step(60);
		for (int i = 0; i < 3; ++i)
		{
			CHECK(f.crowd->getAgent(idx[i])->state == DT_CROWDAGENT_STATE_WALKING);
			CHECK(f.agentOnNavmesh(idx[i]));
		}

		// Bring the target tile back (tile cache finished rebuilding) and let
		// the agents replan across the link.
		f.addTileB();
		for (int i = 0; i < 3; ++i)
			f.requestTarget(idx[i], goalPositions[i]);

		bool allArrived = false;
		for (int frame = 0; frame < 900 && !allArrived; ++frame)
		{
			f.crowd->update(1.0f / 30.0f, 0);
			allArrived = true;
			for (int i = 0; i < 3; ++i)
			{
				const dtCrowdAgent* ag = f.crowd->getAgent(idx[i]);
				float diff[3];
				dtVsub(diff, ag->npos, goalPositions[i]);
				diff[1] = 0.0f;
				if (dtVlenSqr(diff) > dtSqr(2.0f) || ag->state != DT_CROWDAGENT_STATE_WALKING)
					allArrived = false;
			}
		}

		REQUIRE(allArrived);
		for (int i = 0; i < 3; ++i)
		{
			CHECK(f.crowd->getAgent(idx[i])->state == DT_CROWDAGENT_STATE_WALKING);
			CHECK(f.agentOnNavmesh(idx[i]));
		}
	}

	SECTION("agent does not enter a broken off-mesh connection")
	{
		const float startPos[3] = { 2.0f, 0.0f, 5.0f };
		const float goalPos[3] = { 18.0f, 0.0f, 5.0f };
		const int idx = f.addAgent(startPos);
		f.requestTarget(idx, goalPos);

		// Unload the target tile before the agent reaches the link.
		f.removeTileB();

		// The agent should replan and stop on tile A instead of getting stuck.
		f.step(120);
		const dtCrowdAgent* ag = f.crowd->getAgent(idx);
		CHECK(ag->state == DT_CROWDAGENT_STATE_WALKING);
		CHECK(f.agentOnNavmesh(idx));
		// The agent must stay on tile A's side of the gap.
		CHECK(ag->npos[0] < 15.0f);
	}
}
