//
// Tests for dtNavMesh tile swap-out (temporary unload) support and for
// dtPathCorridor::isValid() detection of removed tiles.
//

#include <cstring>

#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourPathCorridor.h"
#include "DetourStatus.h"
#include "catch2/catch_amalgamated.hpp"

namespace
{

// Tiles are 10x10 voxels, 1 world unit per voxel.
const int TILE_SIZE = 10;

// Builds the data for a tile holding a single quad that spans the whole tile,
// with portal edges towards the x- and x+ neighbours.
unsigned char* createTileData(const int tileX, const int tileY, int* outDataSize)
{
	const unsigned short verts[] = {
		0, 0, 0,
		TILE_SIZE, 0, 0,
		TILE_SIZE, 0, TILE_SIZE,
		0, 0, TILE_SIZE,
	};
	const unsigned short nullIdx = 0xffff;
	const unsigned short border = 0x8000 | 0xf; // solid border
	const unsigned short portalXMinus = 0x8000 | 0;
	const unsigned short portalXPlus = 0x8000 | 2;
	const unsigned short polys[] = {
		// Vertices.
		0, 1, 2, 3, nullIdx, nullIdx,
		// Edge adjacency: edge 1 (x+) and edge 3 (x-) are portals to the neighbour tiles.
		border, portalXPlus, border, portalXMinus, nullIdx, nullIdx,
	};
	const unsigned short polyFlags[] = { 1 };
	const unsigned char polyAreas[] = { 0 };

	dtNavMeshCreateParams params;
	memset(&params, 0, sizeof(params));
	params.verts = verts;
	params.vertCount = 4;
	params.polys = polys;
	params.polyFlags = polyFlags;
	params.polyAreas = polyAreas;
	params.polyCount = 1;
	params.nvp = 6;
	params.walkableHeight = 2.0f;
	params.walkableRadius = 0.5f;
	params.walkableClimb = 0.9f;
	params.tileX = tileX;
	params.tileY = tileY;
	params.tileLayer = 0;
	params.bmin[0] = (float)(tileX * TILE_SIZE);
	params.bmin[1] = 0.0f;
	params.bmin[2] = (float)(tileY * TILE_SIZE);
	params.bmax[0] = (float)(tileX * TILE_SIZE + TILE_SIZE);
	params.bmax[1] = 2.0f;
	params.bmax[2] = (float)(tileY * TILE_SIZE + TILE_SIZE);
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

// A navmesh made of a row of single-poly tiles along the x axis.
struct StripNavMesh
{
	static const int TILE_COUNT = 5;

	dtNavMesh* nav;
	dtNavMeshQuery* query;
	dtQueryFilter filter;
	dtTileRef tileRef[TILE_COUNT];
	unsigned char* tileData[TILE_COUNT];
	int tileDataSize[TILE_COUNT];

	StripNavMesh()
		: nav(0), query(0)
	{
		dtNavMeshParams params;
		memset(&params, 0, sizeof(params));
		params.tileWidth = (float)TILE_SIZE;
		params.tileHeight = (float)TILE_SIZE;
		params.maxTiles = 16;
		params.maxPolys = 8;

		nav = dtAllocNavMesh();
		REQUIRE(nav != 0);
		REQUIRE(dtStatusSucceed(nav->init(&params)));

		for (int i = 0; i < TILE_COUNT; ++i)
		{
			tileRef[i] = 0;
			tileData[i] = createTileData(i, 0, &tileDataSize[i]);
			REQUIRE(tileData[i] != 0);
			// Tiles are added without DT_TILE_FREE_DATA: the fixture owns the
			// data and frees it, whether the tile is currently added or not.
			REQUIRE(dtStatusSucceed(nav->addTile(tileData[i], tileDataSize[i], 0, 0, &tileRef[i])));
		}

		query = dtAllocNavMeshQuery();
		REQUIRE(query != 0);
		REQUIRE(dtStatusSucceed(query->init(nav, 2048)));
	}

	~StripNavMesh()
	{
		dtFreeNavMeshQuery(query);
		dtFreeNavMesh(nav);
		for (int i = 0; i < TILE_COUNT; ++i)
			dtFree(tileData[i]);
	}

	// The reference of the single polygon of tile i (valid while the tile is added).
	dtPolyRef polyRef(const int i) const
	{
		return nav->getPolyRefBase(nav->getTileByRef(tileRef[i]));
	}

	// Finds a path from tile 0 to tile 4, crossing all 5 tiles.
	int findStripPath(dtPolyRef* path, const int maxPath)
	{
		const float extents[3] = { 2.0f, 4.0f, 2.0f };
		const float startPos[3] = { 5.0f, 0.0f, 5.0f };
		const float endPos[3] = { 45.0f, 0.0f, 5.0f };
		dtPolyRef startRef = 0, endRef = 0;
		float nearest[3];
		REQUIRE(dtStatusSucceed(query->findNearestPoly(startPos, extents, &filter, &startRef, nearest)));
		REQUIRE(dtStatusSucceed(query->findNearestPoly(endPos, extents, &filter, &endRef, nearest)));
		int npath = 0;
		REQUIRE(dtStatusSucceed(query->findPath(startRef, endRef, startPos, endPos, &filter, path, &npath, maxPath)));
		return npath;
	}
};

} // namespace

TEST_CASE("dtNavMesh tile swap out")
{
	StripNavMesh mesh;

	SECTION("swap out reserves the refs, reload restores them unchanged")
	{
		const dtPolyRef poly = mesh.polyRef(2);
		const unsigned int version = mesh.nav->getTileRemovalVersion();

		unsigned char* data = 0;
		int dataSize = 0;
		REQUIRE(dtStatusSucceed(mesh.nav->swapOutTile(mesh.tileRef[2], &data, &dataSize)));
		CHECK(data == mesh.tileData[2]);
		CHECK(dataSize == mesh.tileDataSize[2]);

		// While swapped out the refs are not valid, but they are not dead either.
		CHECK_FALSE(mesh.nav->isValidPolyRef(poly));
		CHECK(mesh.nav->isSwappedOutPolyRef(poly));
		// Swapping out is not a permanent removal.
		CHECK(mesh.nav->getTileRemovalVersion() == version);

		// Reloading with the saved tile ref restores the exact same poly refs.
		dtTileRef result = 0;
		REQUIRE(dtStatusSucceed(mesh.nav->addTile(data, dataSize, 0, mesh.tileRef[2], &result)));
		CHECK(result == mesh.tileRef[2]);
		CHECK(mesh.nav->isValidPolyRef(poly));
		CHECK_FALSE(mesh.nav->isSwappedOutPolyRef(poly));
		CHECK(mesh.nav->getTileRemovalVersion() == version);
	}

	SECTION("freeing a swapped out tile kills its refs")
	{
		const dtPolyRef poly = mesh.polyRef(2);

		unsigned char* data = 0;
		int dataSize = 0;
		REQUIRE(dtStatusSucceed(mesh.nav->swapOutTile(mesh.tileRef[2], &data, &dataSize)));
		const unsigned int version = mesh.nav->getTileRemovalVersion();

		REQUIRE(dtStatusSucceed(mesh.nav->freeSwappedOutTile(mesh.tileRef[2])));
		CHECK_FALSE(mesh.nav->isSwappedOutPolyRef(poly));
		CHECK_FALSE(mesh.nav->isValidPolyRef(poly));
		CHECK(mesh.nav->getTileRemovalVersion() == version + 1);

		// The tile is no longer swapped out; freeing again fails.
		CHECK(dtStatusFailed(mesh.nav->freeSwappedOutTile(mesh.tileRef[2])));
	}

	SECTION("removeTile is a permanent removal")
	{
		const dtPolyRef poly = mesh.polyRef(2);
		const unsigned int version = mesh.nav->getTileRemovalVersion();

		unsigned char* data = 0;
		int dataSize = 0;
		REQUIRE(dtStatusSucceed(mesh.nav->removeTile(mesh.tileRef[2], &data, &dataSize)));
		CHECK_FALSE(mesh.nav->isValidPolyRef(poly));
		CHECK_FALSE(mesh.nav->isSwappedOutPolyRef(poly));
		CHECK(mesh.nav->getTileRemovalVersion() == version + 1);
	}

	SECTION("reusing the slot for another tile unregisters the swapped out tile")
	{
		const dtPolyRef poly = mesh.polyRef(2);

		unsigned char* data = 0;
		int dataSize = 0;
		REQUIRE(dtStatusSucceed(mesh.nav->swapOutTile(mesh.tileRef[2], &data, &dataSize)));
		const unsigned int version = mesh.nav->getTileRemovalVersion();

		// The free list is LIFO, so the new tile reuses the just-vacated slot.
		int otherDataSize = 0;
		unsigned char* otherData = createTileData(0, 1, &otherDataSize);
		REQUIRE(otherData != 0);
		dtTileRef otherRef = 0;
		REQUIRE(dtStatusSucceed(mesh.nav->addTile(otherData, otherDataSize, 0, 0, &otherRef)));
		CHECK(mesh.nav->decodePolyIdTile((dtPolyRef)otherRef) ==
			  mesh.nav->decodePolyIdTile((dtPolyRef)mesh.tileRef[2]));

		// The old refs are dead for good now.
		CHECK_FALSE(mesh.nav->isSwappedOutPolyRef(poly));
		CHECK_FALSE(mesh.nav->isValidPolyRef(poly));
		CHECK(mesh.nav->getTileRemovalVersion() == version + 1);

		dtFree(otherData);
	}

	SECTION("swap out rejects invalid refs")
	{
		CHECK(dtStatusFailed(mesh.nav->swapOutTile(0, 0, 0)));

		// A tile that is not currently in the navmesh cannot be swapped out.
		unsigned char* data = 0;
		int dataSize = 0;
		REQUIRE(dtStatusSucceed(mesh.nav->removeTile(mesh.tileRef[2], &data, &dataSize)));
		CHECK(dtStatusFailed(mesh.nav->swapOutTile(mesh.tileRef[2], &data, &dataSize)));
	}
}

TEST_CASE("dtPathCorridor::isValid tile removal")
{
	StripNavMesh mesh;

	dtPolyRef path[8];
	const int npath = mesh.findStripPath(path, 8);
	REQUIRE(npath == StripNavMesh::TILE_COUNT);

	const float targetPos[3] = { 45.0f, 0.0f, 5.0f };
	dtPathCorridor corridor;
	REQUIRE(corridor.init(16));
	corridor.setCorridor(targetPos, path, npath);

	SECTION("detects permanently removed tiles beyond the look-ahead")
	{
		REQUIRE(corridor.isValid(3, mesh.query, &mesh.filter));

		// Remove a tile whose polygons are beyond the 3-poly look-ahead window.
		unsigned char* data = 0;
		int dataSize = 0;
		REQUIRE(dtStatusSucceed(mesh.nav->removeTile(mesh.tileRef[3], &data, &dataSize)));

		// The corridor must notice that its path is broken even though the
		// look-ahead polygons are all still valid.
		CHECK_FALSE(corridor.isValid(3, mesh.query, &mesh.filter));
		// Repeated checks keep reporting the corridor as invalid.
		CHECK_FALSE(corridor.isValid(3, mesh.query, &mesh.filter));

		// Re-adding the tile with its original ref heals the corridor.
		dtTileRef result = 0;
		REQUIRE(dtStatusSucceed(mesh.nav->addTile(data, dataSize, 0, mesh.tileRef[3], &result)));
		CHECK(result == mesh.tileRef[3]);
		CHECK(corridor.isValid(3, mesh.query, &mesh.filter));
	}

	SECTION("tolerates temporarily swapped out tiles")
	{
		REQUIRE(corridor.isValid(3, mesh.query, &mesh.filter));

		// Swap out a tile beyond the look-ahead: no replan needed, the refs
		// become valid again once the tile is reloaded.
		unsigned char* data3 = 0;
		int dataSize3 = 0;
		REQUIRE(dtStatusSucceed(mesh.nav->swapOutTile(mesh.tileRef[3], &data3, &dataSize3)));
		CHECK(corridor.isValid(3, mesh.query, &mesh.filter));

		// Swap out a tile inside the look-ahead window: still no replan.
		unsigned char* data1 = 0;
		int dataSize1 = 0;
		REQUIRE(dtStatusSucceed(mesh.nav->swapOutTile(mesh.tileRef[1], &data1, &dataSize1)));
		CHECK(corridor.isValid(3, mesh.query, &mesh.filter));

		// Reloading restores the exact same refs; the corridor stays valid.
		dtTileRef result = 0;
		REQUIRE(dtStatusSucceed(mesh.nav->addTile(data1, dataSize1, 0, mesh.tileRef[1], &result)));
		CHECK(result == mesh.tileRef[1]);
		CHECK(corridor.isValid(3, mesh.query, &mesh.filter));

		// Permanently freeing the other swapped out tile invalidates the corridor.
		REQUIRE(dtStatusSucceed(mesh.nav->freeSwappedOutTile(mesh.tileRef[3])));
		CHECK_FALSE(corridor.isValid(3, mesh.query, &mesh.filter));
	}

	SECTION("detects removed tiles within the look-ahead")
	{
		unsigned char* data = 0;
		int dataSize = 0;
		REQUIRE(dtStatusSucceed(mesh.nav->removeTile(mesh.tileRef[1], &data, &dataSize)));
		CHECK_FALSE(corridor.isValid(3, mesh.query, &mesh.filter));
	}

	SECTION("still detects flag changes within the look-ahead")
	{
		REQUIRE(corridor.isValid(3, mesh.query, &mesh.filter));
		REQUIRE(dtStatusSucceed(mesh.nav->setPolyFlags(path[1], 0)));
		CHECK_FALSE(corridor.isValid(3, mesh.query, &mesh.filter));
	}
}
