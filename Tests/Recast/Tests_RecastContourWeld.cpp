//
// Tests for cross-tile contour border vertex welding
// (rcBuildContoursEx / rcWeldContourBorderVertices).
//

#include <math.h>
#include <string.h>

#include <vector>

#include "catch2/catch_amalgamated.hpp"

#include "Recast.h"
#include "RecastAlloc.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"

namespace
{

// Builds a single-contour contour set by hand for unit testing the weld pass.
rcContourSet* makeContourSet(const int width, const int height, const int borderSize, const float cs,
							 const int* verts, const int nverts)
{
	rcContourSet* cset = rcAllocContourSet();
	cset->cs = cs;
	cset->ch = 0.2f;
	cset->width = width;
	cset->height = height;
	cset->borderSize = borderSize;
	cset->nconts = 1;
	cset->conts = (rcContour*)rcAlloc(sizeof(rcContour), RC_ALLOC_PERM);
	memset(cset->conts, 0, sizeof(rcContour));
	rcContour& cont = cset->conts[0];
	cont.nverts = nverts;
	cont.verts = (int*)rcAlloc(sizeof(int) * nverts * 4, RC_ALLOC_PERM);
	memcpy(cont.verts, verts, sizeof(int) * nverts * 4);
	cont.reg = 1;
	cont.area = RC_WALKABLE_AREA;
	return cset;
}

void addBox(std::vector<float>& verts, std::vector<int>& tris,
			const float x0, const float y0, const float z0,
			const float x1, const float y1, const float z1)
{
	const int base = (int)verts.size() / 3;
	const float corners[8][3] = {
		{x0, y0, z0}, {x1, y0, z0}, {x1, y0, z1}, {x0, y0, z1},
		{x0, y1, z0}, {x1, y1, z0}, {x1, y1, z1}, {x0, y1, z1},
	};
	for (int i = 0; i < 8; ++i)
	{
		verts.push_back(corners[i][0]);
		verts.push_back(corners[i][1]);
		verts.push_back(corners[i][2]);
	}
	// Outward wound faces (y-up).
	const int idx[36] = {
		4, 6, 5, 4, 7, 6,  // top
		0, 2, 1, 0, 3, 2,  // bottom
		0, 5, 1, 0, 4, 5,  // -z
		2, 7, 3, 2, 6, 7,  // +z
		3, 4, 0, 3, 7, 4,  // -x
		1, 6, 2, 1, 5, 6,  // +x
	};
	for (int i = 0; i < 36; ++i)
		tris.push_back(base + idx[i]);
}

struct TileBuild
{
	rcCompactHeightfield* chf = nullptr;
	rcContourSet* cset = nullptr;
	rcPolyMesh* pmesh = nullptr;
	rcPolyMeshDetail* dmesh = nullptr;

	~TileBuild()
	{
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		rcFreePolyMesh(pmesh);
		rcFreePolyMeshDetail(dmesh);
	}
};

const float kCellSize = 0.3f;
const float kCellHeight = 0.2f;
const int kTileSizeVox = 64;
const float kTileWorld = kTileSizeVox * kCellSize;  // 19.2
const int kWalkableHeight = 10;   // 2.0 / 0.2
const int kWalkableClimb = 4;     // 0.9 / 0.2
const int kWalkableRadius = 2;    // 0.6 / 0.3
const int kBorderSize = kWalkableRadius + 3;
const int kMaxVertsPerPoly = 6;

// Runs the standard tiled pipeline for tile (tx, tz) up to the poly mesh.
// contourFlags/weldThreshold are forwarded to rcBuildContoursEx; when
// useLegacyContourApi is true the old rcBuildContours entry point is used instead.
bool buildTile(rcContext& ctx,
			   const std::vector<float>& verts, const std::vector<int>& tris,
			   const float* bmin, const float* bmax,
			   const int tx, const int tz,
			   const int contourFlags, const float weldThreshold, const bool useLegacyContourApi,
			   TileBuild& out)
{
	const int width = kTileSizeVox + kBorderSize * 2;
	const float pad = kBorderSize * kCellSize;
	const float tbmin[3] = {
		bmin[0] + tx * kTileWorld - pad,
		bmin[1],
		bmin[2] + tz * kTileWorld - pad,
	};
	const float tbmax[3] = {
		bmin[0] + (tx + 1) * kTileWorld + pad,
		bmax[1],
		bmin[2] + (tz + 1) * kTileWorld + pad,
	};

	rcHeightfield* hf = rcAllocHeightfield();
	if (!rcCreateHeightfield(&ctx, *hf, width, width, tbmin, tbmax, kCellSize, kCellHeight))
		return false;

	std::vector<unsigned char> areas(tris.size() / 3, RC_WALKABLE_AREA);
	rcMarkWalkableTriangles(&ctx, 45.0f, verts.data(), (int)verts.size() / 3,
							tris.data(), (int)tris.size() / 3, areas.data());
	if (!rcRasterizeTriangles(&ctx, verts.data(), (int)verts.size() / 3, tris.data(), areas.data(),
							  (int)tris.size() / 3, *hf, kWalkableClimb))
	{
		rcFreeHeightField(hf);
		return false;
	}

	rcFilterLowHangingWalkableObstacles(&ctx, kWalkableClimb, *hf);
	rcFilterLedgeSpans(&ctx, kWalkableHeight, kWalkableClimb, *hf);
	rcFilterWalkableLowHeightSpans(&ctx, kWalkableHeight, *hf);

	out.chf = rcAllocCompactHeightfield();
	if (!rcBuildCompactHeightfield(&ctx, kWalkableHeight, kWalkableClimb, *hf, *out.chf))
	{
		rcFreeHeightField(hf);
		return false;
	}
	rcFreeHeightField(hf);

	if (!rcErodeWalkableArea(&ctx, kWalkableRadius, *out.chf))
		return false;
	if (!rcBuildDistanceField(&ctx, *out.chf))
		return false;
	if (!rcBuildRegions(&ctx, *out.chf, kBorderSize, 8 * 8, 20 * 20))
		return false;

	out.cset = rcAllocContourSet();
	bool ok = false;
	if (useLegacyContourApi)
		ok = rcBuildContours(&ctx, *out.chf, 1.3f, 12 / kCellSize, *out.cset, RC_CONTOUR_TESS_WALL_EDGES);
	else
		ok = rcBuildContoursEx(&ctx, *out.chf, 1.3f, 12 / kCellSize, *out.cset, contourFlags, weldThreshold);
	if (!ok)
		return false;

	out.pmesh = rcAllocPolyMesh();
	if (!rcBuildPolyMesh(&ctx, *out.cset, kMaxVertsPerPoly, *out.pmesh))
		return false;

	out.dmesh = rcAllocPolyMeshDetail();
	if (!rcBuildPolyMeshDetail(&ctx, *out.pmesh, *out.chf, 6 * kCellSize, kCellHeight, *out.dmesh))
		return false;

	// Give every polygon a flag so the default query filter accepts them.
	for (int i = 0; i < out.pmesh->npolys; ++i)
		out.pmesh->flags[i] = 1;

	return true;
}

// Fixed test map: a flat ground plane with one tall box obstacle crossing the
// internal vertical tile border.
void buildTestGeometry(std::vector<float>& verts, std::vector<int>& tris, float* bmin, float* bmax)
{
	const float w = kTileWorld * 2;
	// Ground plane.
	const float gy = 0.0f;
	const float ground[4][3] = {{0, gy, 0}, {w, gy, 0}, {w, gy, w}, {0, gy, w}};
	for (int i = 0; i < 4; ++i)
	{
		verts.push_back(ground[i][0]);
		verts.push_back(ground[i][1]);
		verts.push_back(ground[i][2]);
	}
	tris.push_back(0); tris.push_back(2); tris.push_back(1);
	tris.push_back(0); tris.push_back(3); tris.push_back(2);

	// Box obstacle crossing the internal border x == kTileWorld.
	addBox(verts, tris, 14.0f, 0.0f, 8.0f, 25.0f, 3.0f, 15.0f);

	bmin[0] = 0; bmin[1] = 0; bmin[2] = 0;
	bmax[0] = w; bmax[1] = 5.0f; bmax[2] = w;
}

}  // namespace

TEST_CASE("rcWeldContourBorderVertices", "[recast][contour-weld]")
{
	rcContext ctx(false);

	SECTION("snaps off-plane border vertices onto the tile border planes")
	{
		const int F = RC_BORDER_VERTEX;
		const int verts[] = {
			0,   10, 20, F,
			1,   10, 40, F,   // 1 voxel off the x=0 plane -> snapped to x=0
			0,   10, 60, F,
			50,  10, 99, F,   // 1 voxel off the z=100 plane -> snapped to z=100
			100, 10, 50, F,
			60,  10, 0,  F,
			40,  10, 1,  F,   // 1 voxel off the z=0 plane -> snapped to z=0
			2,   10, 80, F,   // 2 voxels off any plane -> beyond threshold, unchanged
			30,  10, 30, 0,   // interior vertex -> unchanged
			1,   10, 70, 0,   // unflagged but 1 voxel off the x=0 plane -> snapped
		};
		rcContourSet* cset = makeContourSet(100, 100, 3, kCellSize, verts, 10);

		// Threshold of 1.5 cells welds vertices deviating up to one voxel.
		REQUIRE(rcWeldContourBorderVertices(&ctx, *cset, 1.5f * kCellSize));

		const rcContour& cont = cset->conts[0];
		REQUIRE(cont.nverts == 10);
		CHECK(cont.verts[1 * 4 + 0] == 0);    // snapped onto x=0
		CHECK(cont.verts[1 * 4 + 2] == 40);
		CHECK(cont.verts[3 * 4 + 2] == 100);  // snapped onto z=100
		CHECK(cont.verts[6 * 4 + 2] == 0);    // snapped onto z=0
		CHECK(cont.verts[7 * 4 + 0] == 2);    // out of reach, untouched
		CHECK(cont.verts[8 * 4 + 0] == 30);   // interior vertex, untouched
		CHECK(cont.verts[8 * 4 + 2] == 30);
		CHECK(cont.verts[9 * 4 + 0] == 0);    // snapped onto x=0 despite no flag
		CHECK(cont.verts[9 * 4 + 2] == 70);

		rcFreeContourSet(cset);
	}

	SECTION("quantizes the coordinate along the border to the canonical lattice")
	{
		const int F = RC_BORDER_VERTEX;
		const int verts[] = {
			0,  10, 21, F,   // -> 22 with a quantum of 2 voxels
			0,  10, 23, F,   // -> 24
			50, 10, 50, 0,
			0,  10, 27, F,   // -> 28
		};
		rcContourSet* cset = makeContourSet(100, 100, 3, kCellSize, verts, 4);

		// Threshold of 2.5 cells -> weld distance 2.5 voxels, lattice quantum 2 voxels.
		REQUIRE(rcWeldContourBorderVertices(&ctx, *cset, 2.5f * kCellSize));

		const rcContour& cont = cset->conts[0];
		REQUIRE(cont.nverts == 4);
		CHECK(cont.verts[0 * 4 + 2] == 22);
		CHECK(cont.verts[1 * 4 + 2] == 24);
		CHECK(cont.verts[3 * 4 + 2] == 28);

		rcFreeContourSet(cset);
	}

	SECTION("removes vertices welded onto their neighbor")
	{
		const int F = RC_BORDER_VERTEX;
		const int verts[] = {
			0,  10, 20, F,
			1,  10, 20, F,   // snaps to (0, 20), duplicating the previous vertex
			50, 10, 50, 0,
			0,  10, 60, F,
		};
		rcContourSet* cset = makeContourSet(100, 100, 3, kCellSize, verts, 4);

		REQUIRE(rcWeldContourBorderVertices(&ctx, *cset, 1.5f * kCellSize));

		const rcContour& cont = cset->conts[0];
		REQUIRE(cont.nverts == 3);
		CHECK(cont.verts[0 * 4 + 0] == 0);
		CHECK(cont.verts[0 * 4 + 2] == 20);
		CHECK(cont.verts[1 * 4 + 0] == 50);
		CHECK(cont.verts[2 * 4 + 2] == 60);

		rcFreeContourSet(cset);
	}

	SECTION("leaves single tile contour sets (no border) untouched")
	{
		const int F = RC_BORDER_VERTEX;
		const int verts[] = {
			1,  10, 20, F,
			1,  10, 40, F,
			50, 10, 99, F,
			30, 10, 30, 0,
		};
		rcContourSet* cset = makeContourSet(100, 100, 0, kCellSize, verts, 4);

		REQUIRE(rcWeldContourBorderVertices(&ctx, *cset, 1.5f * kCellSize));

		const rcContour& cont = cset->conts[0];
		REQUIRE(cont.nverts == 4);
		CHECK(cont.verts[0 * 4 + 0] == 1);
		CHECK(cont.verts[2 * 4 + 2] == 99);

		rcFreeContourSet(cset);
	}
}

TEST_CASE("rcBuildContoursEx without welding matches the legacy build path", "[recast][contour-weld]")
{
	rcContext ctx(false);
	std::vector<float> verts;
	std::vector<int> tris;
	float bmin[3], bmax[3];
	buildTestGeometry(verts, tris, bmin, bmax);

	TileBuild legacy;
	REQUIRE(buildTile(ctx, verts, tris, bmin, bmax, 0, 0,
					  RC_CONTOUR_TESS_WALL_EDGES, 0.0f, true, legacy));

	TileBuild extended;
	REQUIRE(buildTile(ctx, verts, tris, bmin, bmax, 0, 0,
					  RC_CONTOUR_TESS_WALL_EDGES, 0.0f, false, extended));

	// The extended entry point with welding disabled must produce exactly the
	// same contours as the existing single-tile build path.
	const rcContourSet& a = *legacy.cset;
	const rcContourSet& b = *extended.cset;
	REQUIRE(a.nconts == b.nconts);
	REQUIRE(a.nconts > 0);
	for (int i = 0; i < a.nconts; ++i)
	{
		REQUIRE(a.conts[i].nverts == b.conts[i].nverts);
		CHECK(memcmp(a.conts[i].verts, b.conts[i].verts, sizeof(int) * a.conts[i].nverts * 4) == 0);
		REQUIRE(a.conts[i].nrverts == b.conts[i].nrverts);
		CHECK(memcmp(a.conts[i].rverts, b.conts[i].rverts, sizeof(int) * a.conts[i].nrverts * 4) == 0);
	}
}

TEST_CASE("Tiled build with border vertex welding connects tile borders", "[recast][contour-weld][detour]")
{
	rcContext ctx(false);
	std::vector<float> verts;
	std::vector<int> tris;
	float bmin[3], bmax[3];
	buildTestGeometry(verts, tris, bmin, bmax);

	// Build the 2x2 tile grid with border vertex welding enabled.
	dtNavMesh navMesh;
	dtNavMeshParams navParams;
	rcVcopy(navParams.orig, bmin);
	navParams.tileWidth = kTileWorld;
	navParams.tileHeight = kTileWorld;
	navParams.maxTiles = 4;
	navParams.maxPolys = 32768;
	REQUIRE(dtStatusSucceed(navMesh.init(&navParams)));

	TileBuild tiles[4];
	dtTileRef tileRefs[4] = {0, 0, 0, 0};
	for (int tz = 0; tz < 2; ++tz)
	{
		for (int tx = 0; tx < 2; ++tx)
		{
			const int idx = tx + tz * 2;
			REQUIRE(buildTile(ctx, verts, tris, bmin, bmax, tx, tz,
							  RC_CONTOUR_TESS_WALL_EDGES | RC_CONTOUR_WELD_BORDER_VERTICES,
							  1.5f * kCellSize, false, tiles[idx]));
			REQUIRE(tiles[idx].pmesh->npolys > 0);

			dtNavMeshCreateParams params{};
			params.verts = tiles[idx].pmesh->verts;
			params.vertCount = tiles[idx].pmesh->nverts;
			params.polys = tiles[idx].pmesh->polys;
			params.polyAreas = tiles[idx].pmesh->areas;
			params.polyFlags = tiles[idx].pmesh->flags;
			params.polyCount = tiles[idx].pmesh->npolys;
			params.nvp = tiles[idx].pmesh->nvp;
			params.detailMeshes = tiles[idx].dmesh->meshes;
			params.detailVerts = tiles[idx].dmesh->verts;
			params.detailVertsCount = tiles[idx].dmesh->nverts;
			params.detailTris = tiles[idx].dmesh->tris;
			params.detailTriCount = tiles[idx].dmesh->ntris;
			params.walkableHeight = 2.0f;
			params.walkableRadius = 0.6f;
			params.walkableClimb = 0.9f;
			params.tileX = tx;
			params.tileY = tz;
			params.tileLayer = 0;
			rcVcopy(params.bmin, tiles[idx].pmesh->bmin);
			rcVcopy(params.bmax, tiles[idx].pmesh->bmax);
			params.cs = kCellSize;
			params.ch = kCellHeight;
			params.buildBvTree = true;

			unsigned char* navData = nullptr;
			int navDataSize = 0;
			REQUIRE(dtCreateNavMeshData(&params, &navData, &navDataSize));
			REQUIRE(dtStatusSucceed(navMesh.addTile(navData, navDataSize, DT_TILE_FREE_DATA, 0, &tileRefs[idx])));
		}
	}

	// Every portal edge on an internal tile border must be connected to the
	// neighboring tile.
	SECTION("all internal portal edges are connected")
	{
		// Sanity check: the welded contour sets must actually contain vertices
		// on the tile border planes, otherwise the welding path is not exercised.
		int borderVertCount = 0;
		for (int idx = 0; idx < 4; ++idx)
		{
			const rcContourSet& cset = *tiles[idx].cset;
			for (int c = 0; c < cset.nconts; ++c)
				for (int v = 0; v < cset.conts[c].nverts; ++v)
				{
					const int x = cset.conts[c].verts[v * 4 + 0];
					const int z = cset.conts[c].verts[v * 4 + 2];
					if (x == 0 || x == cset.width || z == 0 || z == cset.height)
						++borderVertCount;
				}
		}
		REQUIRE(borderVertCount > 0);

		int internalPortalEdges = 0;
		for (int idx = 0; idx < 4; ++idx)
		{
			const int tx = idx % 2;
			const int tz = idx / 2;
			const rcPolyMesh& pm = *tiles[idx].pmesh;
			const int nvp = pm.nvp;
			const dtMeshTile* tile = navMesh.getTileByRef(tileRefs[idx]);
			REQUIRE(tile != nullptr);

			for (int i = 0; i < pm.npolys; ++i)
			{
				const unsigned short* p = &pm.polys[i * 2 * nvp];
				for (int j = 0; j < nvp; ++j)
				{
					if (p[j] == RC_MESH_NULL_IDX)
						break;
					const unsigned short nei = p[nvp + j];
					if ((nei & 0x8000) == 0)
						continue;
					const unsigned short dir = nei & 0xf;
					if (dir == 0xf)
						continue;  // Solid border, not a portal.
					// Is this portal side shared with another tile?
					const bool internal =
						(dir == 0 && tx > 0) ||  // x- portal
						(dir == 2 && tx < 1) ||  // x+ portal
						(dir == 3 && tz > 0) ||  // z- portal
						(dir == 1 && tz < 1);    // z+ portal
					if (!internal)
						continue;

					++internalPortalEdges;

					// The Detour poly must have a link leaving through this edge.
					const dtPoly& poly = tile->polys[i];
					bool connected = false;
					for (unsigned int k = poly.firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
					{
						if (tile->links[k].edge == j)
						{
							connected = true;
							break;
						}
					}
					CHECK(connected);
				}
			}
		}
		// Make sure the check above actually exercised portal edges.
		REQUIRE(internalPortalEdges > 0);
	}

	// Cross-validate with findNearestPoly on both sides of each internal
	// border, then require a path across.
	SECTION("findNearestPoly and paths across the internal borders")
	{
		dtNavMeshQuery query;
		REQUIRE(dtStatusSucceed(query.init(&navMesh, 2048)));
		dtQueryFilter filter;
		const float extents[3] = {1.0f, 2.0f, 1.0f};

		int pairsChecked = 0;

		// Vertical internal border x = kTileWorld. Skip the z-range blocked by
		// the box obstacle (z in [8, 15]).
		for (float z = 2.0f; z < kTileWorld * 2 - 1.0f; z += 3.1f)
		{
			if (z > 7.0f && z < 16.0f)
				continue;
			const float pa[3] = {kTileWorld - 0.3f, 0.2f, z};
			const float pb[3] = {kTileWorld + 0.3f, 0.2f, z};

			dtPolyRef refA = 0, refB = 0;
			REQUIRE(dtStatusSucceed(query.findNearestPoly(pa, extents, &filter, &refA, nullptr)));
			REQUIRE(dtStatusSucceed(query.findNearestPoly(pb, extents, &filter, &refB, nullptr)));
			REQUIRE(refA != 0);
			REQUIRE(refB != 0);

			// Each side must resolve to a polygon of the expected tile.
			const int expectedA = z < kTileWorld ? 0 : 2;  // tiles (0,0) / (0,1)
			const int expectedB = z < kTileWorld ? 1 : 3;  // tiles (1,0) / (1,1)
			CHECK((int)navMesh.decodePolyIdTile(refA) == expectedA);
			CHECK((int)navMesh.decodePolyIdTile(refB) == expectedB);

			// The two polygons must be connected across the border.
			dtPolyRef path[32];
			int npath = 0;
			REQUIRE(dtStatusSucceed(query.findPath(refA, refB, pa, pb, &filter, path, &npath, 32)));
			REQUIRE(npath >= 2);

			++pairsChecked;
		}

		// Horizontal internal border z = kTileWorld.
		for (float x = 2.0f; x < kTileWorld * 2 - 1.0f; x += 3.1f)
		{
			const float pa[3] = {x, 0.2f, kTileWorld - 0.3f};
			const float pb[3] = {x, 0.2f, kTileWorld + 0.3f};

			dtPolyRef refA = 0, refB = 0;
			REQUIRE(dtStatusSucceed(query.findNearestPoly(pa, extents, &filter, &refA, nullptr)));
			REQUIRE(dtStatusSucceed(query.findNearestPoly(pb, extents, &filter, &refB, nullptr)));
			REQUIRE(refA != 0);
			REQUIRE(refB != 0);

			const int expectedA = x < kTileWorld ? 0 : 1;  // tiles (0,0) / (1,0)
			const int expectedB = x < kTileWorld ? 2 : 3;  // tiles (0,1) / (1,1)
			CHECK((int)navMesh.decodePolyIdTile(refA) == expectedA);
			CHECK((int)navMesh.decodePolyIdTile(refB) == expectedB);

			dtPolyRef path[32];
			int npath = 0;
			REQUIRE(dtStatusSucceed(query.findPath(refA, refB, pa, pb, &filter, path, &npath, 32)));
			REQUIRE(npath >= 2);

			++pairsChecked;
		}

		REQUIRE(pairsChecked > 10);
	}

	// An agent must be able to walk from one corner of the map to the
	// opposite corner, crossing both internal borders.
	SECTION("path across the whole 2x2 map")
	{
		dtNavMeshQuery query;
		REQUIRE(dtStatusSucceed(query.init(&navMesh, 2048)));
		dtQueryFilter filter;
		const float extents[3] = {1.0f, 2.0f, 1.0f};

		const float start[3] = {1.0f, 0.2f, 1.0f};
		const float end[3] = {kTileWorld * 2 - 1.0f, 0.2f, kTileWorld * 2 - 1.0f};

		dtPolyRef startRef = 0, endRef = 0;
		REQUIRE(dtStatusSucceed(query.findNearestPoly(start, extents, &filter, &startRef, nullptr)));
		REQUIRE(dtStatusSucceed(query.findNearestPoly(end, extents, &filter, &endRef, nullptr)));
		REQUIRE(startRef != 0);
		REQUIRE(endRef != 0);

		dtPolyRef path[64];
		int npath = 0;
		REQUIRE(dtStatusSucceed(query.findPath(startRef, endRef, start, end, &filter, path, &npath, 64)));
		REQUIRE(npath > 0);

		// The path must span at least two tiles.
		unsigned int tileMask = 0;
		for (int i = 0; i < npath; ++i)
			tileMask |= 1u << navMesh.decodePolyIdTile(path[i]);
		int tileCount = 0;
		for (int i = 0; i < 4; ++i)
			tileCount += (tileMask >> i) & 1;
		CHECK(tileCount >= 2);
	}
}
