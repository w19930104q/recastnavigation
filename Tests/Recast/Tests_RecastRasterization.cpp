#include "Recast.h"
#include "catch2/catch_amalgamated.hpp"

TEST_CASE("rcAddSpan", "[recast][rasterization]")
{
	rcContext ctx(false);

	constexpr int xSize = 4;
	constexpr int ySize = 10;
	constexpr int zSize = 4;

	constexpr float cellSize = 1.0f;
	constexpr float cellHeight = 2.0f;

	constexpr float minBounds[3] {0.0f, 0.0f, 0.0f};
	constexpr float maxBounds[3] {cellSize * xSize, cellHeight * ySize, cellSize * zSize};

	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight));

	constexpr unsigned char area = 42;
	constexpr int flagMergeThr = 1;

	SECTION("Add a span to an empty heightfield.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Adding invalid or zero-size spans does nothing.")
	{
		// min == max
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 0, area, flagMergeThr));
		REQUIRE(hf.spans[0] == nullptr);

		// min > maxs
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 1, 0, area, flagMergeThr));
		REQUIRE(hf.spans[0] == nullptr);
	}

	SECTION("Two spans that are not touching are not merged.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 2, 3, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->next != nullptr);
		REQUIRE(hf.spans[0]->next->smin == 2);
		REQUIRE(hf.spans[0]->next->smax == 3);
		REQUIRE(hf.spans[0]->next->area == area);
		REQUIRE(hf.spans[0]->next->next == nullptr);
	}

	SECTION("Two spans with different area ids within the flag merge threshold are merged and the highest area ID is used.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, 42, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == 42);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 1, 2, 24, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 2);
		REQUIRE(hf.spans[0]->area == 42); // Higher area ID takes precedent
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Two spans with different area ids outside the flag merge threshold are merged and the area ID of the last span added is used.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, 42, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == 42);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 1, 8, 24, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 8);
		REQUIRE(hf.spans[0]->area == 24); // Area ID of the last-added span takes precedent
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Add a span that gets merged with an existing span.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 1, 2, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 2);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Add a span that merges with two spans above and below.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 2, 3, area, flagMergeThr));
		REQUIRE(hf.spans[0]->next != nullptr);
		REQUIRE(hf.spans[0]->next->smin == 2);
		REQUIRE(hf.spans[0]->next->smax == 3);
		REQUIRE(hf.spans[0]->next->area == area);
		REQUIRE(hf.spans[0]->next->next == nullptr);

		// After adding the third span, they should all get merged into a single span.
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 1, 2, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 3);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Spans are insertion-sorted in ascending order of Y value.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 2, 3, area, flagMergeThr));
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, area, flagMergeThr));
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 6, 7, area, flagMergeThr));

		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next != nullptr);

		REQUIRE(hf.spans[0]->next->smin == 2);
		REQUIRE(hf.spans[0]->next->smax == 3);
		REQUIRE(hf.spans[0]->next->area == area);
		REQUIRE(hf.spans[0]->next->next != nullptr);

		REQUIRE(hf.spans[0]->next->next->smin == 6);
		REQUIRE(hf.spans[0]->next->next->smax == 7);
		REQUIRE(hf.spans[0]->next->next->area == area);
		REQUIRE(hf.spans[0]->next->next->next == nullptr);
	}

	SECTION("Adding a span inside another span merges them.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 8, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 8);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 2, 3, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 8);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Overlapping spans are merged.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 4, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 4);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 2, 6, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 6);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);
	}

}

TEST_CASE("allocSpan", "[recast][rasterization]")
{
	rcContext ctx(false);

	constexpr int xSize = 50;
	constexpr int zSize = 50;
	constexpr int ySize = 10;

	constexpr float cellSize = 1.0f;
	constexpr float cellHeight = 2.0f;

	constexpr float minBounds[3] {0.0f, 0.0f, 0.0f};
	constexpr float maxBounds[3] {cellSize * xSize, cellHeight * ySize, cellSize * zSize};

	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight));

	constexpr unsigned char area = 42;
	constexpr int flagMergeThr = 1;

	SECTION("Attempting to add more spans than the span pool size allocates a new page")
	{
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				REQUIRE(rcAddSpan(&ctx, hf, x, z, 0, 1, area, flagMergeThr));
			}
		}
	}
}

TEST_CASE("rcRasterizeTriangle", "[recast][rasterization]")
{
	rcContext ctx(false);

	constexpr int xSize = 10;
	constexpr int ySize = 10;
	constexpr int zSize = 10;

	constexpr float cellSize = 1.0f;
	constexpr float cellHeight = 1.0f;

	constexpr float minBounds[3] {0.0f, 0.0f, 0.0f};
	constexpr float maxBounds[3] {cellSize * xSize, cellHeight * ySize, cellSize * zSize};

	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight));

	SECTION("Simple triangle in XZ plane")
	{
		// Triangle in the XZ plane with vertices (0,0,0), (2,0,0), (0,0,2)
		//
		//   Z ^
		//     |
		//     |\
		//     |_\__> X
		//
		//
		//   Z ....
		//     X...
		//     XX..  X
		//
		// Should fill cells: (0,0), (0,1), (1,0)

		// Clockwise winding order so the normal points in the positive Y
		float v0[] = {0.0f, 0.0f, 0.0f};
		float v1[] = {2.0f, 0.0f, 0.0f};
		float v2[] = {0.0f, 0.0f, 2.0f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if ((x == 0 && z == 0) || (x == 0 && z == 1) || (x == 1 && z == 0))
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Simple triangle inside a single voxel")
	{
		// Clockwise winding order so the normal points in the positive Y
		float v0[] = {0.0f, 0.0f, 0.0f};
		float v1[] = {1.0f, 0.0f, 0.0f};
		float v2[] = {0.0f, 0.0f, 1.0f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if (x == 0 && z == 0)
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Triangles are clipped by the heightfield bounds")
	{
		// When clipped, this should be the same as the triangle in the previous test
		// Should fill cells: (0,0), (0,1), (1,0)

		// Clockwise winding order so the normal points in the positive Y
		float v0[] = {-2.0f, 0.0f, -2.0f};
		float v1[] = {4.0f, 0.0f, -2.0f};
		float v2[] = {-2.0f, 0.0f, 4.0f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if ((x == 0 && z == 0) || (x == 0 && z == 1) || (x == 1 && z == 0))
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Triangle outside of heightfield bounds rasterizes to nothing")
	{
		{ // Outside xz bounds
			float v0[] = {-5.0f, 0.0f, -5.0f};
			float v1[] = {-5.0f, 0.0f, 5.0f};
			float v2[] = {5.0f, 0.0f, -5.0f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

			// Check that no spans were added
			for (int x = 0; x < xSize; x++)
			{
				for (int z = 0; z < zSize; z++)
				{
					REQUIRE(hf.spans[x + z * hf.width] == nullptr);
				}
			}
		}

		{  // below y bounds
			float v0[] = {0.0f, -1.0f, 0.0f};
			float v1[] = {5.0f, -1.0f, 5.0f};
			float v2[] = {5.0f, -1.0f, 0.0f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

			// Check that no spans were added
			for (int x = 0; x < xSize; x++)
			{
				for (int z = 0; z < zSize; z++)
				{
					REQUIRE(hf.spans[x + z * hf.width] == nullptr);
				}
			}
		}

		{  // above y bounds
			float v0[] = {0.0f, 40.0f, 0.0f};
			float v1[] = {5.0f, 40.0f, 5.0f};
			float v2[] = {5.0f, 40.0f, 0.0f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

			// Check that no spans were added
			for (int x = 0; x < xSize; x++)
			{
				for (int z = 0; z < zSize; z++)
				{
					REQUIRE(hf.spans[x + z * hf.width] == nullptr);
				}
			}
		}
	}

	SECTION("Voxels are rasterized if the triangle overlaps any part of it at all")
	{
		// This triangle when clipped barely overlaps the area of the cell at 0,0

		// Clockwise winding order so the normal points in the positive Y
		float v0[] = {-1.0f, 0.0f, -1.0f};
		float v1[] = {1.01f, 0.0f, -1.0f};
		float v2[] = {-1.0f, 0.0f, 1.01f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if (x == 0 && z == 0)
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("A rasterized triangle vertical span includes all the voxels it is in any way part of.")
	{
		// Clockwise winding order so the normal points in the positive Y
		float v0[] = {0.0f, 0.0f, 0.0f};
		float v1[] = {0.5f, 0.0f, 0.5f};
		float v2[] = {0.5f, 2.01f, 0.5f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if (x == 0 && z == 0)
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 3);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Sloped triangle produces varying span heights")
	{
		//   Y ^
		//    2|     /v2
		//     |    /  \
		//    1|   /    \
		//     |  /      \
		//   --+-v0------v1----> X
		//     |  0   2   4
		//
		//   Should rasterize to...
		//
		//   Y ^
		//    2|  |--|--|
		//     |  |     |
		//    1|--|  +  |--|
		//     |           |
		//   --+--------------> X
		//     |
		//

		float v0[] = {0.0f, 0.0f, 0.5f};
		float v1[] = {4.0f, 0.0f, 0.5f};
		float v2[] = {2.0f, 2.0f, 0.5f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if ((x == 0 && z == 0) || (x == 3 && z == 0))
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else if ((x == 1 && z == 0) || (x == 2 && z == 0))
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 2);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Triangle crossing Y bounds gets clamped")
	{
		// This trangle should be clipped to both the max and min Y bounds of the heightfield.
		float v0[] = {0.0f, -5.0f, 0.0f};
		float v1[] = {1.0f, -5.0f, 1.0f};
		float v2[] = {0.5f, 15.0f, 0.5f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if (x == 0 && z == 0)
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 10);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Degenerate triangles rasterize to nothing")
	{
		// SKIP() requires exceptions, but this project is built with -fno-exceptions,
		// so skip the rest of this section manually.
		WARN("SKIP: Currently Recast rasterizes degenerate triangles as if they were a line or point of non-zero volume.");
		return;
		{ // Co-linear points
			float v0[] = {1.0f, 0.0f, 0.5f};
			float v1[] = {2.0f, 0.0f, 0.5f};
			float v2[] = {4.0f, 0.0f, 0.5f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

			// Check that no spans were added
			for (int x = 0; x < xSize; x++)
			{
				for (int z = 0; z < zSize; z++)
				{
					rcSpan* span = hf.spans[x + z * hf.width];
					REQUIRE(span == nullptr);
				}
			}
		}

		{ // All vertices are the same point
			float v0[] = {0.5f, 0.0f, 0.5f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v0, v0, RC_WALKABLE_AREA, hf, 1));

			// Check that no spans were added
			for (int x = 0; x < xSize; x++)
			{
				for (int z = 0; z < zSize; z++)
				{
					rcSpan* span = hf.spans[x + z * hf.width];
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Triangles outside the heightfield with bounding boxes that overlap the heightfield rasterize nothing")
	{
		// This is a minimal repro case for the issue fixed in PR #476 (https://github.com/recastnavigation/recastnavigation/pull/476)
		float v0[] = {-10.0, 5.5, -10.0};
		float v1[] = {-10.0, 5.5, 3};
		float v2[] = {3.0, 5.5, -10.0};
		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that no spans were added
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				REQUIRE(hf.spans[x + z * hf.width] == nullptr);
			}
		}
	}
}

TEST_CASE("rcRasterizeTriangles thin triangles", "[recast][rasterization]")
{
	rcContext ctx(false);

	constexpr int xSize = 10;
	constexpr int ySize = 20;
	constexpr int zSize = 10;

	constexpr float cellSize = 0.3f;
	constexpr float cellHeight = 0.1f;

	constexpr float minBounds[3] {0.0f, 0.0f, 0.0f};
	constexpr float maxBounds[3] {cellSize * xSize, cellHeight * ySize, cellSize * zSize};

	SECTION("Thin diagonal triangle does not lose slivers at cell corners")
	{
		rcHeightfield hf;
		REQUIRE(rcCreateHeightfield(&ctx, hf, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight));

		// Regression test: this ~0.19m wide diagonal triangle (thinner than the 0.3m
		// cell size) crosses cell (3, 2) only through a thin sliver at the cell
		// corner.  At cell resolution the sliver is lost to floating point rounding
		// at the cell boundary, punching a hole in the diagonal chain of spans.
		float verts[] = {
			1.19607139f, 0.955861211f, 1.01206851f,
			1.20293546f, 1.0749774f, 0.816263437f,
			2.31036806f, 1.05162787f, 0.953107834f,
		};
		int tris[] = {0, 1, 2};
		unsigned char areas[] = {RC_WALKABLE_AREA};
		REQUIRE(rcRasterizeTriangles(&ctx, verts, 3, tris, areas, 1, hf, 1));

		// The triangle should produce spans in exactly these cells, with no holes
		// along the diagonal.
		constexpr bool expected[zSize][xSize] = {
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // z = 0
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // z = 1
			{0, 0, 0, 1, 1, 1, 1, 0, 0, 0}, // z = 2, includes the previously missed cell (3, 2)
			{0, 0, 0, 1, 1, 1, 1, 1, 0, 0}, // z = 3
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // z = 4
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // z = 5
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // z = 6
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // z = 7
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // z = 8
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // z = 9
		};

		for (int z = 0; z < zSize; z++)
		{
			for (int x = 0; x < xSize; x++)
			{
				const rcSpan* toplevelSpan = hf.spans[x + z * hf.width];
				REQUIRE((toplevelSpan != nullptr) == (expected[z][x] != 0));
			}
		}

		// The previously missed cell has a span with the expected vertical range.
		const rcSpan* span = hf.spans[3 + 2 * hf.width];
		REQUIRE(span != nullptr);
		REQUIRE(span->smin == 10);
		REQUIRE(span->smax == 11);
		REQUIRE(span->area == RC_WALKABLE_AREA);
		REQUIRE(span->next == nullptr);
	}

	SECTION("Thin diagonal plank rasterizes to a continuous chain of spans")
	{
		rcHeightfield hf;
		REQUIRE(rcCreateHeightfield(&ctx, hf, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight));

		// A 0.15m wide diagonal plank (half the cell size), like a narrow bridge or
		// ramp placed by a level designer.  Two triangles forming a quad from
		// (0.3, 0.3) to (2.7, 2.7).
		constexpr float halfWidth = 0.075f;
		constexpr float perpX = -0.70710678f * halfWidth;
		constexpr float perpZ = 0.70710678f * halfWidth;
		float verts[] = {
			0.3f + perpX, 0.55f, 0.3f + perpZ,
			0.3f - perpX, 0.55f, 0.3f - perpZ,
			2.7f - perpX, 0.55f, 2.7f - perpZ,
			2.7f + perpX, 0.55f, 2.7f + perpZ,
		};
		int tris[] = {0, 1, 2, 0, 2, 3};
		unsigned char areas[] = {RC_WALKABLE_AREA, RC_WALKABLE_AREA};
		REQUIRE(rcRasterizeTriangles(&ctx, verts, 4, tris, areas, 2, hf, 1));

		// Every cell the plank's centerline passes through must have a walkable
		// span - the plank must not have any holes.
		for (float t = 0.0f; t <= 1.0f; t += 0.005f)
		{
			const float p = 0.3f + 2.4f * t;
			const int x = (int)(p / cellSize);
			const int z = (int)(p / cellSize);
			const rcSpan* span = hf.spans[x + z * hf.width];
			REQUIRE(span != nullptr);
			REQUIRE(span->area == RC_WALKABLE_AREA);
		}
	}

	SECTION("Wide triangles are not affected by thin triangle rasterization")
	{
		// Rasterize a wide platform, then add a thin diagonal plank away from it.
		// The platform's spans must be identical before and after.
		float platformVerts[] = {
			0.0f, 0.5f, 0.0f,
			0.9f, 0.5f, 0.0f,
			0.9f, 0.5f, 0.9f,
			0.0f, 0.5f, 0.9f,
		};
		int platformTris[] = {0, 1, 2, 0, 2, 3};
		unsigned char platformAreas[] = {RC_WALKABLE_AREA, RC_WALKABLE_AREA};

		constexpr float halfWidth = 0.075f;
		constexpr float perpX = -0.70710678f * halfWidth;
		constexpr float perpZ = 0.70710678f * halfWidth;
		float plankVerts[] = {
			1.5f + perpX, 0.55f, 1.5f + perpZ,
			1.5f - perpX, 0.55f, 1.5f - perpZ,
			2.7f - perpX, 0.55f, 2.7f - perpZ,
			2.7f + perpX, 0.55f, 2.7f + perpZ,
		};
		int plankTris[] = {0, 1, 2, 0, 2, 3};
		unsigned char plankAreas[] = {RC_WALKABLE_AREA, RC_WALKABLE_AREA};

		rcHeightfield hfPlatformOnly;
		REQUIRE(rcCreateHeightfield(&ctx, hfPlatformOnly, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight));
		REQUIRE(rcRasterizeTriangles(&ctx, platformVerts, 4, platformTris, platformAreas, 2, hfPlatformOnly, 1));

		rcHeightfield hfCombined;
		REQUIRE(rcCreateHeightfield(&ctx, hfCombined, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight));
		REQUIRE(rcRasterizeTriangles(&ctx, platformVerts, 4, platformTris, platformAreas, 2, hfCombined, 1));
		REQUIRE(rcRasterizeTriangles(&ctx, plankVerts, 4, plankTris, plankAreas, 2, hfCombined, 1));

		int platformSpanCount = 0;
		int combinedSpanCount = 0;
		for (int z = 0; z < zSize; z++)
		{
			for (int x = 0; x < xSize; x++)
			{
				// The platform's spans are unchanged by the addition of the plank.
				const rcSpan* spanA = hfPlatformOnly.spans[x + z * hfPlatformOnly.width];
				const rcSpan* spanB = hfCombined.spans[x + z * hfCombined.width];
				if (x < 3 && z < 3)
				{
					REQUIRE(spanA != nullptr);
					REQUIRE(spanB != nullptr);
					REQUIRE(spanA->smin == spanB->smin);
					REQUIRE(spanA->smax == spanB->smax);
					REQUIRE(spanA->area == spanB->area);
					REQUIRE((spanA->next == nullptr) == (spanB->next == nullptr));
				}
				for (const rcSpan* s = spanA; s; s = s->next)
				{
					platformSpanCount++;
				}
				for (const rcSpan* s = spanB; s; s = s->next)
				{
					combinedSpanCount++;
				}
			}
		}

		// The platform rasterizes to exactly one span in each of its 9 cells, and
		// the plank only adds spans on top of that.
		REQUIRE(platformSpanCount == 9);
		REQUIRE(combinedSpanCount > platformSpanCount);
	}
}
