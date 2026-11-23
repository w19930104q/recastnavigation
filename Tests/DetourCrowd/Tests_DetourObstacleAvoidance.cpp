#include "DetourObstacleAvoidance.h"
#include "DetourCommon.h"
#include "catch2/catch_amalgamated.hpp"

#include <cmath>

namespace
{

dtObstacleAvoidanceParams makeCrowdDefaultParams()
{
	// Same defaults dtCrowd uses for its obstacle avoidance configurations.
	dtObstacleAvoidanceParams p;
	p.velBias = 0.4f;
	p.weightDesVel = 2.0f;
	p.weightCurVel = 0.75f;
	p.weightSide = 0.75f;
	p.weightToi = 2.5f;
	p.horizTime = 2.5f;
	p.gridSize = 33;
	p.adaptiveDivs = 7;
	p.adaptiveRings = 2;
	p.adaptiveDepth = 5;
	return p;
}

float length2D(const float* v)
{
	return std::sqrt(v[0] * v[0] + v[2] * v[2]);
}

float dist2D(const float* a, const float* b)
{
	const float dx = a[0] - b[0];
	const float dz = a[2] - b[2];
	return std::sqrt(dx * dx + dz * dz);
}

bool isFiniteVel(const float* v)
{
	return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

} // namespace

TEST_CASE("dtCalcAdaptiveSamplingResolution adapts to agent radius")
{
	SECTION("Angular resolution increases for small agents, decreases for large agents")
	{
		int divsSmall, ringsSmall, depthSmall;
		int divsMid, ringsMid, depthMid;
		int divsLarge, ringsLarge, depthLarge;

		dtCalcAdaptiveSamplingResolution(0.3f, 8, DT_DEFAULT_AVOIDANCE_SAMPLE_BUDGET,
										 divsSmall, ringsSmall, depthSmall);
		dtCalcAdaptiveSamplingResolution(0.5f, 8, DT_DEFAULT_AVOIDANCE_SAMPLE_BUDGET,
										 divsMid, ringsMid, depthMid);
		dtCalcAdaptiveSamplingResolution(2.0f, 8, DT_DEFAULT_AVOIDANCE_SAMPLE_BUDGET,
										 divsLarge, ringsLarge, depthLarge);

		// Small agents need finer angular steps to resolve narrow gaps.
		CHECK(divsSmall > divsMid);
		CHECK(divsMid > divsLarge);

		// Large agents instead get wider speed-range coverage (more rings).
		CHECK(ringsLarge > ringsSmall);

		// Everything stays within the pattern limits.
		CHECK(divsSmall <= DT_MAX_PATTERN_DIVS);
		CHECK(ringsLarge <= DT_MAX_PATTERN_RINGS);
		CHECK(divsLarge >= 1);
		CHECK(depthSmall >= 1);
		CHECK(depthLarge >= 1);
	}

	SECTION("Density scaling is sub-linear and budget bounded")
	{
		// Locked-in reference values for the default budget in an average scene.
		int divs, rings, depth;
		dtCalcAdaptiveSamplingResolution(0.5f, 8, 256, divs, rings, depth);
		CHECK(divs == 9);
		CHECK(rings == 2);
		CHECK((divs * rings + 1) * depth <= 256);

		// A 6.7x smaller agent must not get 6.7x the samples (sub-linear scaling).
		int divsTiny, ringsTiny, depthTiny;
		dtCalcAdaptiveSamplingResolution(0.3f, 8, 256, divsTiny, ringsTiny, depthTiny);
		const int totalMid = (divs * rings + 1) * depth;
		const int totalTiny = (divsTiny * ringsTiny + 1) * depthTiny;
		CHECK(totalTiny <= 2 * totalMid);

		// A tiny budget must shrink the total sample count.
		int divsCheap, ringsCheap, depthCheap;
		dtCalcAdaptiveSamplingResolution(0.5f, 8, 32, divsCheap, ringsCheap, depthCheap);
		CHECK((divsCheap * ringsCheap + 1) * depthCheap < totalMid);

		// A more complex scene may spend more samples, but stays bounded.
		int divsBusy, ringsBusy, depthBusy;
		dtCalcAdaptiveSamplingResolution(0.5f, 128, 256, divsBusy, ringsBusy, depthBusy);
		CHECK((divsBusy * ringsBusy + 1) * depthBusy <= 2 * 256);
	}
}

TEST_CASE("dtObstacleAvoidanceQuery radius-aware sampling produces valid velocities")
{
	dtObstacleAvoidanceParams params = makeCrowdDefaultParams();

	dtObstacleAvoidanceQuery* query = dtAllocObstacleAvoidanceQuery();
	REQUIRE(query != nullptr);
	REQUIRE(query->init(16, 16));

	SECTION("No obstacles: radius-aware sampling returns the desired velocity")
	{
		const float pos[3] = { 0.0f, 0.0f, 0.0f };
		const float vel[3] = { 0.0f, 0.0f, 0.0f };
		const float dvel[3] = { 0.0f, 0.0f, 2.0f };
		float nvel[3] = { 0.0f, 0.0f, 0.0f };

		const int ns = query->sampleVelocityAdaptiveRadiusAware(pos, 0.3f, 2.0f, vel, dvel, nvel, &params);
		CHECK(ns > 0);
		CHECK(isFiniteVel(nvel));
		CHECK(nvel[0] == Catch::Approx(0.0f).margin(1e-4f));
		CHECK(nvel[2] == Catch::Approx(2.0f).margin(1e-4f));
	}

	SECTION("Default adaptive sampling entry point still works as before")
	{
		const float pos[3] = { 0.0f, 0.0f, 0.0f };
		const float vel[3] = { 0.0f, 0.0f, 0.0f };
		const float dvel[3] = { 0.0f, 0.0f, 2.0f };
		float nvel[3] = { 0.0f, 0.0f, 0.0f };

		const int ns = query->sampleVelocityAdaptive(pos, 0.5f, 2.0f, vel, dvel, nvel, &params);
		CHECK(ns > 0);
		CHECK(isFiniteVel(nvel));
		CHECK(length2D(nvel) <= 2.0f + 1e-3f);
		CHECK(nvel[0] == Catch::Approx(0.0f).margin(1e-4f));
		CHECK(nvel[2] == Catch::Approx(2.0f).margin(1e-4f));
	}

	dtFreeObstacleAvoidanceQuery(query);
}

namespace
{

// A gate scenario: two wall segments along z = wallZ with two circular posts
// leaving a 1.2m clear gap. A 0.3m radius agent (0.6m diameter) fits through,
// a 2.0m radius agent (4.0m diameter) physically cannot.
struct GateScenario
{
	static const float wallZ;
	static const float postX;
	static const float postRad;
	static const float wallHalfLen;

	static void addObstacles(dtObstacleAvoidanceQuery* query)
	{
		const float zero[3] = { 0.0f, 0.0f, 0.0f };
		const float postA[3] = { -postX, 0.0f, wallZ };
		const float postB[3] = { postX, 0.0f, wallZ };
		query->addCircle(postA, postRad, zero, zero);
		query->addCircle(postB, postRad, zero, zero);
		const float wallA0[3] = { -wallHalfLen, 0.0f, wallZ };
		const float wallA1[3] = { -postX, 0.0f, wallZ };
		const float wallB0[3] = { postX, 0.0f, wallZ };
		const float wallB1[3] = { wallHalfLen, 0.0f, wallZ };
		query->addSegment(wallA0, wallA1);
		query->addSegment(wallB0, wallB1);
	}
};

const float GateScenario::wallZ = 5.0f;
const float GateScenario::postX = 0.9f;
const float GateScenario::postRad = 0.3f;
const float GateScenario::wallHalfLen = 8.0f;

struct SimResult
{
	bool crossed;			// agent made it through the gate
	bool velocitiesValid;	// every nvel finite and within vmax
	bool postCollision;		// agent penetrated a post
	bool wallCrossed;		// agent crossed a wall segment away from the gate
	float minPostClearance;	// smallest (dist to post center - postRad) seen
	int steps;
};

SimResult simulateAgent(const float rad, const float* start, const float* goal,
						const int maxSteps, const float dt)
{
	const float vmax = 2.0f;
	dtObstacleAvoidanceParams params = makeCrowdDefaultParams();

	dtObstacleAvoidanceQuery* query = dtAllocObstacleAvoidanceQuery();
	REQUIRE(query != nullptr);
	REQUIRE(query->init(16, 16));

	SimResult res;
	res.crossed = false;
	res.velocitiesValid = true;
	res.postCollision = false;
	res.wallCrossed = false;
	res.minPostClearance = 1e30f;
	res.steps = 0;

	float pos[3] = { start[0], start[1], start[2] };
	float vel[3] = { 0.0f, 0.0f, 0.0f };

	for (int step = 0; step < maxSteps; ++step)
	{
		res.steps = step + 1;

		// Desired velocity points at the goal.
		float dvel[3];
		dvel[0] = goal[0] - pos[0];
		dvel[1] = 0.0f;
		dvel[2] = goal[2] - pos[2];
		const float dlen = length2D(dvel);
		if (dlen < 0.3f)
			break; // reached the goal
		dvel[0] = dvel[0] / dlen * vmax;
		dvel[2] = dvel[2] / dlen * vmax;

		query->reset();
		GateScenario::addObstacles(query);

		float nvel[3] = { 0.0f, 0.0f, 0.0f };
		query->sampleVelocityAdaptiveRadiusAware(pos, rad, vmax, vel, dvel, nvel, &params);

		if (!isFiniteVel(nvel) || length2D(nvel) > vmax + 0.01f)
			res.velocitiesValid = false;

		pos[0] += nvel[0] * dt;
		pos[2] += nvel[2] * dt;
		vel[0] = nvel[0];
		vel[2] = nvel[2];

		// Check clearance against the gate posts.
		const float postA[3] = { -GateScenario::postX, 0.0f, GateScenario::wallZ };
		const float postB[3] = { GateScenario::postX, 0.0f, GateScenario::wallZ };
		const float clearA = dist2D(pos, postA) - GateScenario::postRad;
		const float clearB = dist2D(pos, postB) - GateScenario::postRad;
		res.minPostClearance = dtMin(res.minPostClearance, dtMin(clearA, clearB));
		// Velocity-level avoidance does not guarantee zero penetration (dtCrowd
		// resolves residual overlaps with a separate collision pass); tolerate a
		// shallow transient overlap of up to 10% of the agent radius.
		if (res.minPostClearance < rad * 0.9f)
			res.postCollision = true;

		// The agent must never end up beyond the wall line while in front of a wall segment.
		if (pos[2] > GateScenario::wallZ + 1e-3f && std::fabs(pos[0]) > GateScenario::postX &&
			std::fabs(pos[0]) < GateScenario::wallHalfLen)
			res.wallCrossed = true;

		if (pos[2] > GateScenario::wallZ + 0.5f)
		{
			res.crossed = true;
			break;
		}
	}

	dtFreeObstacleAvoidanceQuery(query);
	return res;
}

} // namespace

TEST_CASE("Radius-aware avoidance with mixed agent radii at a narrow gate")
{
	const float dt = 0.1f;
	const int maxSteps = 300; // 30 seconds of simulated time
	const float goal[3] = { 0.0f, 0.0f, 8.0f };

	SECTION("Small agent (r=0.3) threads the gap without collisions")
	{
		const float start[3] = { 1.5f, 0.0f, 0.0f };
		const SimResult res = simulateAgent(0.3f, start, goal, maxSteps, dt);

		CHECK(res.velocitiesValid);
		CHECK(res.crossed);
		CHECK_FALSE(res.postCollision);
		CHECK_FALSE(res.wallCrossed);
	}

	SECTION("Large agent (r=2.0) cannot pass but still gets safe velocities")
	{
		const float start[3] = { 1.5f, 0.0f, 0.0f };
		const SimResult res = simulateAgent(2.0f, start, goal, maxSteps, dt);

		CAPTURE(res.minPostClearance);
		CHECK(res.velocitiesValid);
		CHECK_FALSE(res.crossed);
		CHECK_FALSE(res.postCollision);
		CHECK_FALSE(res.wallCrossed);
	}
}

TEST_CASE("Radius-aware sampling cost stays within the sample budget")
{
	dtObstacleAvoidanceParams params = makeCrowdDefaultParams();

	dtObstacleAvoidanceQuery* query = dtAllocObstacleAvoidanceQuery();
	REQUIRE(query != nullptr);
	REQUIRE(query->init(16, 16));

	const float pos[3] = { 1.5f, 0.0f, 0.0f };
	const float vel[3] = { 0.0f, 0.0f, 0.0f };
	const float dvel[3] = { 0.0f, 0.0f, 2.0f };
	float nvel[3];

	query->reset();
	GateScenario::addObstacles(query);
	const int obstacleCount = query->getObstacleCircleCount() + query->getObstacleSegmentCount();

	const int nsSmall = query->sampleVelocityAdaptiveRadiusAware(pos, 0.3f, 2.0f, vel, dvel, nvel, &params);
	CHECK(isFiniteVel(nvel));

	const int nsLarge = query->sampleVelocityAdaptiveRadiusAware(pos, 2.0f, 2.0f, vel, dvel, nvel, &params);
	CHECK(isFiniteVel(nvel));

	int divs, rings, depth;
	dtCalcAdaptiveSamplingResolution(0.3f, obstacleCount, DT_DEFAULT_AVOIDANCE_SAMPLE_BUDGET,
									 divs, rings, depth);
	CHECK(nsSmall <= (divs * rings + 1) * depth);
	CHECK(nsSmall > 0);

	dtCalcAdaptiveSamplingResolution(2.0f, obstacleCount, DT_DEFAULT_AVOIDANCE_SAMPLE_BUDGET,
									 divs, rings, depth);
	CHECK(nsLarge <= (divs * rings + 1) * depth);
	CHECK(nsLarge > 0);

	dtFreeObstacleAvoidanceQuery(query);
}
