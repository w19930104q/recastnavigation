//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef DETOUROBSTACLEAVOIDANCE_H
#define DETOUROBSTACLEAVOIDANCE_H

struct dtObstacleCircle
{
	float p[3];				///< Position of the obstacle
	float vel[3];			///< Velocity of the obstacle
	float dvel[3];			///< Velocity of the obstacle
	float rad;				///< Radius of the obstacle
	float dp[3], np[3];		///< Use for side selection during sampling.
};

struct dtObstacleSegment
{
	float p[3], q[3];		///< End points of the obstacle segment
	bool touch;
};


class dtObstacleAvoidanceDebugData
{
public:
	dtObstacleAvoidanceDebugData();
	~dtObstacleAvoidanceDebugData();
	
	bool init(const int maxSamples);
	void reset();
	void addSample(const float* vel, const float ssize, const float pen,
				   const float vpen, const float vcpen, const float spen, const float tpen);
	
	void normalizeSamples();
	
	inline int getSampleCount() const { return m_nsamples; }
	inline const float* getSampleVelocity(const int i) const { return &m_vel[i*3]; }
	inline float getSampleSize(const int i) const { return m_ssize[i]; }
	inline float getSamplePenalty(const int i) const { return m_pen[i]; }
	inline float getSampleDesiredVelocityPenalty(const int i) const { return m_vpen[i]; }
	inline float getSampleCurrentVelocityPenalty(const int i) const { return m_vcpen[i]; }
	inline float getSamplePreferredSidePenalty(const int i) const { return m_spen[i]; }
	inline float getSampleCollisionTimePenalty(const int i) const { return m_tpen[i]; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtObstacleAvoidanceDebugData(const dtObstacleAvoidanceDebugData&);
	dtObstacleAvoidanceDebugData& operator=(const dtObstacleAvoidanceDebugData&);

	int m_nsamples;
	int m_maxSamples;
	float* m_vel;
	float* m_ssize;
	float* m_pen;
	float* m_vpen;
	float* m_vcpen;
	float* m_spen;
	float* m_tpen;
};

dtObstacleAvoidanceDebugData* dtAllocObstacleAvoidanceDebugData();
void dtFreeObstacleAvoidanceDebugData(dtObstacleAvoidanceDebugData* ptr);


static const int DT_MAX_PATTERN_DIVS = 32;	///< Max numver of adaptive divs.
static const int DT_MAX_PATTERN_RINGS = 4;	///< Max number of adaptive rings.

struct dtObstacleAvoidanceParams
{
	float velBias;
	float weightDesVel;
	float weightCurVel;
	float weightSide;
	float weightToi;
	float horizTime;
	unsigned char gridSize;	///< grid
	unsigned char adaptiveDivs;	///< adaptive
	unsigned char adaptiveRings;	///< adaptive
	unsigned char adaptiveDepth;	///< adaptive
};

/// Default per-query sample budget used by radius-aware adaptive sampling.
static const int DT_DEFAULT_AVOIDANCE_SAMPLE_BUDGET = 256;

/// Derives adaptive sampling resolution (divs/rings/depth) from the agent radius and
/// the number of obstacles in the query, bounded by a total sample budget.
///
/// The mapping is deliberately sub-linear so that CPU cost stays bounded:
///  - Angular divs scale with 1/sqrt(radius): small agents get finer angular
///    resolution to thread narrow gaps, large agents get coarser steps.
///  - Speed rings scale with sqrt(radius): large agents get wider speed-range
///    coverage instead of angular density.
///  - The sample budget scales with sqrt(obstacleCount) (clamped), and whatever
///    budget remains after divs/rings are chosen becomes the refinement depth.
///
/// @param[in]	agentRadius		Radius of the agent.
/// @param[in]	obstacleCount	Number of obstacles currently in the query (circles + segments).
/// @param[in]	sampleBudget	Soft cap on total samples per query (divs*rings+1)*depth.
/// @param[out]	outDivs			Angular divisions for the adaptive pattern.
/// @param[out]	outRings		Speed rings for the adaptive pattern.
/// @param[out]	outDepth		Refinement depth for the adaptive pattern.
void dtCalcAdaptiveSamplingResolution(const float agentRadius, const int obstacleCount,
									  const int sampleBudget,
									  int& outDivs, int& outRings, int& outDepth);

class dtObstacleAvoidanceQuery
{
public:
	dtObstacleAvoidanceQuery();
	~dtObstacleAvoidanceQuery();
	
	bool init(const int maxCircles, const int maxSegments);
	
	void reset();

	void addCircle(const float* pos, const float rad,
				   const float* vel, const float* dvel);
				   
	void addSegment(const float* p, const float* q);

	int sampleVelocityGrid(const float* pos, const float rad, const float vmax,
						   const float* vel, const float* dvel, float* nvel,
						   const dtObstacleAvoidanceParams* params,
						   dtObstacleAvoidanceDebugData* debug = 0);

	int sampleVelocityAdaptive(const float* pos, const float rad, const float vmax,
							   const float* vel, const float* dvel, float* nvel,
							   const dtObstacleAvoidanceParams* params, 
							   dtObstacleAvoidanceDebugData* debug = 0);

	/// Adaptive velocity sampling with a sampling pattern derived from the agent
	/// radius and the current obstacle count (see dtCalcAdaptiveSamplingResolution).
	///
	/// Small agents get denser angular sampling so they can find safe velocities
	/// through narrow gaps; large agents get wider speed-range coverage. The total
	/// sample count is bounded by @p sampleBudget to keep per-frame CPU cost in check.
	///
	/// The @p params weights/horizTime are honored as usual; only the sampling
	/// resolution (divs/rings/depth) is derived instead of taken from the params.
	int sampleVelocityAdaptiveRadiusAware(const float* pos, const float rad, const float vmax,
										  const float* vel, const float* dvel, float* nvel,
										  const dtObstacleAvoidanceParams* params,
										  const int sampleBudget = DT_DEFAULT_AVOIDANCE_SAMPLE_BUDGET,
										  dtObstacleAvoidanceDebugData* debug = 0);

	inline int getObstacleCircleCount() const { return m_ncircles; }
	const dtObstacleCircle* getObstacleCircle(const int i) { return &m_circles[i]; }

	inline int getObstacleSegmentCount() const { return m_nsegments; }
	const dtObstacleSegment* getObstacleSegment(const int i) { return &m_segments[i]; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtObstacleAvoidanceQuery(const dtObstacleAvoidanceQuery&);
	dtObstacleAvoidanceQuery& operator=(const dtObstacleAvoidanceQuery&);

	void prepare(const float* pos, const float* dvel);

	int sampleVelocityAdaptiveImpl(const float* pos, const float rad, const float vmax,
								   const float* vel, const float* dvel, float* nvel,
								   const dtObstacleAvoidanceParams* params,
								   const int ndivs, const int nrings, const int depth,
								   dtObstacleAvoidanceDebugData* debug);

	float processSample(const float* vcand, const float cs,
						const float* pos, const float rad,
						const float* vel, const float* dvel,
						const float minPenalty,
						dtObstacleAvoidanceDebugData* debug);

	dtObstacleAvoidanceParams m_params;
	float m_invHorizTime;
	float m_vmax;
	float m_invVmax;

	int m_maxCircles;
	dtObstacleCircle* m_circles;
	int m_ncircles;

	int m_maxSegments;
	dtObstacleSegment* m_segments;
	int m_nsegments;
};

dtObstacleAvoidanceQuery* dtAllocObstacleAvoidanceQuery();
void dtFreeObstacleAvoidanceQuery(dtObstacleAvoidanceQuery* ptr);


#endif // DETOUROBSTACLEAVOIDANCE_H
