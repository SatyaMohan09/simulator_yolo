package com.evtol.trajectoryengine.service;

import com.evtol.trajectoryengine.domain.TrajectoryModel;
import com.evtol.trajectoryengine.domain.TrajectoryPoint;
import com.evtol.trajectoryengine.domain.Waypoint;

import java.util.List;

/**
 * Holds the result of the last full trajectory plan (post-RRT*, pre-bird).
 *
 * <p>This cache exists so that bird-only replans can skip every expensive
 * step that does not need to change when only dynamic obstacles move:</p>
 * <ul>
 *   <li>CSV load           — waypoints never change at runtime</li>
 *   <li>RRT*               — only re-runs when static buildings change</li>
 *   <li>LeastSquaresFitter — depends only on waypoints, not on birds</li>
 *   <li>BSplineCurveBuilder — same dependency</li>
 *   <li>Full resample      — replaced by a partial splice for bird detours</li>
 * </ul>
 *
 * <p>All fields are intentionally immutable after construction.
 * The cache is replaced atomically as a whole unit so readers always
 * see a consistent snapshot.</p>
 */
public final class TrajectoryCache {

    /** Waypoints after RRT* static-obstacle avoidance (birds not yet applied). */
    private final List<Waypoint> baseWaypoints;

    /** Control points produced by LeastSquaresFitter from baseWaypoints. */
    private final List<Waypoint> controlPoints;

    /** B-spline model built from controlPoints. */
    private final TrajectoryModel trajectoryModel;

    /**
     * Fully sampled trajectory produced from trajectoryModel with no bird
     * avoidance applied.  Bird replans splice a local detour into a copy
     * of this list rather than resampling from scratch.
     */
    private final List<TrajectoryPoint> baseTrajectoryPoints;

    /** Lambda value used when this cache was built. */
    private final double lambda;

    public TrajectoryCache(
            List<Waypoint>       baseWaypoints,
            List<Waypoint>       controlPoints,
            TrajectoryModel      trajectoryModel,
            List<TrajectoryPoint> baseTrajectoryPoints,
            double               lambda) {

        this.baseWaypoints         = List.copyOf(baseWaypoints);
        this.controlPoints         = List.copyOf(controlPoints);
        this.trajectoryModel       = trajectoryModel;
        this.baseTrajectoryPoints  = List.copyOf(baseTrajectoryPoints);
        this.lambda                = lambda;
    }

    public List<Waypoint>       getBaseWaypoints()        { return baseWaypoints; }
    public List<Waypoint>       getControlPoints()        { return controlPoints; }
    public TrajectoryModel      getTrajectoryModel()      { return trajectoryModel; }
    public List<TrajectoryPoint> getBaseTrajectoryPoints() { return baseTrajectoryPoints; }
    public double               getLambda()               { return lambda; }
}