package com.evtol.trajectoryengine.service;

import com.evtol.trajectoryengine.bspline.BSplineCurveBuilder;
import com.evtol.trajectoryengine.datasource.CsvWaypointDataProvider;
import com.evtol.trajectoryengine.domain.Obstacle;
import com.evtol.trajectoryengine.domain.TrajectoryModel;
import com.evtol.trajectoryengine.domain.TrajectoryPoint;
import com.evtol.trajectoryengine.domain.Waypoint;
import com.evtol.trajectoryengine.dto.TrajectoryResponse;
import com.evtol.trajectoryengine.fitting.LeastSquaresFitter;
import com.evtol.trajectoryengine.planning.DStarLitePlanner;
import com.evtol.trajectoryengine.planning.RrtStarPlanner;
import com.evtol.trajectoryengine.spline.CubicSplineBuilder;
import com.evtol.trajectoryengine.validation.WaypointValidator;
import lombok.RequiredArgsConstructor;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

import java.util.ArrayList;
import java.util.List;

@Service
@RequiredArgsConstructor
public class TrajectoryService {

    private final CsvWaypointDataProvider dataProvider;
    private final ObstacleRegistryService obstacleRegistryService;
    private final WaypointValidator       validator;
    private final CubicSplineBuilder      cubicSplineBuilder;
    private final BSplineCurveBuilder     bSplineCurveBuilder;
    private final SamplingService         samplingService;
    private final LeastSquaresFitter      leastSquaresFitter;
    private final RrtStarPlanner          rrtStarPlanner;
    private final DStarLitePlanner        dStarLitePlanner;

    @Value("${trajectory.sampling.interval}")
    private double samplingInterval;

    @Value("${trajectory.algorithm}")
    private String algorithm;

    private static final double MAX_TURN_ANGLE_RAD = Math.toRadians(150.0);

    /**
     * Cached result of the last full replan.
     * Replaced atomically on every full replan.
     * Read by bird-only replans to skip all expensive upstream steps.
     */
    private volatile TrajectoryCache trajectoryCache = null;

    // ── public API ────────────────────────────────────────────────────────────

    public TrajectoryResponse generateTrajectory(double lambda) {
        return generateTrajectory(lambda, List.of());
    }

    /**
     * Full replan — runs every stage of the pipeline and refreshes the cache.
     * Called when static obstacles change or on first startup.
     */
    public TrajectoryResponse generateTrajectory(double lambda,
                                                  List<Obstacle> requestObstacles) {

        // 1. Load raw waypoints from CSV
        List<Waypoint> rawWaypoints    = dataProvider.loadWaypoints();

        // 2. Load obstacles
        List<Obstacle> staticObstacles = obstacleRegistryService
                .loadStaticPlannerObstacles(requestObstacles);
        List<Obstacle> birdObstacles   = obstacleRegistryService.loadBirdObstacles();

        // 3. RRT* — static obstacle avoidance
        List<Waypoint> afterRrt = applyRrtAvoidance(rawWaypoints, staticObstacles);

        // 4. Fit B-spline to post-RRT* waypoints (birds not yet considered)
        //    This is the expensive step we want to cache and reuse.
        validator.validate(afterRrt);
        List<Waypoint>  controlPoints   = leastSquaresFitter.fit(afterRrt);
        TrajectoryModel trajectoryModel = buildModel(controlPoints, lambda);

        // 5. Sample the base trajectory (no bird avoidance yet)
        List<TrajectoryPoint> basePoints =
                samplingService.sample(trajectoryModel, samplingInterval);

        // 6. Store everything in cache before applying bird avoidance.
        //    Bird replans start from here and skip steps 1–5.
        trajectoryCache = new TrajectoryCache(
                afterRrt, controlPoints, trajectoryModel, basePoints, lambda);

        // 7. D* Lite — bird avoidance applied on top of the cached base
        List<Waypoint> afterDStar = applyDStarAvoidance(afterRrt, birdObstacles);

        // 8. If D* Lite made no change, the base points are the final answer —
        //    no need to resample.
        List<TrajectoryPoint> finalPoints;
        if (afterDStar == afterRrt) {
            finalPoints = basePoints;
        } else {
            // D* Lite modified some waypoints — rebuild only the affected window.
            finalPoints = applyBirdPatch(afterDStar, afterRrt,
                                          trajectoryModel, basePoints);
        }

        return new TrajectoryResponse(
                finalPoints, afterDStar, controlPoints,
                trajectoryModel.getTotalDuration());
    }

    /**
     * Bird-only partial replan — skips CSV load, RRT*, LeastSquaresFitter,
     * and BSplineCurveBuilder.  Uses the cached model and only re-runs
     * D* Lite, then splices the changed segment back into the cached points.
     *
     * Falls back to a full replan if no cache is available yet.
     */
    public TrajectoryResponse generateBirdAvoidanceTrajectory(double lambda) {
        TrajectoryCache cache = trajectoryCache;

        if (cache == null) {
            System.out.println("[TrajectoryService] No cache yet — " +
                               "falling back to full replan.");
            return generateTrajectory(lambda);
        }

        List<Obstacle> birdObstacles = obstacleRegistryService.loadBirdObstacles();

        // D* Lite only — runs against the cached base waypoints
        List<Waypoint> afterDStar =
                applyDStarAvoidance(cache.getBaseWaypoints(), birdObstacles);

        // If nothing changed, return the cached base trajectory as-is.
        // No resampling, no B-spline work at all.
        if (afterDStar == cache.getBaseWaypoints()) {
            System.out.println("[TrajectoryService] Bird replan: no change detected, " +
                               "returning cached trajectory.");
            return new TrajectoryResponse(
                    cache.getBaseTrajectoryPoints(),
                    cache.getBaseWaypoints(),
                    cache.getControlPoints(),
                    cache.getTrajectoryModel().getTotalDuration());
        }

        // D* Lite changed something — splice only the affected window.
        List<TrajectoryPoint> finalPoints = applyBirdPatch(
                afterDStar,
                cache.getBaseWaypoints(),
                cache.getTrajectoryModel(),
                cache.getBaseTrajectoryPoints());

        return new TrajectoryResponse(
                finalPoints,
                afterDStar,
                cache.getControlPoints(),
                cache.getTrajectoryModel().getTotalDuration());
    }

    // ── private helpers ───────────────────────────────────────────────────────

    /**
     * Finds the time window affected by D* Lite by comparing the modified
     * waypoints against the original, then resamples only that window and
     * splices the result back into the base trajectory.
     */
    private List<TrajectoryPoint> applyBirdPatch(
            List<Waypoint>        modified,
            List<Waypoint>        original,
            TrajectoryModel       model,
            List<TrajectoryPoint> basePoints) {

        // Find the first and last waypoint index that actually changed.
        int firstChanged = -1;
        int lastChanged  = -1;

        int len = Math.min(modified.size(), original.size());
        for (int i = 0; i < len; i++) {
            if (waypointDiffers(modified.get(i), original.get(i))) {
                if (firstChanged == -1) firstChanged = i;
                lastChanged = i;
            }
        }

        // No difference found — return base unchanged.
        if (firstChanged == -1) {
            return basePoints;
        }

        // Convert waypoint indices to mission time for the sampling window.
        // Add a small margin so the splice joins smoothly.
        double windowStart = modified.get(Math.max(0, firstChanged - 1)).getT();
        double windowEnd   = modified.get(
                Math.min(modified.size() - 1, lastChanged + 1)).getT();

        System.out.printf("[TrajectoryService] Bird patch: resampling window " +
                          "t=%.2f → t=%.2f (waypoints %d–%d)%n",
                          windowStart, windowEnd, firstChanged, lastChanged);

        // Rebuild a temporary model for just the affected waypoints so the
        // B-spline reflects the D* Lite detour geometry in that window.
        List<Waypoint> windowWaypoints = modified.subList(
                Math.max(0, firstChanged - 2),
                Math.min(modified.size(), lastChanged + 3));

        // Only refit the B-spline for the window segment if it has enough points.
        // Fall back to the cached model if the window is too small.
        TrajectoryModel windowModel = model;
        if (windowWaypoints.size() >= 4) {
            try {
                List<Waypoint> windowControlPoints =
                        leastSquaresFitter.fit(new ArrayList<>(windowWaypoints));
                windowModel = buildModel(windowControlPoints, trajectoryCache.getLambda());
            } catch (Exception e) {
                System.out.printf("[TrajectoryService] Window refit failed (%s), " +
                                  "using cached model for patch.%n", e.getMessage());
                windowModel = model;
            }
        }

        // Resample only the affected window using the window model.
        List<TrajectoryPoint> patch = samplingService.sampleWindow(
                windowModel, samplingInterval, windowStart, windowEnd);

        // Splice the patch back into the full trajectory.
        return samplingService.splice(basePoints, patch, windowStart, windowEnd);
    }

    private boolean waypointDiffers(Waypoint a, Waypoint b) {
        return Math.abs(a.getX() - b.getX()) > 0.01
            || Math.abs(a.getY() - b.getY()) > 0.01
            || Math.abs(a.getZ() - b.getZ()) > 0.01;
    }

    private List<Waypoint> applyRrtAvoidance(List<Waypoint> source,
                                              List<Obstacle> staticObstacles) {
        if (source == null || source.size() < 3) {
            System.out.println("[RRT] Skipped — too few waypoints");
            return source;
        }
        if (staticObstacles.isEmpty()) {
            System.out.println("[RRT] Skipped — no static obstacles");
            return source;
        }
        System.out.printf("[RRT] Planning around %d static obstacle(s)%n",
                staticObstacles.size());
        try {
            List<Waypoint> planned = rrtStarPlanner.plan(source, staticObstacles);
            if (planned == source) {
                System.out.println("[RRT] No path found — using original");
                return source;
            }
            if (hasSharpTurns(planned)) {
                System.out.println("[RRT] Rejected (sharp turn) — using original");
                return source;
            }
            System.out.println("[RRT] Path accepted");
            return planned;
        } catch (Exception e) {
            System.out.printf("[RRT] Error: %s — using original%n", e.getMessage());
            return source;
        }
    }

    private List<Waypoint> applyDStarAvoidance(List<Waypoint> source,
                                                List<Obstacle> birdObstacles) {
        if (birdObstacles.isEmpty()) {
            System.out.println("[D*] Skipped — no bird obstacles");
            return source;
        }
        System.out.printf("[D*] Planning around %d bird obstacle(s)%n",
                birdObstacles.size());
        try {
            List<Waypoint> planned = dStarLitePlanner.plan(source, birdObstacles);
            if (planned == source) {
                System.out.println("[D*] No path found — keeping input");
                return source;
            }
            if (hasSharpTurns(planned)) {
                System.out.println("[D*] Rejected (sharp turn) — keeping input");
                return source;
            }
            System.out.println("[D*] Path accepted");
            return planned;
        } catch (Exception e) {
            System.out.printf("[D*] Error: %s — keeping input%n", e.getMessage());
            return source;
        }
    }

    private TrajectoryModel buildModel(List<Waypoint> controlPoints, double lambda) {
        if ("bspline".equalsIgnoreCase(algorithm)) {
            return bSplineCurveBuilder.build(controlPoints, lambda);
        } else {
            return cubicSplineBuilder.build(controlPoints);
        }
    }

    private boolean hasSharpTurns(List<Waypoint> waypoints) {
        for (int i = 1; i < waypoints.size() - 1; i++) {
            Waypoint prev = waypoints.get(i - 1);
            Waypoint curr = waypoints.get(i);
            Waypoint next = waypoints.get(i + 1);

            double ax = curr.getX() - prev.getX(), ay = curr.getY() - prev.getY(),
                   az = curr.getZ() - prev.getZ();
            double bx = next.getX() - curr.getX(), by = next.getY() - curr.getY(),
                   bz = next.getZ() - curr.getZ();

            double lenA = Math.sqrt(ax*ax + ay*ay + az*az);
            double lenB = Math.sqrt(bx*bx + by*by + bz*bz);
            if (lenA < 1e-6 || lenB < 1e-6) continue;

            double dot   = (ax*bx + ay*by + az*bz) / (lenA * lenB);
            double angle = Math.acos(Math.max(-1.0, Math.min(1.0, dot)));
            if (angle > MAX_TURN_ANGLE_RAD) return true;
        }
        return false;
    }
}