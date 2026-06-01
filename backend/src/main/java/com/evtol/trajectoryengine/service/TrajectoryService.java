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

import java.util.List;

@Service
@RequiredArgsConstructor
public class TrajectoryService {

    private final CsvWaypointDataProvider dataProvider;
    private final ObstacleRegistryService obstacleRegistryService;
    private final WaypointValidator validator;
    private final CubicSplineBuilder cubicSplineBuilder;
    private final BSplineCurveBuilder bSplineCurveBuilder;
    private final SamplingService samplingService;
    private final LeastSquaresFitter leastSquaresFitter;
    private final RrtStarPlanner rrtStarPlanner;
    private final DStarLitePlanner dStarLitePlanner;

    @Value("${trajectory.sampling.interval}")
    private double samplingInterval;

    @Value("${trajectory.algorithm}")
    private String algorithm;

    private static final double MAX_TURN_ANGLE_RAD = Math.toRadians(150.0);

    public TrajectoryResponse generateTrajectory(double lambda) {
        return generateTrajectory(lambda, List.of());
    }

    public TrajectoryResponse generateTrajectory(double lambda, List<Obstacle> requestObstacles) {
        List<Waypoint> rawWaypoints = dataProvider.loadWaypoints();
        List<Obstacle> staticObstacles = obstacleRegistryService.loadStaticPlannerObstacles(requestObstacles);
        List<Obstacle> birdObstacles = obstacleRegistryService.loadBirdObstacles();
        List<Waypoint> waypoints = applyObstacleAvoidance(rawWaypoints, staticObstacles, birdObstacles);

        validator.validate(waypoints);

        List<Waypoint> controlPoints = leastSquaresFitter.fit(waypoints);
        TrajectoryModel trajectoryModel;

        if ("bspline".equalsIgnoreCase(algorithm)) {
            trajectoryModel = bSplineCurveBuilder.build(controlPoints, lambda);
        } else {
            trajectoryModel = cubicSplineBuilder.build(waypoints);
        }

        List<TrajectoryPoint> points = samplingService.sample(trajectoryModel, samplingInterval);

        return new TrajectoryResponse(
                points,
                waypoints,
                controlPoints,
                trajectoryModel.getTotalDuration());
    }

    private List<Waypoint> applyObstacleAvoidance(
            List<Waypoint> sourceWaypoints,
            List<Obstacle> staticObstacles,
            List<Obstacle> birdObstacles) {
        if (sourceWaypoints == null || sourceWaypoints.size() < 3) {
            System.out.printf("[PLANNER] Skipped - too few waypoints%n");
            return sourceWaypoints;
        }

        // Phase 1: RRT* for buildings, walls, and other static obstacles
        List<Waypoint> afterRrt = sourceWaypoints;
        if (!staticObstacles.isEmpty()) {
            System.out.printf("[RRT] Planning around %d static obstacle(s)%n", staticObstacles.size());
            try {
                List<Waypoint> planned = rrtStarPlanner.plan(sourceWaypoints, staticObstacles);
                if (planned == sourceWaypoints) {
                    System.out.println("[RRT] No collision-free path found - using original");
                } else if (hasSharpTurns(planned)) {
                    System.out.println("[RRT] Deviated path rejected (sharp turn > 120 deg) - using original");
                } else {
                    System.out.println("[RRT] Deviated path accepted");
                    afterRrt = planned;
                }
            } catch (Exception e) {
                System.out.printf("[RRT] Planner threw: %s - using original%n", e.getMessage());
            }
        } else {
            System.out.println("[RRT] Skipped - no static obstacles");
        }

        // Phase 2: D* Lite for dynamic bird obstacles
        List<Waypoint> afterDStar = afterRrt;
        if (!birdObstacles.isEmpty()) {
            System.out.printf("[D*] Planning around %d bird obstacle(s)%n", birdObstacles.size());
            try {
                List<Waypoint> planned = dStarLitePlanner.plan(afterRrt, birdObstacles);
                if (planned == afterRrt) {
                    System.out.println("[D*] No collision-free path found - keeping RRT* result");
                } else if (hasSharpTurns(planned)) {
                    System.out.println("[D*] Deviated path rejected (sharp turn > 120 deg) - keeping RRT* result");
                } else {
                    System.out.println("[D*] Deviated path accepted");
                    afterDStar = planned;
                }
            } catch (Exception e) {
                System.out.printf("[D*] Planner threw: %s - keeping RRT* result%n", e.getMessage());
            }
        } else {
            System.out.println("[D*] Skipped - no bird obstacles");
        }

        return afterDStar;
    }

    private boolean hasSharpTurns(List<Waypoint> waypoints) {
        for (int i = 1; i < waypoints.size() - 1; i++) {
            Waypoint prev = waypoints.get(i - 1);
            Waypoint curr = waypoints.get(i);
            Waypoint next = waypoints.get(i + 1);

            double ax = curr.getX() - prev.getX();
            double ay = curr.getY() - prev.getY();
            double az = curr.getZ() - prev.getZ();
            double bx = next.getX() - curr.getX();
            double by = next.getY() - curr.getY();
            double bz = next.getZ() - curr.getZ();

            double lenA = Math.sqrt(ax * ax + ay * ay + az * az);
            double lenB = Math.sqrt(bx * bx + by * by + bz * bz);
            if (lenA < 1e-6 || lenB < 1e-6) {
                continue;
            }

            double dot = (ax * bx + ay * by + az * bz) / (lenA * lenB);
            dot = Math.max(-1.0, Math.min(1.0, dot));
            double angle = Math.acos(dot);
            if (angle > MAX_TURN_ANGLE_RAD) {
                return true;
            }
        }
        return false;
    }
}
