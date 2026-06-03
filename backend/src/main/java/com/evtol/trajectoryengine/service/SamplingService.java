package com.evtol.trajectoryengine.service;

import com.evtol.trajectoryengine.bspline.DeBoorEvaluator;
import com.evtol.trajectoryengine.datasource.CsvWaypointDataProvider;
import com.evtol.trajectoryengine.domain.TrajectoryModel;
import com.evtol.trajectoryengine.domain.TrajectoryPoint;
import com.evtol.trajectoryengine.domain.Waypoint;

import org.springframework.stereotype.Service;

import java.util.ArrayList;
import java.util.List;

@Service
public class SamplingService {

    private final DeBoorEvaluator       deBoorEvaluator = new DeBoorEvaluator();
    private final CsvWaypointDataProvider csvWaypointDataProvider;

    public SamplingService(CsvWaypointDataProvider csvWaypointDataProvider) {
        this.csvWaypointDataProvider = csvWaypointDataProvider;
    }

    // ── full forward sample ───────────────────────────────────────────────────

    /**
     * Samples the entire trajectory from tStart to tEnd.
     * Used during full replans.
     */
    public List<TrajectoryPoint> sample(TrajectoryModel trajectoryModel,
                                         double timeStep) {

        List<Waypoint> waypoints = csvWaypointDataProvider.loadWaypoints();
        double tStart       = waypoints.get(0).getT();
        double tEnd         = waypoints.get(waypoints.size() - 1).getT();

        return sampleWindow(trajectoryModel, timeStep, tStart, tEnd);
    }

    // ── windowed sample ───────────────────────────────────────────────────────

    /**
     * Samples the trajectory over a specific time window [windowStart, windowEnd].
     *
     * <p>Used by bird-only replans to resample only the segment that
     * D* Lite modified.  The caller splices the result back into the
     * cached base trajectory rather than replacing the whole list.</p>
     *
     * @param trajectoryModel the model to evaluate
     * @param timeStep        sample interval in seconds
     * @param windowStart     real mission time to start sampling (inclusive)
     * @param windowEnd       real mission time to stop  sampling (inclusive)
     * @return trajectory points covering only the requested window
     */
    public List<TrajectoryPoint> sampleWindow(TrajectoryModel trajectoryModel,
                                               double timeStep,
                                               double windowStart,
                                               double windowEnd) {

        List<Waypoint> waypoints      = csvWaypointDataProvider.loadWaypoints();
        double tStart                 = waypoints.get(0).getT();
        double tEnd                   = waypoints.get(waypoints.size() - 1).getT();
        double totalDuration          = tEnd - tStart;

        double[] knots  = trajectoryModel.getKnots();
        int      degree = trajectoryModel.getDegree();
        double   uStart = knots[degree];
        double   uEnd   = knots[knots.length - degree - 1];
        double   uRange = uEnd - uStart;

        // Clamp the requested window to the valid dataset range
        double wStart = Math.max(windowStart, tStart);
        double wEnd   = Math.min(windowEnd,   tEnd);

        List<TrajectoryPoint> samples = new ArrayList<>();

        for (double t = wStart; t <= wEnd + 1e-9; t += timeStep) {
            double tClamped = Math.min(t, wEnd);

            double uNorm = (tClamped - tStart) / totalDuration;
            double u     = uStart + uNorm * uRange;
            u = Math.max(uStart, Math.min(uEnd, u));

            List<Waypoint> controlPoints = trajectoryModel.getControlPoints();
            Waypoint p = deBoorEvaluator.evaluate(u, degree, knots, controlPoints);

            samples.add(new TrajectoryPoint(tClamped, p.getX(), p.getY(), p.getZ()));
        }

        return samples;
    }

    // ── splice helper ─────────────────────────────────────────────────────────

    /**
     * Splices a locally replanned segment back into a full trajectory list.
     *
     * <p>Replaces all points in {@code base} whose timestamp falls within
     * [{@code windowStart}, {@code windowEnd}] with {@code patch}, leaving
     * every point outside that window completely unchanged.</p>
     *
     * @param base        the full cached trajectory (not modified — a new list
     *                    is returned)
     * @param patch       the replacement points for the affected window
     * @param windowStart start of the time window to replace (inclusive)
     * @param windowEnd   end   of the time window to replace (inclusive)
     * @return a new list containing the spliced trajectory
     */
    public List<TrajectoryPoint> splice(List<TrajectoryPoint> base,
                                         List<TrajectoryPoint> patch,
                                         double windowStart,
                                         double windowEnd) {

        List<TrajectoryPoint> result = new ArrayList<>(base.size());

        for (TrajectoryPoint point : base) {
            double t = point.getT();
            if (t < windowStart || t > windowEnd) {
                // Outside the bird-detour window — keep the original point.
                result.add(point);
            }
            // Points inside the window are dropped; patch is added below.
        }

        // Insert the patch at the correct position (sorted by time).
        // Find the insertion index — first point whose time > windowStart.
        int insertAt = 0;
        for (int i = 0; i < result.size(); i++) {
            if (result.get(i).getT() < windowStart) {
                insertAt = i + 1;
            } else {
                break;
            }
        }

        result.addAll(insertAt, patch);
        return result;
    }
}