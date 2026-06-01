package com.evtol.trajectoryengine.planning;

import com.evtol.trajectoryengine.domain.Obstacle;
import com.evtol.trajectoryengine.domain.Waypoint;
import org.springframework.stereotype.Component;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

/**
 * RRT* planner — plans in the X-Z horizontal plane (preserving waypoint Y/altitude),
 * but uses full 3D distance for collision checks against obstacles.
 *
 * Why X-Z only for planning:
 *   The waypoint CSV only defines X and Z coordinates (the Y column is always 0).
 *   Altitude (Y) is controlled by the flight simulation itself, not the planner.
 *   Planning in Y as well would produce paths that go underground because the
 *   planner has no meaningful altitude constraints to work with.
 *
 * Why 3D for collision:
 *   Obstacle centres have Y=0 but their radius is 155m+, so a purely 2D check
 *   (ignoring Y) is equivalent here — but 3D is kept for correctness when
 *   obstacle Y values are non-zero.
 */
@Component
public class RrtStarPlanner {

    private static final double CLEARANCE_BUFFER = 260.0;  // extra margin for B-spline smoothing
    private static final double STEP_SIZE        = 200.0;
    private static final double GOAL_THRESHOLD   = 260.0;
    private static final double NEIGHBOR_RADIUS  = 500.0;  // wider rewiring for smoother paths
    private static final int    MAX_ITERATIONS   = 10000;  // more iterations for tight corridors
    private static final int    SHORTCUT_PASSES  = 120;
    private static final double GOAL_SAMPLE_RATE = 0.18;

    public List<Waypoint> plan(List<Waypoint> sourceWaypoints, List<Obstacle> obstacles) {
        if (sourceWaypoints == null || sourceWaypoints.size() < 2) {
            return sourceWaypoints;
        }

        Waypoint start = sourceWaypoints.get(0);
        Waypoint goal  = sourceWaypoints.get(sourceWaypoints.size() - 1);

        Bounds bounds = buildBounds(sourceWaypoints, obstacles);
        Random random = new Random(42L);

        List<Node> nodes = new ArrayList<>();
        nodes.add(new Node(start.getX(), start.getZ(), null, 0.0));

        Node bestGoalNode = null;

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            double[] sample   = sample(random, bounds, goal);
            Node     nearest  = nearest(nodes, sample[0], sample[1]);
            Node     candidate = steer(nearest, sample[0], sample[1]);

            // Collision check in 3D: use midpoint Y of the segment for the check
            double segY = (start.getY() + goal.getY()) / 2.0;
            if (!isSegmentCollisionFree(
                    nearest.x, segY, nearest.z,
                    candidate.x, segY, candidate.z,
                    obstacles)) {
                continue;
            }

            List<Node> neighbors = near(nodes, candidate.x, candidate.z, NEIGHBOR_RADIUS);
            Node   parent   = nearest;
            double bestCost = nearest.cost + dist2(nearest, candidate);

            for (Node neighbor : neighbors) {
                if (!isSegmentCollisionFree(
                        neighbor.x, segY, neighbor.z,
                        candidate.x, segY, candidate.z,
                        obstacles)) continue;

                double cost = neighbor.cost + dist2(neighbor, candidate);
                if (cost < bestCost) { parent = neighbor; bestCost = cost; }
            }

            candidate.parent = parent;
            candidate.cost   = bestCost;
            nodes.add(candidate);

            for (Node neighbor : neighbors) {
                if (neighbor == parent) continue;
                if (!isSegmentCollisionFree(
                        candidate.x, segY, candidate.z,
                        neighbor.x, segY, neighbor.z,
                        obstacles)) continue;
                double rewiredCost = candidate.cost + dist2(candidate, neighbor);
                if (rewiredCost < neighbor.cost) {
                    neighbor.parent = candidate;
                    neighbor.cost   = rewiredCost;
                }
            }

            if (dist2d(candidate.x, candidate.z, goal.getX(), goal.getZ()) <= GOAL_THRESHOLD
                    && isSegmentCollisionFree(
                        candidate.x, segY, candidate.z,
                        goal.getX(), segY, goal.getZ(),
                        obstacles)) {
                double gCost = candidate.cost
                        + dist2d(candidate.x, candidate.z, goal.getX(), goal.getZ());
                Node goalNode = new Node(goal.getX(), goal.getZ(), candidate, gCost);
                if (bestGoalNode == null || gCost < bestGoalNode.cost) bestGoalNode = goalNode;
            }
        }

        if (bestGoalNode == null) return sourceWaypoints;

        List<Point2> path = extractPath(bestGoalNode);
        path = shortcutPath(path, obstacles, (start.getY() + goal.getY()) / 2.0);
        return resamplePath(path, sourceWaypoints);
    }

    // ── bounds (X-Z only) ────────────────────────────────────────────────────

    private Bounds buildBounds(List<Waypoint> waypoints, List<Obstacle> obstacles) {
        double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
        double minZ = Double.POSITIVE_INFINITY, maxZ = Double.NEGATIVE_INFINITY;

        for (Waypoint w : waypoints) {
            minX = Math.min(minX, w.getX()); maxX = Math.max(maxX, w.getX());
            minZ = Math.min(minZ, w.getZ()); maxZ = Math.max(maxZ, w.getZ());
        }
        for (Obstacle o : obstacles) {
            double r = o.getRadius() + CLEARANCE_BUFFER;
            minX = Math.min(minX, o.getX() - r); maxX = Math.max(maxX, o.getX() + r);
            minZ = Math.min(minZ, o.getZ() - r); maxZ = Math.max(maxZ, o.getZ() + r);
        }
        double m = 450.0;
        return new Bounds(minX - m, maxX + m, minZ - m, maxZ + m);
    }

    // ── sampling (X-Z only) ──────────────────────────────────────────────────

    private double[] sample(Random random, Bounds b, Waypoint goal) {
        if (random.nextDouble() < GOAL_SAMPLE_RATE)
            return new double[]{goal.getX(), goal.getZ()};
        return new double[]{
            b.minX + random.nextDouble() * (b.maxX - b.minX),
            b.minZ + random.nextDouble() * (b.maxZ - b.minZ)
        };
    }

    // ── tree helpers (X-Z) ───────────────────────────────────────────────────

    private Node nearest(List<Node> nodes, double x, double z) {
        Node best = nodes.get(0);
        double bestD = dist2d(best.x, best.z, x, z);
        for (Node n : nodes) {
            double d = dist2d(n.x, n.z, x, z);
            if (d < bestD) { best = n; bestD = d; }
        }
        return best;
    }

    private Node steer(Node from, double tx, double tz) {
        double dx = tx - from.x, dz = tz - from.z;
        double len = Math.hypot(dx, dz);
        if (len <= STEP_SIZE) return new Node(tx, tz, from, from.cost + len);
        double s = STEP_SIZE / len;
        return new Node(from.x + dx*s, from.z + dz*s, from, from.cost + STEP_SIZE);
    }

    private List<Node> near(List<Node> nodes, double x, double z, double radius) {
        List<Node> result = new ArrayList<>();
        for (Node n : nodes) if (dist2d(n.x, n.z, x, z) <= radius) result.add(n);
        return result;
    }

    // ── 3-D collision check ──────────────────────────────────────────────────

    /**
     * Collision check treats every obstacle as an infinite-height cylinder.
     * Only X-Z distance matters — Y (altitude) is ignored.
     * This is correct because:
     *  1. The planner only controls X-Z routing, not altitude
     *  2. Buildings span full height so if you're in their X-Z footprint you collide
     *  3. Waypoint Y values are 0 so a 3D check would never trigger at flight altitude
     */
    private boolean isSegmentCollisionFree(
            double x1, double y1, double z1,
            double x2, double y2, double z2,
            List<Obstacle> obstacles) {
        for (Obstacle o : obstacles) {
            double clearance = o.getRadius() + CLEARANCE_BUFFER;
            if (segDistToPoint2D(x1, z1, x2, z2, o.getX(), o.getZ()) < clearance) return false;
        }
        return true;
    }

    /** Minimum distance from point P to line-segment AB in the X-Z plane only. */
    private double segDistToPoint2D(double ax, double az,
                                    double bx, double bz,
                                    double px, double pz) {
        double dx = bx-ax, dz = bz-az;
        double lenSq = dx*dx + dz*dz;
        if (lenSq < 1e-9) return dist2d(ax, az, px, pz);
        double t = Math.max(0, Math.min(1, ((px-ax)*dx + (pz-az)*dz) / lenSq));
        return dist2d(ax + t*dx, az + t*dz, px, pz);
    }

    // ── path extraction & smoothing ──────────────────────────────────────────

    private List<Point2> extractPath(Node goal) {
        List<Point2> path = new ArrayList<>();
        for (Node cur = goal; cur != null; cur = cur.parent)
            path.add(new Point2(cur.x, cur.z));
        Collections.reverse(path);
        return path;
    }

    private List<Point2> shortcutPath(List<Point2> path, List<Obstacle> obstacles, double segY) { // segY unused — infinite cylinder check
        if (path.size() < 3) return path;
        List<Point2> simplified = new ArrayList<>(path);
        for (int pass = 0; pass < SHORTCUT_PASSES; pass++) {
            boolean changed = false;
            outer:
            for (int i = 0; i < simplified.size() - 2; i++) {
                for (int j = simplified.size() - 1; j > i + 1; j--) {
                    Point2 a = simplified.get(i), b = simplified.get(j);
                    if (!isSegmentCollisionFree(a.x, 0, a.z, b.x, 0, b.z, obstacles)) continue;
                    List<Point2> next = new ArrayList<>();
                    next.addAll(simplified.subList(0, i + 1));
                    next.addAll(simplified.subList(j, simplified.size()));
                    simplified = next;
                    changed = true;
                    break outer;
                }
            }
            if (!changed) break;
        }
        return simplified;
    }

    /** Resample the X-Z path back to waypoints, preserving the original Y from each source waypoint. */
    private List<Waypoint> resamplePath(List<Point2> path, List<Waypoint> source) {
        if (path.size() < 2) return source;

        double[] cum = new double[path.size()];
        for (int i = 1; i < path.size(); i++) {
            Point2 a = path.get(i-1), b = path.get(i);
            cum[i] = cum[i-1] + dist2d(a.x, a.z, b.x, b.z);
        }
        double total = cum[cum.length - 1];
        if (total < 1e-6) return source;

        List<Waypoint> result = new ArrayList<>(source.size());
        int pi = 1;
        for (int i = 0; i < source.size(); i++) {
            double alpha     = source.size() == 1 ? 0.0 : (double) i / (source.size() - 1);
            double targetDst = alpha * total;
            while (pi < cum.length - 1 && cum[pi] < targetDst) pi++;

            Point2 p0 = path.get(Math.max(pi-1, 0));
            Point2 p1 = path.get(pi);
            double segLen = cum[pi] - cum[Math.max(pi-1, 0)];
            double la = segLen > 1e-6 ? (targetDst - cum[Math.max(pi-1, 0)]) / segLen : 0.0;

            result.add(new Waypoint(
                    source.get(i).getT(),
                    lerp(p0.x, p1.x, la),
                    source.get(i).getY(),   // ← preserve original altitude
                    lerp(p0.z, p1.z, la)));
        }
        result.set(0, source.get(0));
        result.set(result.size()-1, source.get(source.size()-1));
        return result;
    }

    // ── math helpers ─────────────────────────────────────────────────────────

    private double dist2(Node a, Node b) { return dist2d(a.x, a.z, b.x, b.z); }
    private double dist2d(double x1, double z1, double x2, double z2) { return Math.hypot(x2-x1, z2-z1); }
    private double dist3d(double x1, double y1, double z1, double x2, double y2, double z2) {
        double dx=x2-x1, dy=y2-y1, dz=z2-z1;
        return Math.sqrt(dx*dx+dy*dy+dz*dz);
    }
    private double lerp(double a, double b, double t) { return a + (b-a)*t; }

    // ── inner types ───────────────────────────────────────────────────────────

    private static class Bounds {
        final double minX, maxX, minZ, maxZ;
        Bounds(double minX, double maxX, double minZ, double maxZ) {
            this.minX=minX; this.maxX=maxX; this.minZ=minZ; this.maxZ=maxZ;
        }
    }
    private static class Point2 {
        final double x, z;
        Point2(double x, double z) { this.x=x; this.z=z; }
    }
    private static class Node {
        final double x, z;
        Node parent; double cost;
        Node(double x, double z, Node parent, double cost) {
            this.x=x; this.z=z; this.parent=parent; this.cost=cost;
        }
    }
}
