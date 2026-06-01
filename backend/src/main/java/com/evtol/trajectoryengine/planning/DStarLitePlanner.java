package com.evtol.trajectoryengine.planning;

import com.evtol.trajectoryengine.domain.Obstacle;
import com.evtol.trajectoryengine.domain.Waypoint;
import org.springframework.stereotype.Component;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.Set;

@Component
public class DStarLitePlanner {

    private static final double CLEARANCE_BUFFER_M = 90.0;
    private static final double GRID_RESOLUTION_M = 80.0;
    private static final double SEARCH_MARGIN_M = 500.0;
    private static final int REJOIN_LOOKAHEAD_WAYPOINTS = 6;
    private static final int MAX_EXTRACT_STEPS = 5_000;
    private static final double INF = 1e15;

    public List<Waypoint> plan(List<Waypoint> sourceWaypoints, List<Obstacle> obstacles) {
        if (sourceWaypoints == null || sourceWaypoints.size() < 2 || obstacles == null || obstacles.isEmpty()) {
            return sourceWaypoints;
        }

        CollisionWindow collisionWindow = findCollisionWindow(sourceWaypoints, obstacles);
        if (collisionWindow == null) {
            return sourceWaypoints;
        }

        int rejoinIndex = Math.min(
                sourceWaypoints.size() - 1,
                collisionWindow.lastCollisionSegment + REJOIN_LOOKAHEAD_WAYPOINTS + 1);
        int preIndex = Math.max(0, collisionWindow.firstCollisionSegment - 1);

        Waypoint start = sourceWaypoints.get(preIndex);
        Waypoint goal = sourceWaypoints.get(rejoinIndex);

        GridBounds bounds = buildBounds(sourceWaypoints, obstacles, preIndex, rejoinIndex);
        Cell startCell = worldToCell(start.getX(), start.getZ(), bounds);
        Cell goalCell = worldToCell(goal.getX(), goal.getZ(), bounds);

        Set<Cell> blocked = buildBlockedCells(bounds, obstacles, startCell, goalCell);
        blocked.remove(startCell);
        blocked.remove(goalCell);

        List<Point2> detour = planGridPath(startCell, goalCell, bounds, blocked);
        if (detour.isEmpty()) {
            return sourceWaypoints;
        }

        List<Point2> mergedPath = new ArrayList<>();
        for (int i = 0; i <= preIndex; i++) {
            mergedPath.add(new Point2(sourceWaypoints.get(i).getX(), sourceWaypoints.get(i).getZ()));
        }
        for (int i = 1; i < detour.size() - 1; i++) {
            mergedPath.add(detour.get(i));
        }
        for (int i = rejoinIndex; i < sourceWaypoints.size(); i++) {
            mergedPath.add(new Point2(sourceWaypoints.get(i).getX(), sourceWaypoints.get(i).getZ()));
        }

        return resamplePath(mergedPath, sourceWaypoints);
    }

    private CollisionWindow findCollisionWindow(List<Waypoint> waypoints, List<Obstacle> obstacles) {
        Integer first = null;
        Integer last = null;

        for (int i = 0; i < waypoints.size() - 1; i++) {
            Waypoint a = waypoints.get(i);
            Waypoint b = waypoints.get(i + 1);
            if (segmentHitsObstacle(a.getX(), a.getZ(), b.getX(), b.getZ(), obstacles)) {
                if (first == null) {
                    first = i;
                }
                last = i;
            }
        }

        if (first == null || last == null) {
            return null;
        }
        return new CollisionWindow(first, last);
    }

    private GridBounds buildBounds(List<Waypoint> source, List<Obstacle> obstacles, int startIndex, int goalIndex) {
        double minX = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double minZ = Double.POSITIVE_INFINITY;
        double maxZ = Double.NEGATIVE_INFINITY;

        for (int i = startIndex; i <= goalIndex; i++) {
            Waypoint waypoint = source.get(i);
            minX = Math.min(minX, waypoint.getX());
            maxX = Math.max(maxX, waypoint.getX());
            minZ = Math.min(minZ, waypoint.getZ());
            maxZ = Math.max(maxZ, waypoint.getZ());
        }

        for (Obstacle obstacle : obstacles) {
            double radius = obstacle.getRadius() + CLEARANCE_BUFFER_M;
            minX = Math.min(minX, obstacle.getX() - radius);
            maxX = Math.max(maxX, obstacle.getX() + radius);
            minZ = Math.min(minZ, obstacle.getZ() - radius);
            maxZ = Math.max(maxZ, obstacle.getZ() + radius);
        }

        return new GridBounds(
                minX - SEARCH_MARGIN_M,
                maxX + SEARCH_MARGIN_M,
                minZ - SEARCH_MARGIN_M,
                maxZ + SEARCH_MARGIN_M,
                GRID_RESOLUTION_M);
    }

    private Set<Cell> buildBlockedCells(GridBounds bounds, List<Obstacle> obstacles, Cell startCell, Cell goalCell) {
        Set<Cell> blocked = new HashSet<>();

        for (Obstacle obstacle : obstacles) {
            double inflatedRadius = obstacle.getRadius() + CLEARANCE_BUFFER_M;
            int minCol = (int) Math.floor((obstacle.getX() - inflatedRadius - bounds.minX) / bounds.resolution);
            int maxCol = (int) Math.ceil((obstacle.getX() + inflatedRadius - bounds.minX) / bounds.resolution);
            int minRow = (int) Math.floor((obstacle.getZ() - inflatedRadius - bounds.minZ) / bounds.resolution);
            int maxRow = (int) Math.ceil((obstacle.getZ() + inflatedRadius - bounds.minZ) / bounds.resolution);

            for (int col = minCol; col <= maxCol; col++) {
                for (int row = minRow; row <= maxRow; row++) {
                    Cell cell = new Cell(col, row);
                    if (cell.equals(startCell) || cell.equals(goalCell)) {
                        continue;
                    }
                    Point2 centre = cellToPoint(cell, bounds);
                    if (Math.hypot(centre.x - obstacle.getX(), centre.z - obstacle.getZ()) <= inflatedRadius) {
                        blocked.add(cell);
                    }
                }
            }
        }

        return blocked;
    }

    private List<Point2> planGridPath(Cell start, Cell goal, GridBounds bounds, Set<Cell> blocked) {
        Map<Cell, Double> g = new HashMap<>();
        Map<Cell, Double> rhs = new HashMap<>();
        PriorityQueue<QueueEntry> open = new PriorityQueue<>(Comparator.comparing(QueueEntry::key));

        rhs.put(goal, 0.0);
        open.offer(new QueueEntry(goal, calculateKey(goal, start, g, rhs)));

        computeShortestPath(start, goal, bounds, blocked, g, rhs, open);
        if (value(rhs, start) >= INF && value(g, start) >= INF) {
            return List.of();
        }

        List<Point2> path = new ArrayList<>();
        Cell current = start;
        path.add(cellToPoint(current, bounds));

        int steps = 0;
        while (!current.equals(goal) && steps++ < MAX_EXTRACT_STEPS) {
            Cell bestNext = null;
            double bestScore = INF;

            for (Cell neighbor : neighbors(current, bounds, blocked)) {
                double score = transitionCost(current, neighbor) + value(g, neighbor);
                if (score < bestScore) {
                    bestScore = score;
                    bestNext = neighbor;
                }
            }

            if (bestNext == null || bestScore >= INF) {
                return List.of();
            }

            current = bestNext;
            path.add(cellToPoint(current, bounds));
        }

        return current.equals(goal) ? simplifyPath(path, bounds, blocked) : List.of();
    }

    private void computeShortestPath(
            Cell start,
            Cell goal,
            GridBounds bounds,
            Set<Cell> blocked,
            Map<Cell, Double> g,
            Map<Cell, Double> rhs,
            PriorityQueue<QueueEntry> open) {

        while (!open.isEmpty()
                && (open.peek().key().compareTo(calculateKey(start, start, g, rhs)) < 0
                || Double.compare(value(rhs, start), value(g, start)) != 0)) {
            QueueEntry entry = open.poll();
            Key latestKey = calculateKey(entry.cell(), start, g, rhs);
            if (!entry.key().equals(latestKey)) {
                continue;
            }

            Cell u = entry.cell();
            if (value(g, u) > value(rhs, u)) {
                g.put(u, value(rhs, u));
                for (Cell predecessor : neighbors(u, bounds, blocked)) {
                    updateVertex(predecessor, start, goal, bounds, blocked, g, rhs, open);
                }
            } else {
                g.put(u, INF);
                updateVertex(u, start, goal, bounds, blocked, g, rhs, open);
                for (Cell predecessor : neighbors(u, bounds, blocked)) {
                    updateVertex(predecessor, start, goal, bounds, blocked, g, rhs, open);
                }
            }
        }
    }

    private void updateVertex(
            Cell u,
            Cell start,
            Cell goal,
            GridBounds bounds,
            Set<Cell> blocked,
            Map<Cell, Double> g,
            Map<Cell, Double> rhs,
            PriorityQueue<QueueEntry> open) {
        if (!u.equals(goal)) {
            double minRhs = INF;
            for (Cell successor : neighbors(u, bounds, blocked)) {
                minRhs = Math.min(minRhs, transitionCost(u, successor) + value(g, successor));
            }
            rhs.put(u, minRhs);
        }

        if (Double.compare(value(g, u), value(rhs, u)) != 0) {
            open.offer(new QueueEntry(u, calculateKey(u, start, g, rhs)));
        }
    }

    private Key calculateKey(Cell cell, Cell start, Map<Cell, Double> g, Map<Cell, Double> rhs) {
        double min = Math.min(value(g, cell), value(rhs, cell));
        return new Key(min + heuristic(start, cell), min);
    }

    private double value(Map<Cell, Double> map, Cell cell) {
        return map.getOrDefault(cell, INF);
    }

    private double heuristic(Cell a, Cell b) {
        double dx = Math.abs(a.col - b.col);
        double dz = Math.abs(a.row - b.row);
        return (Math.max(dx, dz) + (Math.sqrt(2.0) - 1.0) * Math.min(dx, dz)) * GRID_RESOLUTION_M;
    }

    private List<Cell> neighbors(Cell cell, GridBounds bounds, Set<Cell> blocked) {
        List<Cell> result = new ArrayList<>(8);
        for (int dc = -1; dc <= 1; dc++) {
            for (int dr = -1; dr <= 1; dr++) {
                if (dc == 0 && dr == 0) {
                    continue;
                }
                Cell next = new Cell(cell.col + dc, cell.row + dr);
                if (blocked.contains(next)) {
                    continue;
                }
                Point2 point = cellToPoint(next, bounds);
                if (point.x < bounds.minX || point.x > bounds.maxX || point.z < bounds.minZ || point.z > bounds.maxZ) {
                    continue;
                }
                result.add(next);
            }
        }
        return result;
    }

    private double transitionCost(Cell from, Cell to) {
        int dc = Math.abs(from.col - to.col);
        int dr = Math.abs(from.row - to.row);
        return (dc + dr == 2 ? Math.sqrt(2.0) : 1.0) * GRID_RESOLUTION_M;
    }

    private List<Point2> simplifyPath(List<Point2> path, GridBounds bounds, Set<Cell> blocked) {
        if (path.size() < 3) {
            return path;
        }

        List<Point2> simplified = new ArrayList<>();
        simplified.add(path.get(0));

        int anchor = 0;
        while (anchor < path.size() - 1) {
            int farthest = anchor + 1;
            for (int i = path.size() - 1; i > anchor + 1; i--) {
                if (segmentCellCollisionFree(path.get(anchor), path.get(i), bounds, blocked)) {
                    farthest = i;
                    break;
                }
            }
            simplified.add(path.get(farthest));
            anchor = farthest;
        }

        return simplified;
    }

    private boolean segmentCellCollisionFree(Point2 a, Point2 b, GridBounds bounds, Set<Cell> blocked) {
        double distance = Math.hypot(b.x - a.x, b.z - a.z);
        int samples = Math.max(2, (int) Math.ceil(distance / (bounds.resolution * 0.5)));
        for (int i = 0; i <= samples; i++) {
            double t = (double) i / samples;
            double x = a.x + (b.x - a.x) * t;
            double z = a.z + (b.z - a.z) * t;
            if (blocked.contains(worldToCell(x, z, bounds))) {
                return false;
            }
        }
        return true;
    }

    private boolean segmentHitsObstacle(double x1, double z1, double x2, double z2, List<Obstacle> obstacles) {
        for (Obstacle obstacle : obstacles) {
            double clearance = obstacle.getRadius() + CLEARANCE_BUFFER_M;
            if (distancePointToSegment(obstacle.getX(), obstacle.getZ(), x1, z1, x2, z2) < clearance) {
                return true;
            }
        }
        return false;
    }

    private double distancePointToSegment(double px, double pz, double ax, double az, double bx, double bz) {
        double dx = bx - ax;
        double dz = bz - az;
        double lenSq = dx * dx + dz * dz;
        if (lenSq < 1e-9) {
            return Math.hypot(px - ax, pz - az);
        }
        double t = Math.max(0.0, Math.min(1.0, ((px - ax) * dx + (pz - az) * dz) / lenSq));
        double cx = ax + t * dx;
        double cz = az + t * dz;
        return Math.hypot(px - cx, pz - cz);
    }

    private Cell worldToCell(double x, double z, GridBounds bounds) {
        int col = (int) Math.round((x - bounds.minX) / bounds.resolution);
        int row = (int) Math.round((z - bounds.minZ) / bounds.resolution);
        return new Cell(col, row);
    }

    private Point2 cellToPoint(Cell cell, GridBounds bounds) {
        double x = bounds.minX + (cell.col * bounds.resolution);
        double z = bounds.minZ + (cell.row * bounds.resolution);
        return new Point2(x, z);
    }

    private List<Waypoint> resamplePath(List<Point2> path, List<Waypoint> source) {
        if (path.size() < 2) {
            return source;
        }

        double[] cumulativeDistance = new double[path.size()];
        for (int i = 1; i < path.size(); i++) {
            Point2 a = path.get(i - 1);
            Point2 b = path.get(i);
            cumulativeDistance[i] = cumulativeDistance[i - 1] + Math.hypot(b.x - a.x, b.z - a.z);
        }

        double total = cumulativeDistance[cumulativeDistance.length - 1];
        if (total < 1e-6) {
            return source;
        }

        List<Waypoint> result = new ArrayList<>(source.size());
        int pathIndex = 1;

        for (int i = 0; i < source.size(); i++) {
            double alpha = source.size() == 1 ? 0.0 : (double) i / (source.size() - 1);
            double targetDistance = alpha * total;
            while (pathIndex < cumulativeDistance.length - 1 && cumulativeDistance[pathIndex] < targetDistance) {
                pathIndex++;
            }

            int prevIndex = Math.max(0, pathIndex - 1);
            Point2 p0 = path.get(prevIndex);
            Point2 p1 = path.get(pathIndex);
            double segmentLength = cumulativeDistance[pathIndex] - cumulativeDistance[prevIndex];
            double localAlpha = segmentLength > 1e-6
                    ? (targetDistance - cumulativeDistance[prevIndex]) / segmentLength
                    : 0.0;

            result.add(new Waypoint(
                    source.get(i).getT(),
                    lerp(p0.x, p1.x, localAlpha),
                    source.get(i).getY(),
                    lerp(p0.z, p1.z, localAlpha)));
        }

        result.set(0, source.get(0));
        result.set(result.size() - 1, source.get(source.size() - 1));
        return result;
    }

    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static final class CollisionWindow {
        private final int firstCollisionSegment;
        private final int lastCollisionSegment;

        private CollisionWindow(int firstCollisionSegment, int lastCollisionSegment) {
            this.firstCollisionSegment = firstCollisionSegment;
            this.lastCollisionSegment = lastCollisionSegment;
        }
    }

    private static final class GridBounds {
        private final double minX;
        private final double maxX;
        private final double minZ;
        private final double maxZ;
        private final double resolution;

        private GridBounds(double minX, double maxX, double minZ, double maxZ, double resolution) {
            this.minX = minX;
            this.maxX = maxX;
            this.minZ = minZ;
            this.maxZ = maxZ;
            this.resolution = resolution;
        }
    }

    private static final class Point2 {
        private final double x;
        private final double z;

        private Point2(double x, double z) {
            this.x = x;
            this.z = z;
        }
    }

    private static final class Cell {
        private final int col;
        private final int row;

        private Cell(int col, int row) {
            this.col = col;
            this.row = row;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) {
                return true;
            }
            if (!(obj instanceof Cell other)) {
                return false;
            }
            return col == other.col && row == other.row;
        }

        @Override
        public int hashCode() {
            return Objects.hash(col, row);
        }
    }

    private static final class QueueEntry {
        private final Cell cell;
        private final Key key;

        private QueueEntry(Cell cell, Key key) {
            this.cell = cell;
            this.key = key;
        }

        private Cell cell() {
            return cell;
        }

        private Key key() {
            return key;
        }
    }

    private static final class Key implements Comparable<Key> {
        private final double first;
        private final double second;

        private Key(double first, double second) {
            this.first = first;
            this.second = second;
        }

        @Override
        public int compareTo(Key other) {
            int byFirst = Double.compare(first, other.first);
            if (byFirst != 0) {
                return byFirst;
            }
            return Double.compare(second, other.second);
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) {
                return true;
            }
            if (!(obj instanceof Key other)) {
                return false;
            }
            return Double.compare(first, other.first) == 0
                    && Double.compare(second, other.second) == 0;
        }

        @Override
        public int hashCode() {
            return Objects.hash(first, second);
        }
    }
}
