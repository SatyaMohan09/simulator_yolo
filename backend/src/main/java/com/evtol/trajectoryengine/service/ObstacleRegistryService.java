package com.evtol.trajectoryengine.service;

import com.evtol.trajectoryengine.datasource.CsvObstacleDataProvider;
import com.evtol.trajectoryengine.domain.Obstacle;
import com.evtol.trajectoryengine.dto.ObstacleSnapshotResponse;
import com.evtol.trajectoryengine.dto.ObstacleValidationResponse;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

@Service
public class ObstacleRegistryService {

    private final CsvObstacleDataProvider csvObstacleDataProvider;

    @Value("${trajectory.obstacle.seed-file-path:}")
    private String seedObstacleFilePath;

    @Value("${trajectory.obstacle.detected-file-path:}")
    private String detectedObstacleFilePath;

    @Value("${trajectory.obstacle.dynamic-file-path:}")
    private String dynamicObstacleFilePath;

    @Value("${trajectory.obstacle.ground-truth-file-path:}")
    private String groundTruthObstacleFilePath;

    @Value("${trajectory.obstacle.imageprocessing-file-path:}")
    private String imageProcessingObstacleFilePath;

    @Value("${trajectory.obstacle.match-threshold-m:120}")
    private double matchThresholdM;

    public ObstacleRegistryService(CsvObstacleDataProvider csvObstacleDataProvider) {
        this.csvObstacleDataProvider = csvObstacleDataProvider;
    }

    public List<Obstacle> loadSeedObstacles() {
        return csvObstacleDataProvider.loadObstacles(seedObstacleFilePath);
    }

    public List<Obstacle> loadDetectedObstacles() {
        return csvObstacleDataProvider.loadObstacles(detectedObstacleFilePath);
    }

    public List<Obstacle> loadDynamicObstacle2Obstacles() {
        return csvObstacleDataProvider.loadObstacles(dynamicObstacleFilePath);
    }

    public List<Obstacle> loadGroundTruthObstacles() {
        return csvObstacleDataProvider.loadObstacles(groundTruthObstacleFilePath);
    }

    public List<Obstacle> loadImageProcessingObstacles() {
        return csvObstacleDataProvider.loadObstacles(imageProcessingObstacleFilePath);
    }

    public List<Obstacle> loadPlannerObstacles(List<Obstacle> requestObstacles) {
        List<Obstacle> merged = new ArrayList<>();
        merged.addAll(loadSeedObstacles());
        merged.addAll(loadDetectedObstacles());
        if (requestObstacles != null) {
            merged.addAll(requestObstacles);
        }
        return deduplicate(merged);
    }

    public List<Obstacle> loadStaticPlannerObstacles(List<Obstacle> requestObstacles) {
        return loadPlannerObstacles(requestObstacles);
    }

    public List<Obstacle> loadBirdObstacles() {
        return loadDynamicObstacle2Obstacles();
    }

    public ObstacleSyncResult syncRuntimeObstacles(List<Obstacle> detections, List<Obstacle> birdDetections) {
        List<Obstacle> normalizedDetections = deduplicate(detections);
        List<Obstacle> normalizedBirds = deduplicate(birdDetections);

        List<Obstacle> previousDetections = loadDetectedObstacles();
        List<Obstacle> previousBirds = loadDynamicObstacle2Obstacles();

        writeObstaclesCsv(detectedObstacleFilePath, normalizedDetections);
        writeObstaclesCsv(dynamicObstacleFilePath, normalizedBirds);

        boolean changed = !sameObstacleSet(previousDetections, normalizedDetections)
                || !sameObstacleSet(previousBirds, normalizedBirds);

        return new ObstacleSyncResult(normalizedDetections, normalizedBirds, changed);
    }

    public ObstacleSnapshotResponse getSnapshot() {
        List<Obstacle> imageProcessing = loadImageProcessingObstacles();
        List<Obstacle> seed = loadSeedObstacles();
        List<Obstacle> detected = loadDetectedObstacles();
        List<Obstacle> planner = loadPlannerObstacles(List.of());
        return new ObstacleSnapshotResponse(imageProcessing, seed, detected, planner);
    }

    public ObstacleSnapshotResponse bootstrapStaticScenario(int seedCount) {
        List<Obstacle> imageProcessing = deduplicate(loadImageProcessingObstacles());

        int safeSeedCount = Math.max(0, seedCount);
        int limit = Math.min(safeSeedCount, imageProcessing.size());

        writeObstaclesCsv(seedObstacleFilePath, imageProcessing.subList(0, limit));
        writeObstaclesCsv(detectedObstacleFilePath, List.of());
        writeObstaclesCsv(dynamicObstacleFilePath, List.of());
        return getSnapshot();
    }

    public ObstacleSnapshotResponse revealSurpriseObstacles(int count) {
        List<Obstacle> imageProcessing = deduplicate(loadImageProcessingObstacles());
        List<Obstacle> seed = loadSeedObstacles();
        List<Obstacle> detected = loadDetectedObstacles();

        Map<String, Obstacle> known = new LinkedHashMap<>();
        for (Obstacle obstacle : seed) {
            Obstacle normalized = normalize(obstacle);
            if (normalized != null) {
                known.put(buildKey(normalized), normalized);
            }
        }
        for (Obstacle obstacle : detected) {
            Obstacle normalized = normalize(obstacle);
            if (normalized != null) {
                known.put(buildKey(normalized), normalized);
            }
        }

        int revealLimit = Math.max(0, count);
        List<Obstacle> surprises = new ArrayList<>(detected);
        for (Obstacle obstacle : imageProcessing) {
            Obstacle normalized = normalize(obstacle);
            if (normalized == null) {
                continue;
            }

            String key = buildKey(normalized);
            if (known.containsKey(key)) {
                continue;
            }

            surprises.add(normalized);
            known.put(key, normalized);
            if (surprises.size() - detected.size() >= revealLimit) {
                break;
            }
        }

        writeObstaclesCsv(detectedObstacleFilePath, deduplicate(surprises));
        return getSnapshot();
    }

    public ObstacleValidationResponse validateDetectedAgainstGroundTruth() {
        List<Obstacle> groundTruth = loadGroundTruthObstacles();
        List<Obstacle> detected = loadDetectedObstacles();

        if (groundTruth.isEmpty() || detected.isEmpty()) {
            return new ObstacleValidationResponse(
                    groundTruth.size(),
                    detected.size(),
                    0,
                    groundTruth.size(),
                    detected.size(),
                    matchThresholdM,
                    null,
                    null);
        }

        boolean[] matchedDetections = new boolean[detected.size()];
        List<Double> errors = new ArrayList<>();
        int matchedCount = 0;

        for (Obstacle truth : groundTruth) {
            int nearestIndex = -1;
            double nearestDistance = Double.POSITIVE_INFINITY;

            for (int i = 0; i < detected.size(); i++) {
                if (matchedDetections[i]) {
                    continue;
                }

                double distance = distance(truth, detected.get(i));
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    nearestIndex = i;
                }
            }

            if (nearestIndex >= 0 && nearestDistance <= matchThresholdM) {
                matchedDetections[nearestIndex] = true;
                matchedCount++;
                errors.add(nearestDistance);
            }
        }

        int unmatchedDetections = 0;
        for (boolean matched : matchedDetections) {
            if (!matched) {
                unmatchedDetections++;
            }
        }

        int missedCount = groundTruth.size() - matchedCount;
        Double meanError = errors.isEmpty()
                ? null
                : errors.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
        Double maxError = errors.isEmpty()
                ? null
                : errors.stream().mapToDouble(Double::doubleValue).max().orElse(0.0);

        return new ObstacleValidationResponse(
                groundTruth.size(),
                detected.size(),
                matchedCount,
                missedCount,
                unmatchedDetections,
                matchThresholdM,
                meanError,
                maxError);
    }

    private List<Obstacle> deduplicate(List<Obstacle> obstacles) {
        Map<String, Obstacle> deduped = new LinkedHashMap<>();
        if (obstacles == null) {
            return new ArrayList<>();
        }

        for (Obstacle obstacle : obstacles) {
            if (obstacle == null) {
                continue;
            }

            Obstacle normalized = normalize(obstacle);
            if (normalized == null) {
                continue;
            }

            deduped.put(buildKey(normalized), normalized);
        }

        return deduped.values().stream()
                .sorted(Comparator.comparingDouble(Obstacle::getX)
                        .thenComparingDouble(Obstacle::getZ)
                        .thenComparingDouble(Obstacle::getY))
                .toList();
    }

    private Obstacle normalize(Obstacle obstacle) {
        if (!Double.isFinite(obstacle.getX())
                || !Double.isFinite(obstacle.getY())
                || !Double.isFinite(obstacle.getZ())) {
            return null;
        }

        double radius = Double.isFinite(obstacle.getRadius()) && obstacle.getRadius() > 0
                ? obstacle.getRadius()
                : 155.0;

        return new Obstacle(
                round2(obstacle.getX()),
                round2(obstacle.getY()),
                round2(obstacle.getZ()),
                round2(radius),
                trimToNull(obstacle.getLabel()),
                trimToNull(obstacle.getSource()),
                roundNullable(obstacle.getEstimatedX()),
                roundNullable(obstacle.getEstimatedY()),
                roundNullable(obstacle.getEstimatedZ()),
                roundNullable(obstacle.getCalibratedX()),
                roundNullable(obstacle.getCalibratedY()),
                roundNullable(obstacle.getCalibratedZ()),
                obstacle.getCalibrated());
    }

    private boolean sameObstacleSet(List<Obstacle> left, List<Obstacle> right) {
        List<Obstacle> a = deduplicate(left);
        List<Obstacle> b = deduplicate(right);
        if (a.size() != b.size()) {
            return false;
        }

        for (int i = 0; i < a.size(); i++) {
            if (!comparableRow(a.get(i)).equals(comparableRow(b.get(i)))) {
                return false;
            }
        }

        return true;
    }

    private String comparableRow(Obstacle obstacle) {
        return String.format(
                Locale.US,
                "%.2f|%.2f|%.2f|%.2f",
                obstacle.getX(),
                obstacle.getY(),
                obstacle.getZ(),
                obstacle.getRadius());
    }

    private String buildKey(Obstacle obstacle) {
        return Math.round(obstacle.getX() / 20.0)
                + ":"
                + Math.round(obstacle.getY() / 20.0)
                + ":"
                + Math.round(obstacle.getZ() / 20.0)
                + ":"
                + Math.round(obstacle.getRadius() / 10.0);
    }

    private double distance(Obstacle a, Obstacle b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        double dz = a.getZ() - b.getZ();
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    private double round2(double value) {
        return Math.round(value * 100.0) / 100.0;
    }

    private Double roundNullable(Double value) {
        return value == null || !Double.isFinite(value) ? null : round2(value);
    }

    private String trimToNull(String value) {
        if (value == null) {
            return null;
        }
        String trimmed = value.trim();
        return trimmed.isEmpty() ? null : trimmed;
    }

    private String safeString(String value) {
        return value == null ? "" : value;
    }

    private String safeNumber(Double value) {
        return value == null ? "" : String.format(Locale.US, "%.2f", value);
    }

    private List<Obstacle> mergeLists(List<Obstacle> left, List<Obstacle> right) {
        List<Obstacle> merged = new ArrayList<>();
        if (left != null) {
            merged.addAll(left);
        }
        if (right != null) {
            merged.addAll(right);
        }
        return merged;
    }

    private void writeObstaclesCsv(String filePath, List<Obstacle> obstacles) {
        if (filePath == null || filePath.isBlank()) {
            return;
        }

        Path path = csvObstacleDataProvider.resolveConfiguredFile(filePath).toPath();
        List<String> lines = new ArrayList<>();
        lines.add("x,y,z,radius");

        for (Obstacle obstacle : obstacles) {
            lines.add(String.format(
                    Locale.US,
                    "%.2f,%.2f,%.2f,%.2f",
                    obstacle.getX(),
                    obstacle.getY(),
                    obstacle.getZ(),
                    obstacle.getRadius()));
        }

        try {
            Path parent = path.getParent();
            if (parent != null) {
                Files.createDirectories(parent);
            }
            Files.write(path, lines, StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new IllegalStateException("Failed to write obstacle CSV: " + filePath, e);
        }
    }

    public record ObstacleSyncResult(
            List<Obstacle> detectedObstacles,
            List<Obstacle> dynamicBirdObstacles,
            boolean changed) {
    }
}
