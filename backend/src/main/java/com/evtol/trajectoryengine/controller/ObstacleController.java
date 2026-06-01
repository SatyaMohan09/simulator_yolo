package com.evtol.trajectoryengine.controller;

import com.evtol.trajectoryengine.domain.Obstacle;
import com.evtol.trajectoryengine.dto.ObstacleSnapshotResponse;
import com.evtol.trajectoryengine.dto.ObstacleSyncRequest;
import com.evtol.trajectoryengine.dto.ObstacleValidationResponse;
import com.evtol.trajectoryengine.service.ObstacleRegistryService;
import com.evtol.trajectoryengine.websocket.TrajectoryBroadcastService;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.*;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@RestController
@RequestMapping("/api/obstacles")
@CrossOrigin(origins = "*")
@RequiredArgsConstructor
public class ObstacleController {

    private final ObstacleRegistryService obstacleRegistryService;
    private final TrajectoryBroadcastService trajectoryBroadcastService;

    /** Full snapshot: imageProcessing + detected + merged planner list. */
    @GetMapping
    public ObstacleSnapshotResponse getObstacles() {
        return obstacleRegistryService.getSnapshot();
    }

    /**
     * Classified view used by the frontend to colour buildings.
     * Returns the backend-known static set and the current YOLO-detected set
     * so the UI can distinguish white (known), orange (surprise), red (detected).
     */
    @GetMapping("/classified")
    public Map<String, List<Obstacle>> getClassifiedObstacles() {
        Map<String, List<Obstacle>> result = new HashMap<>();
        result.put("knownObstacles",    obstacleRegistryService.loadSeedObstacles());
        result.put("detectedObstacles", obstacleRegistryService.loadDetectedObstacles());
        return result;
    }

    /**
     * Demo helper: keep only the first N imageprocessing obstacles as static,
     * known backend obstacles and clear all surprise detections.
     */
    @PostMapping("/demo/bootstrap-static")
    public ObstacleSnapshotResponse bootstrapStaticScenario(
            @RequestParam(defaultValue = "4") int seedCount) {
        return obstacleRegistryService.bootstrapStaticScenario(seedCount);
    }

    /**
     * Demo helper: reveal additional obstacles from the imageprocessing feed as
     * runtime surprises so the planner must react mid-flight.
     */
    @PostMapping("/demo/reveal-surprises")
    public ObstacleSnapshotResponse revealSurpriseObstacles(
            @RequestParam(defaultValue = "1") int count) {
        return obstacleRegistryService.revealSurpriseObstacles(count);
    }

    /**
     * Returns only YOLO-detected obstacles.
     * Frontend overlays these in a different colour without needing the seed list.
     */
    @GetMapping("/detected")
    public List<Obstacle> getDetectedObstacles() {
        return obstacleRegistryService.loadDetectedObstacles();
    }

    /**
     * YOLO service POSTs its detections here every frame.
     * Replaces the current detected set, mirrors bird detections into
     * dynamic_obstacle2.csv, and triggers a trajectory replan whenever the
     * runtime obstacle state changes.
     */
    @PostMapping("/detections")
    public List<Obstacle> syncDetections(
            @RequestBody(required = false) ObstacleSyncRequest request) {
        List<Obstacle> detections = request != null && request.getObstacles() != null
                ? request.getObstacles()
                : List.of();
        List<Obstacle> birdDetections = request != null && request.getBirdObstacles() != null
                ? request.getBirdObstacles()
                : List.of();

        ObstacleRegistryService.ObstacleSyncResult syncResult =
                obstacleRegistryService.syncRuntimeObstacles(detections, birdDetections);

        if (syncResult.changed()) {
            trajectoryBroadcastService.scheduleReplan();
        }

        return syncResult.detectedObstacles();
    }

    /** Validate detected obstacles against the frontend ground-truth CSV. */
    @GetMapping("/validation")
    public ObstacleValidationResponse validateDetections() {
        return obstacleRegistryService.validateDetectedAgainstGroundTruth();
    }
}
