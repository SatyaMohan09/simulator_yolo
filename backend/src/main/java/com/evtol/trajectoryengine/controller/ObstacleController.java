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

    private final ObstacleRegistryService   obstacleRegistryService;
    private final TrajectoryBroadcastService trajectoryBroadcastService;

    /** Full snapshot: imageProcessing + detected + merged planner list. */
    @GetMapping
    public ObstacleSnapshotResponse getObstacles() {
        return obstacleRegistryService.getSnapshot();
    }

    /**
     * Classified view used by the frontend to colour buildings.
     */
    @GetMapping("/classified")
    public Map<String, List<Obstacle>> getClassifiedObstacles() {
        Map<String, List<Obstacle>> result = new HashMap<>();
        result.put("knownObstacles",    obstacleRegistryService.loadSeedObstacles());
        result.put("detectedObstacles", obstacleRegistryService.loadDetectedObstacles());
        return result;
    }

    /**
     * Demo helper: keep only the first N imageprocessing obstacles as static.
     */
    @PostMapping("/demo/bootstrap-static")
    public ObstacleSnapshotResponse bootstrapStaticScenario(
            @RequestParam(defaultValue = "4") int seedCount) {
        return obstacleRegistryService.bootstrapStaticScenario(seedCount);
    }

    /**
     * Demo helper: reveal additional surprise obstacles at runtime.
     */
    @PostMapping("/demo/reveal-surprises")
    public ObstacleSnapshotResponse revealSurpriseObstacles(
            @RequestParam(defaultValue = "1") int count) {
        return obstacleRegistryService.revealSurpriseObstacles(count);
    }

    /** Returns only YOLO-detected static obstacles. */
    @GetMapping("/detected")
    public List<Obstacle> getDetectedObstacles() {
        return obstacleRegistryService.loadDetectedObstacles();
    }

    /**
     * YOLO service POSTs its detections here every frame.
     *
     * <p>The sync result now clearly tells us what changed:</p>
     * <ul>
     *   <li>Static obstacles changed → full replan (RRT* + B-spline refit)</li>
     *   <li>Only birds changed       → bird replan (D* Lite only, no RRT*)</li>
     *   <li>Nothing changed          → no replan at all</li>
     * </ul>
     */
    @PostMapping("/detections")
    public List<Obstacle> syncDetections(
            @RequestBody(required = false) ObstacleSyncRequest request) {

        List<Obstacle> detections     = request != null && request.getObstacles() != null
                ? request.getObstacles()
                : List.of();
        List<Obstacle> birdDetections = request != null && request.getBirdObstacles() != null
                ? request.getBirdObstacles()
                : List.of();

        ObstacleRegistryService.ObstacleSyncResult syncResult =
                obstacleRegistryService.syncRuntimeObstacles(detections, birdDetections);

        if (syncResult.staticChanged()) {
            // Static buildings changed — full replan required.
            // This also covers birds because the full pipeline reads bird
            // obstacles too, so no separate bird replan is needed.
            trajectoryBroadcastService.scheduleFullReplan();

        } else if (syncResult.birdsChanged()) {
            // Only dynamic bird positions changed — partial replan is enough.
            trajectoryBroadcastService.scheduleBirdReplan();
        }
        // If nothing changed, we do nothing — no unnecessary replan.

        return syncResult.detectedObstacles();
    }

    /** Validate detected obstacles against the frontend ground-truth CSV. */
    @GetMapping("/validation")
    public ObstacleValidationResponse validateDetections() {
        return obstacleRegistryService.validateDetectedAgainstGroundTruth();
    }
}