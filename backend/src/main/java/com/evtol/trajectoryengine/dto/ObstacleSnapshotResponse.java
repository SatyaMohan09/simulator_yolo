package com.evtol.trajectoryengine.dto;

import com.evtol.trajectoryengine.domain.Obstacle;
import java.util.List;

public class ObstacleSnapshotResponse {

    private final List<Obstacle> imageProcessingObstacles;
    private final List<Obstacle> seedObstacles;
    private final List<Obstacle> detectedObstacles;
    private final List<Obstacle> plannerObstacles;
    private final int imageProcessingCount;
    private final int seedCount;
    private final int detectedCount;
    /** How many obstacles the backend is currently planning around. */
    private final int totalPlannerCount;

    public ObstacleSnapshotResponse(
            List<Obstacle> imageProcessingObstacles,
            List<Obstacle> seedObstacles,
            List<Obstacle> detectedObstacles,
            List<Obstacle> plannerObstacles) {
        this.imageProcessingObstacles = imageProcessingObstacles;
        this.seedObstacles     = seedObstacles;
        this.detectedObstacles = detectedObstacles;
        this.plannerObstacles  = plannerObstacles;
        this.imageProcessingCount = imageProcessingObstacles != null ? imageProcessingObstacles.size() : 0;
        this.seedCount = seedObstacles != null ? seedObstacles.size() : 0;
        this.detectedCount = detectedObstacles != null ? detectedObstacles.size() : 0;
        this.totalPlannerCount = plannerObstacles != null ? plannerObstacles.size() : 0;
    }

    public List<Obstacle> getImageProcessingObstacles() { return imageProcessingObstacles; }
    public List<Obstacle> getSeedObstacles()     { return seedObstacles; }
    public List<Obstacle> getDetectedObstacles() { return detectedObstacles; }
    public List<Obstacle> getPlannerObstacles()  { return plannerObstacles; }
    public int getImageProcessingCount()         { return imageProcessingCount; }
    public int getSeedCount()                    { return seedCount; }
    public int getDetectedCount()                { return detectedCount; }
    public int getTotalPlannerCount()            { return totalPlannerCount; }
}
