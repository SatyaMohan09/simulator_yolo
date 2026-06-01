package com.evtol.trajectoryengine.dto;

import com.evtol.trajectoryengine.domain.Obstacle;

import java.util.List;

public class ObstacleSyncRequest {

    private Integer frameIndex;
    private String source;
    private List<Obstacle> obstacles;
    private List<Obstacle> birdObstacles;

    public Integer getFrameIndex() { return frameIndex; }
    public void setFrameIndex(Integer frameIndex) { this.frameIndex = frameIndex; }

    public String getSource() { return source; }
    public void setSource(String source) { this.source = source; }

    public List<Obstacle> getObstacles() { return obstacles; }
    public void setObstacles(List<Obstacle> obstacles) { this.obstacles = obstacles; }

    public List<Obstacle> getBirdObstacles() { return birdObstacles; }
    public void setBirdObstacles(List<Obstacle> birdObstacles) { this.birdObstacles = birdObstacles; }
}
