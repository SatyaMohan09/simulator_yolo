package com.evtol.trajectoryengine.dto;

public class ObstacleValidationResponse {

    private final int groundTruthCount;
    private final int detectedCount;
    private final int matchedCount;
    private final int missedCount;
    private final int unmatchedDetectionCount;
    private final double matchThresholdM;
    private final Double meanErrorM;
    private final Double maxErrorM;

    public ObstacleValidationResponse(
            int groundTruthCount,
            int detectedCount,
            int matchedCount,
            int missedCount,
            int unmatchedDetectionCount,
            double matchThresholdM,
            Double meanErrorM,
            Double maxErrorM) {
        this.groundTruthCount = groundTruthCount;
        this.detectedCount = detectedCount;
        this.matchedCount = matchedCount;
        this.missedCount = missedCount;
        this.unmatchedDetectionCount = unmatchedDetectionCount;
        this.matchThresholdM = matchThresholdM;
        this.meanErrorM = meanErrorM;
        this.maxErrorM = maxErrorM;
    }

    public int getGroundTruthCount() {
        return groundTruthCount;
    }

    public int getDetectedCount() {
        return detectedCount;
    }

    public int getMatchedCount() {
        return matchedCount;
    }

    public int getMissedCount() {
        return missedCount;
    }

    public int getUnmatchedDetectionCount() {
        return unmatchedDetectionCount;
    }

    public double getMatchThresholdM() {
        return matchThresholdM;
    }

    public Double getMeanErrorM() {
        return meanErrorM;
    }

    public Double getMaxErrorM() {
        return maxErrorM;
    }
}
