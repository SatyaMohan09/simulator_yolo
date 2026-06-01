package com.evtol.trajectoryengine.domain;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

@JsonIgnoreProperties(ignoreUnknown = true)
public class Obstacle {

    private double x;
    private double y;
    private double z;
    private double radius;
    private String label;
    private String source;
    private Double estimatedX;
    private Double estimatedY;
    private Double estimatedZ;
    private Double calibratedX;
    private Double calibratedY;
    private Double calibratedZ;
    private Boolean calibrated;

    public Obstacle() {
    }

    public Obstacle(double x, double y, double z, double radius) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.radius = radius;
    }

    public Obstacle(
            double x,
            double y,
            double z,
            double radius,
            String label,
            String source,
            Double estimatedX,
            Double estimatedY,
            Double estimatedZ,
            Double calibratedX,
            Double calibratedY,
            Double calibratedZ,
            Boolean calibrated) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.radius = radius;
        this.label = label;
        this.source = source;
        this.estimatedX = estimatedX;
        this.estimatedY = estimatedY;
        this.estimatedZ = estimatedZ;
        this.calibratedX = calibratedX;
        this.calibratedY = calibratedY;
        this.calibratedZ = calibratedZ;
        this.calibrated = calibrated;
    }

    public double getX() { return x; }
    public void setX(double x) { this.x = x; }

    public double getY() { return y; }
    public void setY(double y) { this.y = y; }

    public double getZ() { return z; }
    public void setZ(double z) { this.z = z; }

    public double getRadius() { return radius; }
    public void setRadius(double radius) { this.radius = radius; }

    public String getLabel() { return label; }
    public void setLabel(String label) { this.label = label; }

    public String getSource() { return source; }
    public void setSource(String source) { this.source = source; }

    public Double getEstimatedX() { return estimatedX; }
    public void setEstimatedX(Double estimatedX) { this.estimatedX = estimatedX; }

    public Double getEstimatedY() { return estimatedY; }
    public void setEstimatedY(Double estimatedY) { this.estimatedY = estimatedY; }

    public Double getEstimatedZ() { return estimatedZ; }
    public void setEstimatedZ(Double estimatedZ) { this.estimatedZ = estimatedZ; }

    public Double getCalibratedX() { return calibratedX; }
    public void setCalibratedX(Double calibratedX) { this.calibratedX = calibratedX; }

    public Double getCalibratedY() { return calibratedY; }
    public void setCalibratedY(Double calibratedY) { this.calibratedY = calibratedY; }

    public Double getCalibratedZ() { return calibratedZ; }
    public void setCalibratedZ(Double calibratedZ) { this.calibratedZ = calibratedZ; }

    public Boolean getCalibrated() { return calibrated; }
    public void setCalibrated(Boolean calibrated) { this.calibrated = calibrated; }
}
