package com.evtol.trajectoryengine.planning;

import com.evtol.trajectoryengine.domain.Waypoint;

import java.util.List;

public record DStarResult(
        List<Waypoint> waypoints,
        int affectedStartIndex,
        int affectedEndIndex,
        boolean modified) {
}