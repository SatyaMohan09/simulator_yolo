package com.evtol.trajectoryengine.datasource;

import com.evtol.trajectoryengine.domain.Waypoint;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

@Component
public class CsvWaypointDataProvider {

    @Value("${trajectory.waypoint.file-path:../data-generation/sample-data/evtol_trajectory_final(1).csv}")
    private String waypointFilePath;

    public List<Waypoint> loadWaypoints() {

        List<Waypoint> waypoints = new ArrayList<>();

        try {
            File waypointFile = resolveFile(waypointFilePath);

            BufferedReader reader = new BufferedReader(
                    new FileReader(waypointFile));

            String line;

            reader.readLine(); // skip header

            while ((line = reader.readLine()) != null) {

                String[] values = line.split(",");

                double t = Double.parseDouble(values[0]);
                double x = Double.parseDouble(values[1]);
                double y = Double.parseDouble(values[2]);
                double z = Double.parseDouble(values[3]);

                waypoints.add(new Waypoint(t, x, y, z));
            }

            reader.close();

        } catch (Exception e) {
            e.printStackTrace();
        }

        // to be removed later (just for testing)
        // for (Waypoint wp : waypoints) {
        // System.out.println(
        // "t=" + wp.getT() +
        // ", x=" + wp.getX() +
        // ", y=" + wp.getY() +
        // ", z=" + wp.getZ()
        // );
        // }

        return waypoints;
    }

    private File resolveFile(String filePath) {
        File file = new File(filePath);
        if (file.isAbsolute()) {
            return file;
        }

        try {
            File jarDir = new File(
                    CsvWaypointDataProvider.class
                            .getProtectionDomain()
                            .getCodeSource()
                            .getLocation()
                            .toURI()
            ).getParentFile();

            File candidate = jarDir;
            for (int i = 0; i < 6; i++) {
                File resolved = new File(candidate, filePath);
                if (resolved.exists()) {
                    return resolved;
                }
                if (candidate.getParentFile() == null) {
                    break;
                }
                candidate = candidate.getParentFile();
            }
        } catch (URISyntaxException ignored) {
        }

        return file;
    }
}
