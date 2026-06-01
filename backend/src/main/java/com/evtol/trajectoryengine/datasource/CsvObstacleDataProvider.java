package com.evtol.trajectoryengine.datasource;

import com.evtol.trajectoryengine.domain.Obstacle;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

@Component
public class CsvObstacleDataProvider {

    @Value("${trajectory.obstacle.file-path:}")
    private String obstacleFilePath;

    public List<Obstacle> loadObstacles() {
        return loadObstacles(obstacleFilePath);
    }

    public List<Obstacle> loadObstacles(String filePath) {
        List<Obstacle> obstacles = new ArrayList<>();

        if (filePath == null || filePath.isBlank()) {
            return obstacles;
        }

        File file = resolveFile(filePath);
        if (!file.exists()) {
            System.err.println("[CsvObstacleDataProvider] File not found: " + file.getAbsolutePath());
            return obstacles;
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line = reader.readLine(); // skip header

            while ((line = reader.readLine()) != null) {
                if (line.isBlank()) continue;

                String[] values = line.split(",", -1);
                if (values.length < 4) continue;

                double x      = Double.parseDouble(values[0].trim());
                double y      = Double.parseDouble(values[1].trim());
                double z      = Double.parseDouble(values[2].trim());
                double radius = Double.parseDouble(values[3].trim());

                Obstacle obstacle = new Obstacle(x, y, z, radius);
                if (values.length > 4) obstacle.setLabel(blankToNull(values[4]));
                if (values.length > 5) obstacle.setSource(blankToNull(values[5]));
                if (values.length > 6) obstacle.setEstimatedX(parseOptionalDouble(values[6]));
                if (values.length > 7) obstacle.setEstimatedY(parseOptionalDouble(values[7]));
                if (values.length > 8) obstacle.setEstimatedZ(parseOptionalDouble(values[8]));
                if (values.length > 9) obstacle.setCalibratedX(parseOptionalDouble(values[9]));
                if (values.length > 10) obstacle.setCalibratedY(parseOptionalDouble(values[10]));
                if (values.length > 11) obstacle.setCalibratedZ(parseOptionalDouble(values[11]));
                if (values.length > 12) obstacle.setCalibrated(parseOptionalBoolean(values[12]));

                obstacles.add(obstacle);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        return obstacles;
    }

    public File resolveConfiguredFile(String filePath) {
        if (filePath == null || filePath.isBlank()) {
            return null;
        }
        return resolveFile(filePath);
    }

    /**
     * Resolves a file path in this order:
     *  1. As an absolute path (if the path is already absolute — unchanged behaviour)
     *  2. Relative to the directory that contains the running JAR / classes
     *     (i.e. next to the backend executable, which is where backend/data/ lives)
     *  3. Relative to the current working directory as a last resort
     */
    private File resolveFile(String filePath) {
        File f = new File(filePath);
        if (f.isAbsolute()) return f;

        // Locate the directory containing this class / the JAR
        try {
            File jarDir = new File(
                CsvObstacleDataProvider.class
                    .getProtectionDomain()
                    .getCodeSource()
                    .getLocation()
                    .toURI()
            ).getParentFile();

            // When running via `mvn spring-boot:run` the classes live under
            // target/classes — walk up until we find the backend project root
            // (the folder that contains the "data" directory).
            File candidate = jarDir;
            for (int i = 0; i < 5; i++) {
                File dataDir = new File(candidate, "data");
                if (dataDir.isDirectory()) {
                    File resolved = new File(candidate, filePath);
                    if (resolved.exists()) return resolved;
                }
                if (candidate.getParentFile() == null) break;
                candidate = candidate.getParentFile();
            }
        } catch (URISyntaxException ignored) {}

        // Fallback: relative to working directory
        return f;
    }

    private Double parseOptionalDouble(String raw) {
        String value = blankToNull(raw);
        return value == null ? null : Double.parseDouble(value);
    }

    private Boolean parseOptionalBoolean(String raw) {
        String value = blankToNull(raw);
        return value == null ? null : Boolean.parseBoolean(value);
    }

    private String blankToNull(String raw) {
        if (raw == null) {
            return null;
        }
        String trimmed = raw.trim();
        return trimmed.isEmpty() ? null : trimmed;
    }
}
