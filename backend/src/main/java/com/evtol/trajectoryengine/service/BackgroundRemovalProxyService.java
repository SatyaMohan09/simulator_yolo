package com.evtol.trajectoryengine.service;

import jakarta.annotation.PreDestroy;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.core.io.ByteArrayResource;
import org.springframework.http.HttpEntity;
import org.springframework.http.HttpHeaders;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Service;
import org.springframework.util.LinkedMultiValueMap;
import org.springframework.util.MultiValueMap;
import org.springframework.web.client.ResourceAccessException;
import org.springframework.web.client.RestTemplate;
import org.springframework.web.multipart.MultipartFile;

import java.net.URI;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.Duration;
import java.time.Instant;

@Service
public class BackgroundRemovalProxyService {

    private static final Duration STARTUP_TIMEOUT = Duration.ofSeconds(20);

    private final RestTemplate restTemplate = new RestTemplate();

    @Value("${bgremoval.python.command:python}")
    private String pythonCommand;

    @Value("${bgremoval.service.url:http://127.0.0.1:5001/remove-bg}")
    private String pythonUrl;

    @Value("${bgremoval.script.path:../imageprocessing/image/bg-removal-python/bg_service.py}")
    private String scriptPath;

    private Process pythonProcess;

    public synchronized byte[] removeBackground(MultipartFile image, String filename) throws Exception {
        ensurePythonServiceStarted();

        MultiValueMap<String, Object> body = new LinkedMultiValueMap<>();
        body.add("image", new ByteArrayResource(image.getBytes()) {
            @Override
            public String getFilename() {
                return filename;
            }
        });

        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.MULTIPART_FORM_DATA);

        HttpEntity<MultiValueMap<String, Object>> request = new HttpEntity<>(body, headers);
        ResponseEntity<byte[]> response = restTemplate.postForEntity(pythonUrl, request, byte[].class);
        return response.getBody();
    }

    @PreDestroy
    public void stopPythonService() {
        if (pythonProcess != null && pythonProcess.isAlive()) {
            pythonProcess.destroy();
        }
    }

    private synchronized void ensurePythonServiceStarted() throws Exception {
        if (isServiceAvailable()) {
            return;
        }

        Path resolvedScript = resolveScriptPath();
        ProcessBuilder processBuilder = new ProcessBuilder(
                pythonCommand,
                resolvedScript.toString()
        );
        processBuilder.directory(resolvedScript.getParent().toFile());
        processBuilder.redirectErrorStream(true);
        processBuilder.redirectOutput(ProcessBuilder.Redirect.INHERIT);

        pythonProcess = processBuilder.start();
        waitForService();
    }

    private void waitForService() throws Exception {
        Instant deadline = Instant.now().plus(STARTUP_TIMEOUT);
        Exception lastError = null;

        while (Instant.now().isBefore(deadline)) {
            if (pythonProcess != null && !pythonProcess.isAlive()) {
                throw new IllegalStateException("Background removal Python service exited before startup completed.");
            }

            try {
                if (isServiceAvailable()) {
                    return;
                }
            } catch (Exception ex) {
                lastError = ex;
            }

            Thread.sleep(1000);
        }

        throw new IllegalStateException("Timed out waiting for background removal Python service to start.", lastError);
    }

    private boolean isServiceAvailable() {
        try {
            restTemplate.optionsForAllow(URI.create(pythonUrl));
            return true;
        } catch (ResourceAccessException ex) {
            return false;
        }
    }

    private Path resolveScriptPath() {
        Path configuredPath = Paths.get(scriptPath);
        if (configuredPath.isAbsolute()) {
            return configuredPath;
        }

        return Paths.get("")
                .toAbsolutePath()
                .resolve(configuredPath)
                .normalize();
    }
}
