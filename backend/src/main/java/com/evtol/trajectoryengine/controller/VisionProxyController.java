package com.evtol.trajectoryengine.controller;

import com.evtol.trajectoryengine.service.BackgroundRemovalProxyService;
import com.evtol.trajectoryengine.service.LocalVisionProcessingService;

import java.io.ByteArrayOutputStream;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.util.Map;

import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.CrossOrigin;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.multipart.MultipartFile;

@RestController
@RequestMapping("/api/vision")
@CrossOrigin(origins = "*")
public class VisionProxyController {

    private static final String YOLO_SERVICE_BASE = "http://localhost:5050";

    private final HttpClient http = HttpClient.newBuilder()
            .connectTimeout(Duration.ofSeconds(3))
            .build();
    private final BackgroundRemovalProxyService backgroundRemovalProxyService;
    private final LocalVisionProcessingService localVisionProcessingService;

    public VisionProxyController(
            BackgroundRemovalProxyService backgroundRemovalProxyService,
            LocalVisionProcessingService localVisionProcessingService
    ) {
        this.backgroundRemovalProxyService = backgroundRemovalProxyService;
        this.localVisionProcessingService = localVisionProcessingService;
    }

    @PostMapping("/edge")
    public ResponseEntity<?> edge(@RequestBody Map<String, Object> body) {
        return processLocally(body, true);
    }

    @PostMapping("/threshold")
    public ResponseEntity<?> threshold(@RequestBody Map<String, Object> body) {
        return processLocally(body, false);
    }

    @PostMapping("/bg-reveal")
    public ResponseEntity<?> bgReveal(@RequestBody Map<String, Object> body) {
        return proxyJson(YOLO_SERVICE_BASE + "/vision/bg-reveal", body);
    }

    /**
     * Proxies to python BG service (expects multipart field name "image").
     * Client can call:
     *   - multipart/form-data with "image" file
     *   - optional ?filename=frame.png
     */
    @PostMapping(value = "/remove-bg", consumes = MediaType.MULTIPART_FORM_DATA_VALUE)
    public ResponseEntity<byte[]> removeBg(
            @RequestParam("image") MultipartFile image,
            @RequestParam(value = "filename", defaultValue = "frame.png") String filename) {
        try {
            HttpHeaders headers = new HttpHeaders();
            headers.setContentType(MediaType.IMAGE_PNG);
            return new ResponseEntity<>(
                    backgroundRemovalProxyService.removeBackground(image, filename),
                    headers,
                    HttpStatus.OK
            );
        } catch (Exception e) {
            return ResponseEntity.status(HttpStatus.BAD_GATEWAY)
                    .contentType(MediaType.APPLICATION_JSON)
                    .body(("{\"error\":\"" + safe(e.getMessage()) + "\"}").getBytes(StandardCharsets.UTF_8));
        }
    }

    private ResponseEntity<?> proxyJson(String url, Map<String, Object> body) {
        try {
            String json = JsonMini.encode(body);
            HttpRequest req = HttpRequest.newBuilder()
                    .uri(URI.create(url))
                    .timeout(Duration.ofSeconds(10))
                    .header("Content-Type", "application/json")
                    .POST(HttpRequest.BodyPublishers.ofString(json, StandardCharsets.UTF_8))
                    .build();
            HttpResponse<String> resp = http.send(req, HttpResponse.BodyHandlers.ofString(StandardCharsets.UTF_8));
            return ResponseEntity.status(resp.statusCode())
                    .contentType(MediaType.APPLICATION_JSON)
                    .body(resp.body());
        } catch (Exception e) {
            return ResponseEntity.status(HttpStatus.BAD_GATEWAY)
                    .contentType(MediaType.APPLICATION_JSON)
                    .body("{\"error\":\"" + safe(e.getMessage()) + "\"}");
        }
    }

    private ResponseEntity<?> processLocally(Map<String, Object> body, boolean edgeMode) {
        try {
            Object frame = body.get("frame");
            if (!(frame instanceof String frameData) || frameData.isBlank()) {
                return ResponseEntity.badRequest()
                        .contentType(MediaType.APPLICATION_JSON)
                        .body("{\"error\":\"Missing frame\"}");
            }

            String image = edgeMode
                    ? localVisionProcessingService.edgeDetection(frameData)
                    : localVisionProcessingService.thresholdSegmentation(frameData);

            return ResponseEntity.ok()
                    .contentType(MediaType.APPLICATION_JSON)
                    .body("{\"image\":\"" + safe(image) + "\"}");
        } catch (IllegalArgumentException e) {
            return ResponseEntity.badRequest()
                    .contentType(MediaType.APPLICATION_JSON)
                    .body("{\"error\":\"" + safe(e.getMessage()) + "\"}");
        } catch (Exception e) {
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR)
                    .contentType(MediaType.APPLICATION_JSON)
                    .body("{\"error\":\"" + safe(e.getMessage()) + "\"}");
        }
    }

    private static String safe(String s) {
        if (s == null) return "unknown";
        return s.replace("\\", "\\\\").replace("\"", "\\\"");
    }

    record MultipartBody(String boundary, byte[] body) {}

    private static MultipartBody buildMultipart(String field, String filename, byte[] content, String contentType) throws Exception {
        String boundary = "----evtolBoundary" + System.currentTimeMillis();
        String ct = (contentType == null || contentType.isBlank()) ? "application/octet-stream" : contentType;

        ByteArrayOutputStream out = new ByteArrayOutputStream();
        out.write(("--" + boundary + "\r\n").getBytes(StandardCharsets.UTF_8));
        out.write(("Content-Disposition: form-data; name=\"" + field + "\"; filename=\"" + filename + "\"\r\n")
                .getBytes(StandardCharsets.UTF_8));
        out.write(("Content-Type: " + ct + "\r\n\r\n").getBytes(StandardCharsets.UTF_8));
        out.write(content);
        out.write("\r\n".getBytes(StandardCharsets.UTF_8));
        out.write(("--" + boundary + "--\r\n").getBytes(StandardCharsets.UTF_8));

        return new MultipartBody(boundary, out.toByteArray());
    }

    /**
     * Minimal JSON encoder for simple maps (string/number/bool/null and nested maps).
     * Keeps backend dependency-free (no Jackson wiring changes).
     */
    static class JsonMini {
        static String encode(Object v) {
            if (v == null) return "null";
            if (v instanceof String s) return "\"" + escape(s) + "\"";
            if (v instanceof Number || v instanceof Boolean) return String.valueOf(v);
            if (v instanceof Map<?, ?> m) {
                StringBuilder sb = new StringBuilder();
                sb.append("{");
                boolean first = true;
                for (Map.Entry<?, ?> e : m.entrySet()) {
                    if (!(e.getKey() instanceof String)) continue;
                    if (!first) sb.append(",");
                    first = false;
                    sb.append("\"").append(escape((String) e.getKey())).append("\":").append(encode(e.getValue()));
                }
                sb.append("}");
                return sb.toString();
            }
            // Best-effort for other types (arrays/lists) if they show up
            if (v instanceof Iterable<?> it) {
                StringBuilder sb = new StringBuilder();
                sb.append("[");
                boolean first = true;
                for (Object o : it) {
                    if (!first) sb.append(",");
                    first = false;
                    sb.append(encode(o));
                }
                sb.append("]");
                return sb.toString();
            }
            return "\"" + escape(String.valueOf(v)) + "\"";
        }

        private static String escape(String s) {
            return s.replace("\\", "\\\\")
                    .replace("\"", "\\\"")
                    .replace("\n", "\\n")
                    .replace("\r", "\\r")
                    .replace("\t", "\\t");
        }
    }
}
