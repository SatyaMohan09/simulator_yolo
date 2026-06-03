package com.evtol.trajectoryengine.websocket;

import com.evtol.trajectoryengine.dto.TrajectoryResponse;
import com.evtol.trajectoryengine.service.TrajectoryService;
import lombok.RequiredArgsConstructor;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.stereotype.Service;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Broadcasts replanned trajectories to all connected WebSocket clients.
 *
 * <p>Two separate replan paths are provided:</p>
 * <ul>
 *   <li>{@link #scheduleFullReplan()} — triggered when static obstacles
 *       (buildings, walls) change.  Runs the complete pipeline: RRT* +
 *       B-spline refit + full resample.  Debounced at 300 ms.</li>
 *   <li>{@link #scheduleBirdReplan()} — triggered when only dynamic bird
 *       obstacles change.  Skips RRT* and only runs D* Lite on the cached
 *       base trajectory.  Debounced at 150 ms (birds move fast).</li>
 * </ul>
 *
 * <p>If a full replan is already pending, an incoming bird replan is
 * silently dropped — the full replan will incorporate the latest bird
 * positions anyway when it eventually fires.</p>
 */
@Service
@RequiredArgsConstructor
public class TrajectoryBroadcastService {

    private final SimpMessagingTemplate messagingTemplate;
    private final TrajectoryService     trajectoryService;

    // ── debounce flags ────────────────────────────────────────────────────────

    private final AtomicBoolean fullReplanPending = new AtomicBoolean(false);
    private final AtomicBoolean birdReplanPending = new AtomicBoolean(false);

    private final ScheduledExecutorService scheduler =
            Executors.newSingleThreadScheduledExecutor(r -> {
                Thread t = new Thread(r, "trajectory-broadcast");
                t.setDaemon(true);
                return t;
            });

    // ── public API ────────────────────────────────────────────────────────────

    /**
     * Schedule a full replan (static obstacles changed).
     * Cancels any pending bird-only replan — the full replan covers both.
     */
    public void scheduleFullReplan() {
        // Cancel a pending bird replan; the full replan subsumes it.
        birdReplanPending.set(false);

        if (fullReplanPending.compareAndSet(false, true)) {
            scheduler.schedule(this::fullReplanAndBroadcast, 300, TimeUnit.MILLISECONDS);
        }
    }

    /**
     * Schedule a bird-only (partial) replan.
     * Ignored when a full replan is already queued.
     */
    public void scheduleBirdReplan() {
        // Full replan already covers birds — no need to duplicate work.
        if (fullReplanPending.get()) {
            return;
        }

        if (birdReplanPending.compareAndSet(false, true)) {
            scheduler.schedule(this::birdReplanAndBroadcast, 150, TimeUnit.MILLISECONDS);
        }
    }

    // ── kept for backward compatibility (e.g. any direct callers) ────────────

    /**
     * @deprecated Use {@link #scheduleFullReplan()} or
     *             {@link #scheduleBirdReplan()} instead.
     */
    @Deprecated
    public void scheduleReplan() {
        scheduleFullReplan();
    }

    // ── internal replan workers ───────────────────────────────────────────────

    private void fullReplanAndBroadcast() {
        fullReplanPending.set(false);
        try {
            System.out.println("[WS] Full replan triggered (static obstacles changed) …");
            TrajectoryResponse response = trajectoryService.generateTrajectory(0.1);
            broadcast(response);
            System.out.printf("[WS] Full replan broadcast complete — %d points%n",
                    response.getTrajectory() != null ? response.getTrajectory().size() : 0);
        } catch (Exception e) {
            System.err.printf("[WS] Full replan failed: %s%n", e.getMessage());
        }
    }

    private void birdReplanAndBroadcast() {
        birdReplanPending.set(false);

        // Guard: if a full replan sneaked in while we were waiting, skip.
        if (fullReplanPending.get()) {
            System.out.println("[WS] Bird replan skipped — full replan already pending.");
            return;
        }

        try {
            System.out.println("[WS] Bird replan triggered (dynamic obstacles only) …");
            TrajectoryResponse response = trajectoryService.generateBirdAvoidanceTrajectory(0.1);
            broadcast(response);
            System.out.printf("[WS] Bird replan broadcast complete — %d points%n",
                    response.getTrajectory() != null ? response.getTrajectory().size() : 0);
        } catch (Exception e) {
            System.err.printf("[WS] Bird replan failed: %s%n", e.getMessage());
        }
    }

    private void broadcast(TrajectoryResponse response) {
        messagingTemplate.convertAndSend("/topic/trajectory", response);
    }
}