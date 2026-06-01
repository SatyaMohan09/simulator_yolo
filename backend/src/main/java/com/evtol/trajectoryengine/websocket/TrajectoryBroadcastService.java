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
 * Broadcasts replanned trajectories to all connected WebSocket clients whenever
 * new obstacle detections arrive.  A debounce window (300 ms) collapses rapid
 * bursts of YOLO frames into a single replan so the backend isn't overwhelmed.
 */
@Service
@RequiredArgsConstructor
public class TrajectoryBroadcastService {

    private final SimpMessagingTemplate messagingTemplate;
    private final TrajectoryService trajectoryService;

    private final AtomicBoolean replanPending = new AtomicBoolean(false);
    private final ScheduledExecutorService scheduler =
            Executors.newSingleThreadScheduledExecutor(r -> {
                Thread t = new Thread(r, "trajectory-broadcast");
                t.setDaemon(true);
                return t;
            });

    /**
     * Schedule a replan 300 ms from now.  If another call arrives before the
     * timer fires the flag is already set, so the work is not duplicated.
     */
    public void scheduleReplan() {
        if (replanPending.compareAndSet(false, true)) {
            scheduler.schedule(this::replanAndBroadcast, 300, TimeUnit.MILLISECONDS);
        }
    }

    private void replanAndBroadcast() {
        replanPending.set(false);
        try {
            System.out.println("[WS] Replanning trajectory for broadcast …");
            TrajectoryResponse response = trajectoryService.generateTrajectory(0.1);
            messagingTemplate.convertAndSend("/topic/trajectory", response);
            System.out.printf("[WS] Broadcast complete — %d points%n",
                    response.getTrajectory() != null ? response.getTrajectory().size() : 0);
        } catch (Exception e) {
            System.err.printf("[WS] Replan/broadcast failed: %s%n", e.getMessage());
        }
    }
}
