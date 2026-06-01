import * as THREE from "three";
import React from "react";
import {
  useEffect,
  useRef,
  useState,
  forwardRef,
  useImperativeHandle,
} from "react";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";
import * as SkeletonUtils from "three/examples/jsm/utils/SkeletonUtils.js";
import { MissionStateMachine, MissionStatus } from "./missionStatus";
import VertiportDataPanel from "./VertiportDataPanel";
import { getTheme } from "../themes/environmentThemes";
import birdData from "../data/dynamic_obstacle2.json";

let animationId;
const UnifiedVisualizationScene = forwardRef(
  ({ cameraMode = "OVERVIEW", missionMode = true, onPositionUpdate, onMissionUpdate, theme = "daylight" }, ref) => {
    const containerRef = useRef();
    const perspectiveCameraRef = useRef();
    const controlsRef = useRef();
    const rendererRef = useRef();

    // Theme lighting references
    const ambientLightRef = useRef(null);
    const directionalLightRef = useRef(null);
    const hemisphereLightRef = useRef(null);

    const personRef = useRef();
    const evtolRef = useRef();
    const fpvRef = useRef();
    const worldFixedConfigRef = useRef({
      position: null,
      target: null,
    });

    // screenshots now store objects: { dataUrl, frameIndex, obstacles, annotatedImg }
    const [screenshots, setScreenshots] = useState([]);
    const [selectedImage, setSelectedImage] = useState(null);
    const [showPanel, setShowPanel] = useState(true);

    // Vision previews (edge / threshold / bg reveal / bg removal)
    const [visionMode, setVisionMode] = useState("original"); // original|annotated|edge|threshold|bg-reveal|remove-bg
    const [visionPreviewUrl, setVisionPreviewUrl] = useState(null);

    // ── DOWNWARD CAMERA & VERTIPORT VISION ────────────────────────────────────
    const downCamCanvasRef = useRef(null);
    const downCamRendererRef = useRef(null);
    const downCamRef = useRef(null);        // PerspectiveCamera pointing down
    const [heightRadiusLog, setHeightRadiusLog] = useState([]);
    const lastLogHeightRef = useRef(null);
    const LOG_HEIGHT_STEP = 5;              // log a row every 5 m of altitude change
    const VERTIPAD_REAL_RADIUS = 75;        // matches CylinderGeometry radius in scene units
    // focal length proxy: camera is mounted 0 units below evtol center, FOV=80°
    const DOWN_CAM_FOV = 80;
    const DOWN_CAM_HEIGHT_OFFSET = -5;      // below evtol origin

    // ── AUTO-CAPTURE: starts on play(), stops on pause()/reset() ──────────────
    const autoCaptureRef = useRef(false);
    const lastCaptureRef = useRef(0);      // elapsed seconds since last capture
    const frameIndexRef = useRef(0);
    const CAPTURE_INTERVAL = 0.1;            // 10 fps

    // ── YOLO obstacle detection state ─────────────────────────────────────────
    const YOLO_URL = "http://localhost:5050/detect";
    const [latestObstacles, setLatestObstacles] = useState([]);
    const [obstacleHistory, setObstacleHistory] = useState([]);

    const lineGeoRef = useRef();
    const lineRef = useRef(null);
    const forwardLineGeoRef = useRef(null);
    const backwardLineGeoRef = useRef(null);
    const forwardLineRef = useRef(null);
    const backwardLineRef = useRef(null);
    const forwardTrajectoryRef = useRef(null);
    const pathSplitIndexRef = useRef(0);
    const trajectoryRef = useRef([]);
    const trajectoryVectorsRef = useRef([]);
    const sceneRef = useRef(null);
    const initialFlightPosRef = useRef(null);
    const trajectoryReadyRef = useRef(false);
    const obstacleBuildingsRef = useRef([]); // ALL visual buildings (for 3D rendering)
    const birdRef = useRef(null);
    const birdPathRef = useRef([]);
    const birdIndexRef = useRef(0);
    const birdDirectionRef = useRef(1);
    const knownObstaclesRef    = useRef([]); // seed + YOLO-confirmed (for collision avoidance)
    const seedObstaclesRef     = useRef([]); // initial seed only (kept for reset)
    const detectedObstacleKeysRef = useRef(new Set());
    const detectionCandidateCountsRef = useRef(new Map());
    const replanInFlightRef = useRef(false);
    const originalTrajectoryRef = useRef(null); // stores pre-replan path line
    const originalLineRef = useRef(null);
    const detectedObstacleMeshesRef = useRef([]);
    const fovConeRef = useRef(null);
    const [isHovering, setIsHovering] = useState(false);
    const hoverTimerRef = useRef(0);
    const REPLAN_TIMEOUT_S = 5.0; // hover if replan takes longer than this
    const replanTrajectoryFromDetectionsRef = useRef(async () => {});
    const proactiveReplanRef = useRef(() => {});
    const markSurpriseDetectedRef = useRef(() => {});
    const ingestPlannerObstaclesRef = useRef(async () => {});
    const loadTrajectoryRef = useRef(async () => false);
    const lastProactiveCheckRef = useRef(0);
    // Index of the last detour's postIdx — proactive check is suppressed until
    // the eVTOL passes this point (prevents cascade detours in the same zone)
    const lastDetourPostIdxRef = useRef(-1);

    const [alerts, setAlerts] = useState([]);
    const [replanStatus, setReplanStatus] = useState("IDLE");
    const [validationResult, setValidationResult]   = useState(null);
    const [validationLoading, setValidationLoading] = useState(false);

    // ── VERIFICATION / CLASSIFICATION STATE ───────────────────────────────────
    // Tracks which buildings are backend-known vs. "surprise" (frontend-only)
    // and how many surprise obstacles YOLO has detected so far.
    const [backendKnownCount,    setBackendKnownCount]    = useState(0);
    const [surpriseObstacles,    setSurpriseObstacles]    = useState([]);   // [{id,x,z,radius}]
    const [detectedSurpriseIds,  setDetectedSurpriseIds]  = useState(new Set());
    const obstacleClassMapRef  = useRef({});  // id → 'known' | 'surprise'
    const buildingMeshMapRef   = useRef({});  // id → THREE.Object3D
    const detectionRingMapRef  = useRef({});  // id → THREE.Mesh (red detection ring)
    const wsStompRef           = useRef(null);
    const [wsConnected,        setWsConnected]        = useState(false);

    const missionStateRef = useRef(new MissionStateMachine());
    const [missionStatus, setMissionStatus] = useState(MissionStatus.IDLE);
    const cameraModeRef = useRef(cameraMode);

    const sim = useRef({
      isRunning: false,
      t: 0,
      index: 0,
      playbackSpeed: 1,
      waitTimer: 0,
      isLanded: false,
      personPicked: false,
      pathCompleted: false,
      returning: false,
      postPickupAltTimer: 0,
    });

    // ===== OFFSETS & COORDINATES =====
    const groundLevel = 0;
    const evtolHeightOffset = 15; // Vertical offset to prevent sinking
    const stationCenter = new THREE.Vector3(-1400, groundLevel, 350);

    const startPos = new THREE.Vector3(
      stationCenter.x - 90,
      evtolHeightOffset,
      stationCenter.z - 90,
    );
    const landingPadPos = new THREE.Vector3(3420, 0, 500);


    const personPos = landingPadPos.clone().add(new THREE.Vector3(-50, 0, -60));

    function getInitialFlightPosition() {
      return initialFlightPosRef.current?.clone() ?? startPos.clone();
    }

    function getTrajectoryTransform(traj) {
      if (!Array.isArray(traj) || traj.length < 2) return null;

      const rawStart = traj[0];
      let rawEnd = traj[traj.length - 1];
      let maxDistanceSq = -1;

      traj.forEach((point) => {
        const dx = point.x - rawStart.x;
        const dz = point.z - rawStart.z;
        const distanceSq = dx * dx + dz * dz;

        if (distanceSq > maxDistanceSq) {
          maxDistanceSq = distanceSq;
          rawEnd = point;
        }
      });

      const rawDelta = new THREE.Vector2(
        rawEnd.x - rawStart.x,
        rawEnd.z - rawStart.z,
      );
      const targetDelta = new THREE.Vector2(
        landingPadPos.x - startPos.x,
        landingPadPos.z - startPos.z,
      );

      const rawDistance = rawDelta.length();
      const targetDistance = targetDelta.length();
      const scale =
        rawDistance > 0.0001 ? targetDistance / rawDistance : 1;

      const rawAngle = Math.atan2(rawDelta.y, rawDelta.x);
      const targetAngle = Math.atan2(targetDelta.y, targetDelta.x);
      const rotation = targetAngle - rawAngle;

      return {
        rawStart,
        scale,
        rotation,
      };
    }

    function transformTrajectoryToMissionSpace(traj) {
      if (!Array.isArray(traj) || traj.length < 2) return [];

      const first = traj[0];
      const last = traj[traj.length - 1];
      const alreadyAligned =
        Math.abs(first.x - startPos.x) < 25 &&
        Math.abs(first.z - startPos.z) < 25 &&
        Math.abs(last.x - landingPadPos.x) < 25 &&
        Math.abs(last.z - landingPadPos.z) < 25;

      if (alreadyAligned) {
        return traj.map((point) => ({
          ...point,
          y: point.y + evtolHeightOffset,
        }));
      }

      const transform = getTrajectoryTransform(traj);
      if (!transform) return [];

      const { rawStart, scale, rotation } = transform;
      const cos = Math.cos(rotation);
      const sin = Math.sin(rotation);

      return traj.map((point) => {
        const localX = (point.x - rawStart.x) * scale;
        const localZ = (point.z - rawStart.z) * scale;

        const rotatedX = localX * cos - localZ * sin;
        const rotatedZ = localX * sin + localZ * cos;

        return {
          ...point,
          x: startPos.x + rotatedX,
          y: point.y + evtolHeightOffset,
          z: startPos.z + rotatedZ,
        };
      });
    }

    function createSeparatedReturnLeg(forwardLeg) {
      if (!Array.isArray(forwardLeg) || forwardLeg.length < 2) return [];

      const returnSource = forwardLeg.slice(0, -1).reverse();
      const first = forwardLeg[0];
      const last = forwardLeg[forwardLeg.length - 1];
      const dx = first.x - last.x;
      const dz = first.z - last.z;
      const len = Math.sqrt(dx * dx + dz * dz) || 1;
      const perpX = -dz / len;
      const perpZ = dx / len;
      const offset = 260;

      return returnSource.map((point, index) => {
        const progress = returnSource.length > 1
          ? index / (returnSource.length - 1)
          : 0;
        const taper = Math.sin(Math.PI * progress);

        return {
          ...point,
          x: point.x + perpX * offset * taper,
          z: point.z + perpZ * offset * taper,
          t: (last.t ?? forwardLeg.length - 1) + index + 1,
        };
      });
    }

    function ensureRoundTripTrajectory(traj) {
      if (!Array.isArray(traj) || traj.length < 2) return traj;

      const last = traj[traj.length - 1];
      const endsAtStart =
        Math.abs(last.x - startPos.x) < 120 &&
        Math.abs(last.z - startPos.z) < 120;

        if (endsAtStart) {
          return traj;
        }

      if (!forwardTrajectoryRef.current) {
        forwardTrajectoryRef.current = traj;
      }

      const returnLeg = createSeparatedReturnLeg(traj);

      return [...traj, ...returnLeg];
    }

    function mapObstacleToRawSpace(obstacle, transform) {
      if (!transform) return obstacle;

      const { rawStart, scale, rotation } = transform;
      const dx = obstacle.x - startPos.x;
      const dz = obstacle.z - startPos.z;

      const cos = Math.cos(rotation);
      const sin = Math.sin(rotation);

      const localX = (dx * cos + dz * sin) / scale;
      const localZ = (-dx * sin + dz * cos) / scale;

      return {
        x: rawStart.x + localX,
        y: obstacle.y,
        z: rawStart.z + localZ,
        radius: obstacle.radius / scale,
      };
    }

    function getWorldFixedTarget() {
      return startPos.clone().lerp(landingPadPos, 0.5).setY(120);
    }

    function getWorldFixedPosition(target) {
      return target.clone().add(new THREE.Vector3(2200, 1800, 2200));
    }

    function getExactVisibleObstacles(camera) {
      if (!camera) {
        return [];
      }

      camera.updateMatrixWorld();
      camera.updateProjectionMatrix();

      const projectionMatrix = new THREE.Matrix4().multiplyMatrices(
        camera.projectionMatrix,
        camera.matrixWorldInverse,
      );
      const frustum = new THREE.Frustum().setFromProjectionMatrix(projectionMatrix);

      const buildingObstacles = obstacleBuildingsRef.current
        .map((obstacle) => {
          const worldPoint = new THREE.Vector3(obstacle.x, obstacle.y, obstacle.z);
          const sphere = new THREE.Sphere(
            worldPoint.clone(),
            Math.max(obstacle.radius || 0, 120),
          );

          if (!frustum.intersectsSphere(sphere)) {
            return null;
          }

          const cameraPoint = worldPoint.clone().applyMatrix4(camera.matrixWorldInverse);
          const projected = worldPoint.clone().project(camera);
          const screenX = ((projected.x + 1) / 2) * (rendererRef.current?.domElement.width ?? 1);
          const screenY = ((1 - projected.y) / 2) * (rendererRef.current?.domElement.height ?? 1);

          return {
            id: obstacle.id ?? "OBS-UNK",
            label: "ground-truth",
            confidence: 1,
            X_world: Number(obstacle.x.toFixed(2)),
            Y_world: Number(obstacle.y.toFixed(2)),
            Z_world: Number(obstacle.z.toFixed(2)),
            X_camera: Number(cameraPoint.x.toFixed(2)),
            Y_camera: Number(cameraPoint.y.toFixed(2)),
            Z_camera: Number(cameraPoint.z.toFixed(2)),
            u: Number(screenX.toFixed(1)),
            v: Number(screenY.toFixed(1)),
            radius: Number((obstacle.radius ?? 0).toFixed(2)),
            distance: Number(
              camera.position.distanceTo(worldPoint).toFixed(2),
            ),
            source: "frontend-ground-truth",
          };
        })
        .filter(Boolean);

      const bird = birdRef.current;
      const birdObstacles = [];
      if (bird) {
        const worldPoint = bird.position.clone();
        const sphere = new THREE.Sphere(worldPoint.clone(), 60);
        if (frustum.intersectsSphere(sphere)) {
          const cameraPoint = worldPoint.clone().applyMatrix4(camera.matrixWorldInverse);
          const projected = worldPoint.clone().project(camera);
          const screenX = ((projected.x + 1) / 2) * (rendererRef.current?.domElement.width ?? 1);
          const screenY = ((1 - projected.y) / 2) * (rendererRef.current?.domElement.height ?? 1);

          birdObstacles.push({
            id: "BIRD-DYNAMIC-1",
            label: "bird",
            confidence: 1,
            X_world: Number(worldPoint.x.toFixed(2)),
            Y_world: Number(worldPoint.y.toFixed(2)),
            Z_world: Number(worldPoint.z.toFixed(2)),
            X_camera: Number(cameraPoint.x.toFixed(2)),
            Y_camera: Number(cameraPoint.y.toFixed(2)),
            Z_camera: Number(cameraPoint.z.toFixed(2)),
            u: Number(screenX.toFixed(1)),
            v: Number(screenY.toFixed(1)),
            radius: 55,
            distance: Number(camera.position.distanceTo(worldPoint).toFixed(2)),
            source: "dynamic_obstacles2",
          });
        }
      }

      return [...buildingObstacles, ...birdObstacles]
        .sort((a, b) => a.distance - b.distance);
    }

    function makeDetectionKey(obstacle) {
      const x = Math.round((obstacle.X_world ?? obstacle.x ?? 0) / 120);
      const z = Math.round((obstacle.Z_world ?? obstacle.z ?? 0) / 120);
      const radius = Math.round((obstacle.radius ?? 155) / 20);
      return `${x}:${z}:${radius}`;
    }

    function findNearestTrajectoryIndex(
      currentPosition,
      trajectoryVectors,
      options = {},
    ) {
      if (!currentPosition || !Array.isArray(trajectoryVectors) || trajectoryVectors.length === 0) {
        return 0;
      }

      const {
        minIndex = 0,
        maxIndex = trajectoryVectors.length - 1,
      } = options;

      const safeMinIndex = Math.max(0, Math.floor(minIndex));
      const safeMaxIndex = Math.min(
        trajectoryVectors.length - 1,
        Math.floor(maxIndex),
      );

      if (safeMinIndex > safeMaxIndex) {
        return safeMinIndex;
      }

      let nearestIndex = safeMinIndex;
      let nearestDistanceSq = Number.POSITIVE_INFINITY;

      for (let index = safeMinIndex; index <= safeMaxIndex; index += 1) {
        const point = trajectoryVectors[index];
        const distanceSq = currentPosition.distanceToSquared(point);
        if (distanceSq < nearestDistanceSq) {
          nearestDistanceSq = distanceSq;
          nearestIndex = index;
        }
      }

      return nearestIndex;
    }

    function findProgressMatchedTrajectoryIndex(
      currentPosition,
      trajectoryVectors,
      previousTrajectoryLength,
      currentIndex,
      options = {},
    ) {
      if (!currentPosition || !Array.isArray(trajectoryVectors) || trajectoryVectors.length === 0) {
        return 0;
      }

      const {
        minIndex = 0,
        maxIndex = trajectoryVectors.length - 1,
      } = options;

      const safeMinIndex = Math.max(0, Math.floor(minIndex));
      const safeMaxIndex = Math.min(
        trajectoryVectors.length - 1,
        Math.floor(maxIndex),
      );

      if (safeMinIndex > safeMaxIndex) {
        return safeMinIndex;
      }

      const previousLength = Math.max(previousTrajectoryLength ?? trajectoryVectors.length, 2);
      const normalizedProgress = THREE.MathUtils.clamp(
        currentIndex / (previousLength - 1),
        0,
        1,
      );
      const projectedIndex = Math.round(
        normalizedProgress * Math.max(trajectoryVectors.length - 1, 1),
      );

      const searchMin = Math.max(safeMinIndex, projectedIndex - 20);
      const searchMax = Math.min(safeMaxIndex, projectedIndex + 120);

      let bestIndex = Math.max(searchMin, Math.min(projectedIndex, searchMax));
      let bestScore = Number.POSITIVE_INFINITY;

      for (let index = searchMin; index <= searchMax; index += 1) {
        const point = trajectoryVectors[index];
        const distanceSq = currentPosition.distanceToSquared(point);
        const progressOffset = Math.abs(index - projectedIndex);
        const score = distanceSq + progressOffset * progressOffset * 25;

        if (score < bestScore) {
          bestScore = score;
          bestIndex = index;
        }
      }

      return bestIndex;
    }

    function findMissionTurnaroundIndex(trajectoryVectors) {
      if (!Array.isArray(trajectoryVectors) || trajectoryVectors.length === 0) {
        return 0;
      }

      let turnaroundIndex = 0;
      let bestDistanceSq = Number.POSITIVE_INFINITY;

      trajectoryVectors.forEach((point, index) => {
        const distanceSq = point.distanceToSquared(landingPadPos);
        if (distanceSq < bestDistanceSq) {
          bestDistanceSq = distanceSq;
          turnaroundIndex = index;
        }
      });

      return turnaroundIndex;
    }

    function updatePathDrawRanges() {
      const totalRawPoints = trajectoryRef.current.length;
      if (totalRawPoints <= 0) {
        return;
      }

      const currentIndex = Math.floor(sim.current.index);
      const splitIndex = THREE.MathUtils.clamp(
        pathSplitIndexRef.current,
        0,
        totalRawPoints - 1,
      );

      if (forwardLineGeoRef.current) {
        const forwardRawCount = Math.max(splitIndex + 1, 2);
        const forwardProgress = Math.min(currentIndex, splitIndex);
        const drawCount = Math.floor((forwardProgress / Math.max(forwardRawCount - 1, 1)) * 1000);
        forwardLineGeoRef.current.setDrawRange(0, sim.current.personPicked ? 1001 : drawCount);
      }

      if (backwardLineGeoRef.current) {
        const backwardRawCount = Math.max(totalRawPoints - splitIndex, 2);
        const backwardProgress = Math.max(0, currentIndex - splitIndex);
        const drawCount = Math.floor((backwardProgress / Math.max(backwardRawCount - 1, 1)) * 1000);
        backwardLineGeoRef.current.setDrawRange(0, sim.current.personPicked ? drawCount : 0);
      }

      if (lineGeoRef.current && !forwardLineGeoRef.current && !backwardLineGeoRef.current) {
        const ratio = 1000 / totalRawPoints;
        lineGeoRef.current.setDrawRange(0, Math.floor(sim.current.index * ratio));
      }
    }

    function getConfirmedNewDetections(detections) {
      const newlyConfirmed = [];
      const seenThisFrame = new Set();

      detections.forEach((detection) => {
        if (
          !Number.isFinite(detection?.X_world) ||
          !Number.isFinite(detection?.Z_world)
        ) {
          return;
        }

        const key = makeDetectionKey(detection);
        seenThisFrame.add(key);

        if (detectedObstacleKeysRef.current.has(key)) {
          return;
        }

        const nextCount = (detectionCandidateCountsRef.current.get(key) ?? 0) + 1;
        detectionCandidateCountsRef.current.set(key, nextCount);

        if (nextCount >= 1) {  // Trigger replan on first detection — at 220m/s waiting 3 frames = 66m lost
          detectedObstacleKeysRef.current.add(key);
          detectionCandidateCountsRef.current.delete(key);
          newlyConfirmed.push(detection);
        }
      });

      Array.from(detectionCandidateCountsRef.current.keys()).forEach((key) => {
        if (!seenThisFrame.has(key)) {
          const decayed = (detectionCandidateCountsRef.current.get(key) ?? 1) - 1;
          if (decayed <= 0) {
            detectionCandidateCountsRef.current.delete(key);
          } else {
            detectionCandidateCountsRef.current.set(key, decayed);
          }
        }
      });

      return newlyConfirmed;
    }

    // ── OBSTACLE CLASSIFICATION HELPER ────────────────────────────────────────
    function isObstacleKnown(obs, knownList) {
      return knownList.some(k => {
        const dx = (k.x ?? k.X_world ?? 0) - obs.x;
        const dz = (k.z ?? k.Z_world ?? 0) - obs.z;
        return Math.sqrt(dx * dx + dz * dz) < 200;
      });
    }

    // ── BACKEND WEBSOCKET – receive replanned trajectory in real-time ──────────
    useEffect(() => {
      let stompClient = null;
      let active = true;

      async function connectWS() {
        try {
          const [stompMod, sockjsMod] = await Promise.all([
            import('@stomp/stompjs'),
            import('sockjs-client'),
          ]);
          const { Client } = stompMod;
          const SockJS = sockjsMod.default;

          stompClient = new Client({
            webSocketFactory: () => new SockJS('http://localhost:8080/ws'),
            reconnectDelay: 5000,
            onConnect: () => {
              console.log('[WS] Connected to backend WebSocket');
              setWsConnected(true);
              stompClient.subscribe('/topic/trajectory', (msg) => {
                if (!active) return;
                try {
                  const data = JSON.parse(msg.body);
                  const traj = data.trajectory;
                  if (Array.isArray(traj) && traj.length > 1) {
                    console.log(`[WS] Received replanned trajectory: ${traj.length} pts`);
                    // applyLoadedTrajectory is defined inside a different useEffect —
                    // trigger an update via a custom event so it doesn't need a ref.
                    window.dispatchEvent(new CustomEvent('evtol-trajectory-update', { detail: data }));
                    setReplanStatus('UPDATED ✓');
                    setTimeout(() => setReplanStatus('IDLE'), 4000);
                  }
                } catch (e) {
                  console.warn('[WS] Failed to parse trajectory message', e);
                }
              });
            },
            onDisconnect: () => { console.log('[WS] Disconnected'); setWsConnected(false); },
            onStompError: (f) => console.warn('[WS] STOMP error', f.headers?.message),
          });

          stompClient.activate();
          wsStompRef.current = stompClient;
        } catch (e) {
          console.warn('[WS] WebSocket setup failed (backend may not be running):', e.message);
        }
      }

      connectWS();
      return () => {
        active = false;
        stompClient?.deactivate();
      };
    // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    useImperativeHandle(ref, () => ({
      play: () => {
        if (!trajectoryReadyRef.current || trajectoryRef.current.length < 2) {
          console.warn("Trajectory not ready yet, cannot start playback.");
          return false;
        }
        sim.current.isRunning = true;
        autoCaptureRef.current = true;   // ← start 10fps capture
        lastCaptureRef.current = 0;
        return true;
      },
      pause: () => {
        sim.current.isRunning = false;
        autoCaptureRef.current = false;   // ← stop capture
      },
      reset: () => {
        sim.current.isRunning = false;
        autoCaptureRef.current = false;   // ← stop capture

        sim.current.isLanded = false;
        sim.current.t = 0;
        sim.current.index = 0;
        sim.current.waitTimer = 0;
        sim.current.personPicked = false;
        sim.current.pathCompleted = false;
        sim.current.postPickupAltTimer = 0;

        // reset capture counters and YOLO state
        frameIndexRef.current = 0;
        setScreenshots([]);
        setLatestObstacles([]);
        setObstacleHistory([]);
        detectedObstacleKeysRef.current = new Set();
        detectionCandidateCountsRef.current = new Map();
        replanInFlightRef.current = false;
        lastProactiveCheckRef.current = 0;
        lastDetourPostIdxRef.current = -1;
        forwardTrajectoryRef.current = null;
        knownObstaclesRef.current = [...seedObstaclesRef.current]; // restore to seed only
        birdIndexRef.current = 0;
        birdDirectionRef.current = 1;
        setReplanStatus("IDLE");
        setValidationResult(null);
        // Reset verification state — detection rings removed from scene
        setDetectedSurpriseIds(new Set());
        Object.values(detectionRingMapRef.current).forEach(ring => {
          sceneRef.current?.remove(ring);
        });
        detectionRingMapRef.current = {};
        fetch("http://localhost:5050/reset_smooth", { method: "POST" }).catch(() => { });
        // Clear backend detected_obstacles.csv then reload a fresh trajectory so
        // the initial path does NOT already avoid surprise buildings.
        fetch("http://localhost:8080/api/obstacles/detections", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ source: "reset", frameIndex: 0, obstacles: [] }),
        })
          .then(() => loadTrajectoryRef.current({ preserveFlightProgress: false }))
          .catch(() => { });

        if (personRef.current) personRef.current.visible = true;
        if (birdRef.current && birdPathRef.current.length > 0) {
          birdRef.current.position.copy(birdPathRef.current[0]);
          birdRef.current.rotation.set(0, 0, 0);
        }
        const resetPos = getInitialFlightPosition();
        if (evtolRef.current) {
          evtolRef.current.position.copy(resetPos);
          evtolRef.current.rotation.set(0, 0, 0);
        }
        if (lineGeoRef.current) lineGeoRef.current.setDrawRange(0, 0);
        if (forwardLineGeoRef.current) forwardLineGeoRef.current.setDrawRange(0, 0);
        if (backwardLineGeoRef.current) backwardLineGeoRef.current.setDrawRange(0, 0);
        onPositionUpdate?.({
          x: resetPos.x, y: resetPos.y, z: resetPos.z,
          speed: 0, altitude: resetPos.y, heading: 0,
        });
        missionStateRef.current.reset();
      },
      getMissionStatus: () => missionStateRef.current.getStatus(),

      setMissionStatus: (status) =>
        missionStateRef.current.setStatus(status),

      nextMissionPhase: () => missionStateRef.current.next(),

      resetMissionStatus: () => missionStateRef.current.reset(),
      setSpeed: (v) => (sim.current.playbackSpeed = v),
      captureFPV: () => {
        const renderer = rendererRef.current;
        if (!renderer) return;
        const dataUrl = renderer.domElement.toDataURL("image/png");
        setScreenshots((prev) => [
          ...prev,
          { dataUrl, frameIndex: frameIndexRef.current, obstacles: [], annotatedImg: null },
        ]);
      },
      exportHeightRadiusCSV: () => {
        setHeightRadiusLog((log) => {
          if (log.length === 0) { alert("No height-radius data yet. Play the simulation first."); return log; }
          const header = "Index,Phase,Height_m,Apparent_Radius_px,R_H_ratio\n";
          const rows = log.map((r, i) =>
            `${i + 1},${r.phase},${r.height},${r.apparentRadius},${r.ratio}`
          ).join("\n");
          const blob = new Blob([header + rows], { type: "text/csv" });
          const url = URL.createObjectURL(blob);
          const a = document.createElement("a");
          a.href = url;
          a.download = "vertiport_height_radius.csv";
          a.click();
          URL.revokeObjectURL(url);
          return log;
        });
      },
    }));
    useEffect(() => {
      const missionState = missionStateRef.current;

      if (onMissionUpdate) {
        missionState.setUpdateCallback(onMissionUpdate);
      }

    }, [onMissionUpdate]);

    useEffect(() => {
      // Reset preview when switching images
      setVisionMode("original");
      if (visionPreviewUrl?.startsWith("blob:")) {
        URL.revokeObjectURL(visionPreviewUrl);
      }
      setVisionPreviewUrl(null);
      // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [selectedImage]);

    async function runVision(kind, preset = "auto") {
      const frame = selectedImage?.dataUrl;
      if (!frame) return;

      // Clean up previous blob URL if present
      if (visionPreviewUrl?.startsWith("blob:")) {
        URL.revokeObjectURL(visionPreviewUrl);
      }
      setVisionPreviewUrl(null);

      try {
        if (kind === "remove-bg") {
          const blob = await (await fetch(frame)).blob();
          const fd = new FormData();
          fd.append("image", blob, "frame.png");
          const res = await fetch("http://localhost:8080/api/vision/remove-bg", {
            method: "POST",
            body: fd,
          });
          const outBlob = await res.blob();
          const url = URL.createObjectURL(outBlob);
          setVisionPreviewUrl(url);
          setVisionMode("remove-bg");
          return;
        }

        const path =
          kind === "edge" ? "edge"
            : kind === "threshold" ? "threshold"
              : "bg-reveal";

        const res = await fetch(`http://localhost:8080/api/vision/${path}`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ frame, preset }),
        });
        const data = await res.json();
        if (data?.image) {
          setVisionPreviewUrl(data.image);
          setVisionMode(kind);
        }
      } catch (e) {
        console.warn("Vision request failed:", e);
      }
    }

    useEffect(() => {
      function sanitizeTrajectoryPoints(traj) {
        if (!Array.isArray(traj)) return [];

        return traj.filter((point) =>
          point &&
          Number.isFinite(point.x) &&
          Number.isFinite(point.y) &&
          Number.isFinite(point.z)
        );
      }

      function buildPathGeometry(points) {
        if (!Array.isArray(points) || points.length < 2) {
          return null;
        }

        const linePoints = points.map((p) => new THREE.Vector3(p.x, p.y + 2, p.z));
        const curve = new THREE.CatmullRomCurve3(linePoints);
        curve.curveType = "catmullrom";
        curve.tension = 0.5;
        return new THREE.BufferGeometry().setFromPoints(curve.getPoints(1000));
      }

      function replaceTrajectoryLines(missionTrajectory) {
        if (!sceneRef.current || missionTrajectory.length < 2) {
          return;
        }

        if (lineRef.current) sceneRef.current.remove(lineRef.current);
        if (forwardLineRef.current) sceneRef.current.remove(forwardLineRef.current);
        if (backwardLineRef.current) sceneRef.current.remove(backwardLineRef.current);

        const vectors = missionTrajectory.map((p) => new THREE.Vector3(p.x, p.y, p.z));
        const splitIndex = findMissionTurnaroundIndex(vectors);
        pathSplitIndexRef.current = splitIndex;

        const forwardPoints = missionTrajectory.slice(0, splitIndex + 1);
        const backwardPoints = missionTrajectory.slice(splitIndex);

        forwardLineGeoRef.current = buildPathGeometry(forwardPoints);
        backwardLineGeoRef.current = buildPathGeometry(backwardPoints);

        forwardLineRef.current = forwardLineGeoRef.current
          ? new THREE.Line(
            forwardLineGeoRef.current,
            new THREE.LineBasicMaterial({ color: 0x00ff88 }),
          )
          : null;
        backwardLineRef.current = backwardLineGeoRef.current
          ? new THREE.Line(
            backwardLineGeoRef.current,
            new THREE.LineBasicMaterial({ color: 0x3b82ff }),
          )
          : null;

        if (forwardLineRef.current) sceneRef.current.add(forwardLineRef.current);
        if (backwardLineRef.current) sceneRef.current.add(backwardLineRef.current);

        lineRef.current = null;
        lineGeoRef.current = sim.current.personPicked
          ? backwardLineGeoRef.current
          : forwardLineGeoRef.current;
      }

      function applyLoadedTrajectory(traj, options = {}) {
        const { preserveFlightProgress = false } = options;
        const cleanTrajectory = sanitizeTrajectoryPoints(traj);

        if (cleanTrajectory.length < 2) {
          console.error("Invalid trajectory:", traj);
          return false;
        }

        const outboundTrajectory = transformTrajectoryToMissionSpace(cleanTrajectory);
        if (!preserveFlightProgress || !forwardTrajectoryRef.current) {
          forwardTrajectoryRef.current = outboundTrajectory;
        }

        const missionTrajectory =
          preserveFlightProgress && sim.current.personPicked && forwardTrajectoryRef.current
            ? [
              ...forwardTrajectoryRef.current,
              ...createSeparatedReturnLeg(outboundTrajectory),
            ]
            : ensureRoundTripTrajectory(outboundTrajectory);

        if (
          missionTrajectory.length < 2 ||
          missionTrajectory.some(
            (point) =>
              !Number.isFinite(point.x) ||
              !Number.isFinite(point.y) ||
              !Number.isFinite(point.z),
          )
        ) {
          console.error("Mission trajectory contains invalid points:", missionTrajectory);
          return false;
        }

        // Keep a copy of the first trajectory as "original" for diff display
        if (!originalTrajectoryRef.current) {
          originalTrajectoryRef.current = missionTrajectory;
        }

        const previousTrajectory = trajectoryRef.current;
        const previousTrajectoryLength = trajectoryVectorsRef.current.length;
        const currentIndex = Math.floor(sim.current.index);
        const missionVectors = missionTrajectory.map(
          (p) => new THREE.Vector3(p.x, p.y, p.z)
        );

        // Pre-compute the resume index before building geometry so the draw range
        // is set correctly on the very first frame — prevents the green trail from
        // flashing to zero when the trajectory is hot-swapped mid-flight.
        let resumeIndex = 0;
        if (preserveFlightProgress && evtolRef.current) {
          const turnaroundIndex = findMissionTurnaroundIndex(
            missionVectors,
          );
          const resumeWindow = sim.current.personPicked
            ? {
              minIndex: Math.max(turnaroundIndex, currentIndex),
              maxIndex: missionVectors.length - 1,
            }
            : {
              minIndex: Math.max(0, currentIndex),
              maxIndex: Math.max(turnaroundIndex, currentIndex + 160),
            };

          resumeIndex = findProgressMatchedTrajectoryIndex(
            evtolRef.current.position.clone(),
            missionVectors,
            previousTrajectoryLength,
            currentIndex,
            resumeWindow,
          );
        }

        const activeTrajectory = missionTrajectory;

        trajectoryRef.current = activeTrajectory;
        trajectoryVectorsRef.current = activeTrajectory.map(
          (p) => new THREE.Vector3(p.x, p.y, p.z)
        );
        initialFlightPosRef.current = trajectoryVectorsRef.current[0]?.clone() ?? startPos.clone();
        replaceTrajectoryLines(activeTrajectory);

        if (preserveFlightProgress) {
          sim.current.index = resumeIndex;
          // No position copy — the animation loop sets evtol.position every frame
          // via Catmull-Rom on trajectoryRef, so it self-corrects on the next frame
          // without causing a visible position snap.
        } else {
          if (evtolRef.current) evtolRef.current.position.copy(initialFlightPosRef.current);
          sim.current.index = 0;
        }

        updatePathDrawRanges();
        trajectoryReadyRef.current = true;
        console.log("Trajectory loaded:", trajectoryVectorsRef.current.length);
        return true;
      }

      async function loadObstacles() {
        try {
          const response = await fetch('/data/obstacles.csv');
          const csvText = await response.text();
          const lines = csvText.trim().split('\n');
          const obstacles = [];

          for (let i = 1; i < lines.length; i++) { // Skip header
            const parts = lines[i].split(',');
            if (parts.length < 4) continue;
            const [x, y, z, radius] = parts.map(Number);
            if (!isFinite(x) || !isFinite(z)) continue;
            obstacles.push({
              id: `OBS-${String(i).padStart(3, "0")}`,
              x, y, z, radius,
            });
          }

          obstacleBuildingsRef.current = obstacles;
          console.log("[Visual] Buildings loaded:", obstacles.length);
        } catch (err) {
          console.error("Failed to load obstacles:", err);
          obstacleBuildingsRef.current = [];
        }
      }

      // Loads backend-known (imageprocessing CSV) + detected obstacles.
      // Classifies each visual building as 'known' or 'surprise' and stores
      // that in obstacleClassMapRef so the building renderer can colour them.
      async function loadKnownObstacles() {
        try {
          const resp = await fetch('http://localhost:8080/api/obstacles');
          const data = await resp.json();

          // seedObstacles = what backend permanently knows (matches visual building coordinates)
          const imgProc = (data.seedObstacles || []);
          const known = imgProc.map((o, i) => ({
            id: `KNOWN-${i}`, x: o.x, y: o.y, z: o.z, radius: o.radius || 155,
          }));
          seedObstaclesRef.current  = known;
          knownObstaclesRef.current = [...known];
          setBackendKnownCount(known.length);
          console.log("[KnownObs] Backend-known loaded:", known.length, "→ YOLO will discover the rest");

          // Classify every visual building
          const allVisual = obstacleBuildingsRef.current;
          const surprises = [];
          const classMap  = {};
          allVisual.forEach(obs => {
            const kn = isObstacleKnown(obs, known);
            classMap[obs.id] = kn ? 'known' : 'surprise';
            if (!kn) surprises.push(obs);

            // Apply colour if the building mesh is already in the scene
            const mesh = buildingMeshMapRef.current[obs.id];
            if (mesh) applyBuildingClass(mesh, classMap[obs.id]);
          });
          obstacleClassMapRef.current = classMap;
          setSurpriseObstacles(surprises);
          console.log(`[KnownObs] Classified: ${known.length} known, ${surprises.length} surprise`);

          // Also restore any YOLO-detected obstacles from a previous session
          const detected = (data.detectedObstacles || []).map((o, i) => ({
            id: `DETECTED-${i}`, x: o.x, y: o.y, z: o.z, radius: o.radius || 155,
          }));
          if (detected.length > 0) {
            knownObstaclesRef.current = [...knownObstaclesRef.current, ...detected];
            console.log("[KnownObs] Restored detected obstacles:", detected.length);
          }
        } catch (e) {
          console.warn("[KnownObs] Backend unreachable — loading local known_obstacles.csv fallback");
          try {
            const fallbackResp = await fetch('/data/known_obstacles.csv');
            const fallbackCsv  = await fallbackResp.text();
            const fallbackLines = fallbackCsv.trim().split('\n').slice(1);
            const fallbackKnown = fallbackLines.map((line, i) => {
              const [x, y, z, radius] = line.split(',').map(Number);
              return { id: `KNOWN-${i}`, x, y, z, radius: radius || 155 };
            }).filter(o => Number.isFinite(o.x) && Number.isFinite(o.z));

            seedObstaclesRef.current  = fallbackKnown;
            knownObstaclesRef.current = [...fallbackKnown];
            setBackendKnownCount(fallbackKnown.length);

            // Classify visual buildings using the local known list
            const allVisual = obstacleBuildingsRef.current;
            const classMap  = {};
            const surprises = [];
            allVisual.forEach(obs => {
              const kn = isObstacleKnown(obs, fallbackKnown);
              classMap[obs.id] = kn ? 'known' : 'surprise';
              if (!kn) surprises.push(obs);
              const mesh = buildingMeshMapRef.current[obs.id];
              if (mesh) applyBuildingClass(mesh, classMap[obs.id]);
            });
            obstacleClassMapRef.current = classMap;
            setSurpriseObstacles(surprises);
            console.log(`[KnownObs] Offline fallback: ${fallbackKnown.length} known, ${surprises.length} surprise`);
          } catch (fe) {
            console.warn("[KnownObs] Local fallback also failed — classification unavailable");
            seedObstaclesRef.current  = [];
            knownObstaclesRef.current = [];
          }
        }
      }

      // Apply visual colour class to a building Object3D
      function applyBuildingClass(buildingObj, cls) {
        if (cls === 'surprise') {
          buildingObj.traverse(child => {
            if (child.isMesh) {
              if (!child.__origMat) child.__origMat = child.material;
              child.material = child.material.clone();
              child.material.color.setHex(0xff6600);
              child.material.emissive = new THREE.Color(0x1a0800);
              child.material.emissiveIntensity = 0.25;
            }
          });
        }
      }

      async function loadTrajectory(options = {}) {
        const {
          requestObstacles = null,
          preserveFlightProgress = false,
        } = options;

        // Always include all visual buildings so the backend planner avoids every
        // building from the very first trajectory load.  Merge any extra per-call
        // obstacles (e.g. bird positions) without duplicating visual buildings.
        const visualObs = obstacleBuildingsRef.current.map(b => ({
          x: b.x, y: b.y || 0, z: b.z, radius: b.radius || 155,
        }));
        const extraObs = Array.isArray(requestObstacles) ? requestObstacles : [];
        const allPlannerObs = [...visualObs];
        for (const o of extraObs) {
          const dup = allPlannerObs.some(b => {
            const dx = b.x - o.x, dz = b.z - o.z;
            return Math.sqrt(dx * dx + dz * dz) < 120;
          });
          if (!dup) allPlannerObs.push(o);
        }

        try {
          trajectoryReadyRef.current = false;
          const response = await fetch("http://localhost:8080/api/trajectory", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ lambda: 0.1, obstacles: allPlannerObs }),
          });
          if (!response.ok) {
            throw new Error(`Trajectory request failed with status ${response.status}`);
          }

          const data = await response.json();
          const trajectory = sanitizeTrajectoryPoints(data.trajectory);
          return applyLoadedTrajectory(trajectory, { preserveFlightProgress });

        } catch (err) {
          console.error("Fetch failed:", err);
          return false;
        }
      }

      loadTrajectoryRef.current = loadTrajectory;

      async function persistDetectedObstacles(obstacles, source, frameIndex) {
        const toCoord = (o) => ({
          x: o.X_world ?? o.x,
          y: o.Y_world ?? o.y ?? 0,
          z: o.Z_world ?? o.z,
          radius: o.radius || 155,
        });
        const isValid = (o) => Number.isFinite(o.x) && Number.isFinite(o.y) && Number.isFinite(o.z);
        const payload = {
          source,
          frameIndex,
          obstacles:     (obstacles || []).filter(o => o.label !== "bird").map(toCoord).filter(isValid),
          birdObstacles: (obstacles || []).filter(o => o.label === "bird").map(toCoord).filter(isValid),
        };

        try {
          await fetch("http://localhost:8080/api/obstacles/detections", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(payload),
          });
        } catch (error) {
          console.warn("[DetectedObs] Failed to save detected obstacles:", error);
        }
      }

      // Mark a building's detection ring visible (or create it) and update state
      function markSurpriseDetected(obsId, scene3d) {
        setDetectedSurpriseIds(prev => {
          if (prev.has(obsId)) return prev;
          const next = new Set(prev);
          next.add(obsId);
          return next;
        });

        // Add a red glowing ring around the detected surprise building
        const mesh = buildingMeshMapRef.current[obsId];
        if (mesh && scene3d && !detectionRingMapRef.current[obsId]) {
          const ringGeo = new THREE.RingGeometry(175, 195, 32);
          const ringMat = new THREE.MeshBasicMaterial({
            color: 0xff2200, transparent: true, opacity: 0.85, side: THREE.DoubleSide,
          });
          const ring = new THREE.Mesh(ringGeo, ringMat);
          ring.rotation.x = -Math.PI / 2;
          ring.position.set(mesh.position.x, 12, mesh.position.z);
          scene3d.add(ring);
          detectionRingMapRef.current[obsId] = ring;
        }
      }

      async function ingestPlannerObstacles(plannerObs, currentIndex, sourcePrefix) {
        const validObs = (plannerObs || []).filter(
          (o) => Number.isFinite(o.X_world) && Number.isFinite(o.Z_world));

        if (validObs.length === 0) {
          return;
        }

        const updated = [...knownObstaclesRef.current];
        let changed = false;
        for (const o of validObs) {
          const key = makeDetectionKey({
            X_world: o.X_world,
            Z_world: o.Z_world,
            radius: o.radius || 155,
          });
          const exists = updated.some((k) =>
            makeDetectionKey({ X_world: k.x, Z_world: k.z, radius: k.radius }) === key);
          if (!exists) {
            updated.push({
              id: `${sourcePrefix}-${Math.round(o.X_world)}-${Math.round(o.Z_world)}`,
              label: o.label || "structure",
              x: o.X_world,
              y: o.Y_world || 0,
              z: o.Z_world,
              radius: o.radius || 155,
            });
            changed = true;

            // Check if this newly-detected obstacle matches a surprise building
            obstacleBuildingsRef.current.forEach(b => {
              if (obstacleClassMapRef.current[b.id] === 'surprise') {
                const dx = b.x - o.X_world;
                const dz = b.z - o.Z_world;
                if (Math.sqrt(dx * dx + dz * dz) < 400) {
                  markSurpriseDetected(b.id, sceneRef.current);
                  console.log(`[Verify] Surprise obstacle ${b.id} DETECTED by ${sourcePrefix}`);
                }
              }
            });
          }
        }

        if (changed) {
          knownObstaclesRef.current = updated;
        }

        await persistDetectedObstacles(
          updated.filter((k) => k.id && !k.id.startsWith("SEED-")),
          sourcePrefix.toLowerCase(),
          currentIndex,
        );

        await replanTrajectoryFromDetectionsRef.current(validObs);
      }

      markSurpriseDetectedRef.current = markSurpriseDetected;
      ingestPlannerObstaclesRef.current = ingestPlannerObstacles;

      async function replanTrajectoryFromDetections(detections) {
        if (!Array.isArray(detections) || detections.length === 0) return;

        // Suppress if still inside the previous detour zone
        if (lastDetourPostIdxRef.current >= 0 &&
            sim.current.index < lastDetourPostIdxRef.current) return;

        const confirmedDetections = getConfirmedNewDetections(detections);
        if (confirmedDetections.length === 0) { setReplanStatus("TRACKING"); return; }

        // YOLO found new obstacles — feed them through the same single-pass detour
        applyInPlaceDetour();
      }

      async function replanTrajectoryViaBackend(detections) {
        if (!Array.isArray(detections) || detections.length === 0) return;

        if (lastDetourPostIdxRef.current >= 0 &&
            sim.current.index < lastDetourPostIdxRef.current) return;

        const confirmedDetections = getConfirmedNewDetections(detections);
        if (confirmedDetections.length === 0) {
          setReplanStatus("TRACKING");
          return;
        }

        setReplanStatus("REPLANNING");

        const plannerObstacles = knownObstaclesRef.current.map((o) => ({
          x: o.x,
          y: o.y || 0,
          z: o.z,
          radius: o.radius || 155,
        }));

        const replanned = await loadTrajectory({
          requestObstacles: plannerObstacles,
          preserveFlightProgress: true,
        });

        if (replanned) {
          lastDetourPostIdxRef.current = Math.max(
            lastDetourPostIdxRef.current,
            Math.floor(sim.current.index) + 60,
          );
          setReplanStatus("UPDATED");
          return;
        }

        applyInPlaceDetour();
      }

      replanTrajectoryFromDetectionsRef.current = replanTrajectoryViaBackend;

      /**
       * Single-pass obstacle avoidance: scans ALL known obstacles in one go,
       * computes ONE combined arc around the entire cluster, and suppresses
       * further triggers until the eVTOL has physically cleared the detour zone.
       *
       * Why single-pass: applying separate detours per obstacle causes each one
       * to push the path near a different building, cascading into the wild
       * zigzag seen in the screenshot.  One arc → clean S-curve → done.
       */
      function applyInPlaceDetour() {
        const traj = trajectoryRef.current;
        if (!traj || traj.length < 4) return;

        const currentIdx = Math.floor(sim.current.index);

        // ── 1. Scan surprise obstacles only (backend trajectory already avoids KNOWN-*) ──
        // Birds use a tight clearance so they don't inflate building cluster geometry.
        const allObs = knownObstaclesRef.current.filter(o => !o.id?.startsWith('KNOWN-'));
        if (!allObs || allObs.length === 0) return;

        const isBird = o => o.label === 'bird';
        // Buildings are rendered at scale 250 — visual extent ~400 m from centre,
        // so the scan buffer must exceed that. Birds are small; keep buffer tight.
        const obsClearance = o => isBird(o) ? (o.radius || 55) + 40 : (o.radius || 155) + 300;
        const bypassFactor = o => isBird(o) ? 1.5 : 2.5;

        let globalEntry = traj.length;
        let globalExit  = -1;
        // Separate centroids: buildings drive the bypass point; birds are handled with a smaller offset
        let bldSumX = 0, bldSumZ = 0, bldCount = 0;
        let birdSumX = 0, birdSumZ = 0, birdClusterCount = 0;
        let maxClearance = 0;
        let maxBypassFactor = 2.5;
        const clusterSet = new Set(); // obstacle keys that are IN the cluster

        for (const obs of allObs) {
          const clearance = obsClearance(obs);
          let obsFirst = -1, obsLast = -1;

          for (let i = currentIdx + 1; i < traj.length; i++) {
            const dx = traj[i].x - obs.x;
            const dz = traj[i].z - obs.z;
            // Birds: 3D sphere check. Buildings: x-z cylinder.
            const dist = isBird(obs)
              ? Math.sqrt(dx * dx + (traj[i].y - (obs.y || 0)) ** 2 + dz * dz)
              : Math.sqrt(dx * dx + dz * dz);
            if (dist < clearance) {
              if (obsFirst === -1) obsFirst = i;
              obsLast = i;
            }
          }

          if (obsFirst !== -1) {
            globalEntry = Math.min(globalEntry, obsFirst);
            globalExit  = Math.max(globalExit,  obsLast);
            if (isBird(obs)) {
              birdSumX += obs.x; birdSumZ += obs.z; birdClusterCount++;
            } else {
              bldSumX += obs.x; bldSumZ += obs.z; bldCount++;
            }
            if (clearance > maxClearance) {
              maxClearance    = clearance;
              maxBypassFactor = bypassFactor(obs);
            }
            const key = makeDetectionKey({ X_world: obs.x, Z_world: obs.z, radius: obs.radius });
            clusterSet.add(key);
            detectedObstacleKeysRef.current.add(key);
          }
        }

        if (globalExit === -1 || (bldCount + birdClusterCount) === 0) return;

        // If the threat zone starts too close to the current position there is no
        // room to carve a smooth arc — skip and let the next tick handle it.
        if (globalEntry < currentIdx + 10) return;

        // Bypass centroid: prefer building center; fall back to bird center if no buildings
        const clusterCount = bldCount > 0 ? bldCount : birdClusterCount;
        const clusterSumX  = bldCount > 0 ? bldSumX  : birdSumX;
        const clusterSumZ  = bldCount > 0 ? bldSumZ  : birdSumZ;

        // ── 2. Expand window for smooth entry / exit blending ─────────────────
        // Cap pad so the arc window never spans more than ~300 trajectory points
        // beyond the danger zone — prevents half-trajectory loops.
        const zoneLen = Math.max(20, globalExit - globalEntry);
        const pad     = Math.max(40, Math.min(150, Math.floor(zoneLen * 0.5)));
        // preIdx must be at least 30 steps ahead so the eVTOL has room to smoothly
        // enter the bypass arc without creating a tight visual loop.
        const preIdx  = Math.max(currentIdx + 30, globalEntry - pad);
        const postIdx = Math.min(traj.length - 1, globalExit  + pad);

        const prePt  = traj[preIdx];
        const postPt = traj[postIdx];

        // ── 3. Travel direction and perpendicular ──────────────────────────────
        const travDx  = postPt.x - prePt.x;
        const travDz  = postPt.z - prePt.z;
        const travLen = Math.sqrt(travDx * travDx + travDz * travDz);
        if (travLen < 1) return;

        const perpX = -travDz / travLen;
        const perpZ =  travDx / travLen;

        // ── 4. Side: pick the bypass direction that clears non-cluster obstacles ─
        const clusterX = clusterSumX / clusterCount;
        const clusterZ = clusterSumZ / clusterCount;
        const midX     = (prePt.x + postPt.x) / 2;
        const midZ     = (prePt.z + postPt.z) / 2;
        const dot      = (clusterX - midX) * perpX + (clusterZ - midZ) * perpZ;
        const preferredSign = dot >= 0 ? -1 : 1;
        let   sign          = preferredSign;

        // ── 5. Bypass point: cluster centre + 2.5× largest clearance ──────────
        //   Sample arc midpoints (t=0.3, 0.5, 0.7) against non-cluster obstacles.
        //   Cluster members are excluded — the 2.5× offset guarantees they're clear.
        //   If preferred direction clips a non-cluster building, try the other.
        //   If both fail, revert to preferred (still far better than cutting through).
        function isBypassClear(s) {
          const bx = clusterX + s * perpX * maxClearance * maxBypassFactor;
          const bz = clusterZ + s * perpZ * maxClearance * maxBypassFactor;
          for (const t5 of [0.3, 0.5, 0.7]) {
            const b0 = (1-t5)*(1-t5), b1 = 2*(1-t5)*t5, b2 = t5*t5;
            const ax = b0*prePt.x + b1*bx + b2*postPt.x;
            const az = b0*prePt.z + b1*bz + b2*postPt.z;
            for (const obs of allObs) {
              if (clusterSet.has(
                  makeDetectionKey({ X_world: obs.x, Z_world: obs.z, radius: obs.radius })))
                continue; // cluster member — guaranteed clear by 2.5× factor
              const c  = (obs.radius || 155) + 120;
              const dx = ax - obs.x, dz = az - obs.z;
              if (Math.sqrt(dx*dx + dz*dz) < c) return false;
            }
          }
          return true;
        }

        if (!isBypassClear(sign)) {
          sign = -sign;
          if (!isBypassClear(sign)) sign = preferredSign; // both fail — revert to preferred
        }

        // ── 5b. Find a bypass factor that actually clears ALL obstacles ──────────
        // Sample 21 points along the Bézier arc and check every building in allObs.
        // Retry with a larger factor (up to 4×) and try both sides if needed.
        function isArcClear(bx, bz) {
          for (let step = 0; step <= 20; step++) {
            const t  = step / 20;
            const b0 = (1 - t) * (1 - t), b1 = 2 * (1 - t) * t, b2 = t * t;
            const ax = b0 * prePt.x + b1 * bx + b2 * postPt.x;
            const az = b0 * prePt.z + b1 * bz + b2 * postPt.z;
            for (const obs of allObs) {
              if (isBird(obs)) continue; // birds are small — don't force wide arcs for them
              const c  = obsClearance(obs);
              const dx = ax - obs.x, dz = az - obs.z;
              if (Math.sqrt(dx * dx + dz * dz) < c) return false;
            }
          }
          return true;
        }

        let finalFactor = maxBypassFactor;
        let finalSign   = sign;
        let bypassX = clusterX + finalSign * perpX * maxClearance * finalFactor;
        let bypassZ = clusterZ + finalSign * perpZ * maxClearance * finalFactor;

        if (!isArcClear(bypassX, bypassZ)) {
          for (let attempt = 1; attempt <= 4; attempt++) {
            finalFactor = maxBypassFactor + attempt * 0.5;
            bypassX = clusterX + finalSign * perpX * maxClearance * finalFactor;
            bypassZ = clusterZ + finalSign * perpZ * maxClearance * finalFactor;
            if (isArcClear(bypassX, bypassZ)) break;
            // Try opposite side
            const altX = clusterX - finalSign * perpX * maxClearance * finalFactor;
            const altZ = clusterZ - finalSign * perpZ * maxClearance * finalFactor;
            if (isArcClear(altX, altZ)) { bypassX = altX; bypassZ = altZ; finalSign = -finalSign; break; }
          }
        }

        const bypassY = (prePt.y + postPt.y) / 2;

        // ── 6. Splice ONE quadratic bezier into [preIdx … postIdx] ────────────
        const newTraj = traj.slice();
        const span    = postIdx - preIdx;
        for (let i = preIdx; i <= postIdx; i++) {
          const t  = span > 0 ? (i - preIdx) / span : 0;
          const b0 = (1 - t) * (1 - t);
          const b1 = 2 * (1 - t) * t;
          const b2 = t * t;
          newTraj[i] = {
            ...traj[i],
            x: b0 * prePt.x + b1 * bypassX + b2 * postPt.x,
            y: b0 * prePt.y + b1 * bypassY + b2 * postPt.y,
            z: b0 * prePt.z + b1 * bypassZ + b2 * postPt.z,
          };
        }

        trajectoryRef.current        = newTraj;
        trajectoryVectorsRef.current = newTraj.map(p => new THREE.Vector3(p.x, p.y, p.z));

        // ── 7. Suppress re-triggering until eVTOL clears the detour zone ──────
        lastDetourPostIdxRef.current = postIdx;

        // ── 8. Rebuild green line — draw range matches current progress ────────
        replaceTrajectoryLines(newTraj);
        updatePathDrawRanges();

        setReplanStatus("UPDATED✓");
      }

      proactiveReplanRef.current = applyInPlaceDetour;

      // Listen for backend-pushed trajectory updates (via WebSocket → custom event)
      function onWsTrajectoryUpdate(evt) {
        const data = evt.detail;
        const traj = data?.trajectory;
        if (Array.isArray(traj) && traj.length > 1) {
          console.log('[WS] Applying server-pushed trajectory update');
          applyLoadedTrajectory(traj, { preserveFlightProgress: true });
        }
      }
      window.addEventListener('evtol-trajectory-update', onWsTrajectoryUpdate);

      // Clear stale detected obstacles THEN load — ensures the initial trajectory
      // does not already avoid surprise buildings (surprise must be discovered in-flight).
      const clearDetected = () => fetch("http://localhost:8080/api/obstacles/detections", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ source: "init", frameIndex: 0, obstacles: [] }),
      }).catch(() => { });

      // Sequential: clearDetected → loadObstacles → loadKnownObstacles →
      // loadTrajectory (all buildings sent to backend) → frontend safety-net detour
      // for any surprise building the backend still left in the path.
      clearDetected().finally(() =>
        loadObstacles()
          .then(() => loadKnownObstacles())
          .then(() => loadTrajectory())
          .then(() => {
            const traj = trajectoryRef.current;
            if (!traj || traj.length === 0) return;
            const clearance = 455; // radius(155) + buffer(300)
            let added = false;
            obstacleBuildingsRef.current.forEach(b => {
              if (obstacleClassMapRef.current[b.id] === 'known') return;
              const alreadyHandled = knownObstaclesRef.current.some(k => {
                const dx = k.x - b.x, dz = k.z - b.z;
                return Math.sqrt(dx * dx + dz * dz) < 240;
              });
              if (alreadyHandled) return;
              const threatens = traj.some(pt => {
                const dx = pt.x - b.x, dz = pt.z - b.z;
                return Math.sqrt(dx * dx + dz * dz) < clearance;
              });
              if (!threatens) return;
              knownObstaclesRef.current = [...knownObstaclesRef.current, {
                id: `SURP-${Math.round(b.x)}-${Math.round(b.z)}`,
                label: 'building', x: b.x, y: b.y || 0, z: b.z,
                radius: b.radius || 155,
              }];
              added = true;
            });
            if (added) applyInPlaceDetour();
          })
      );

      return () => {
        window.removeEventListener('evtol-trajectory-update', onWsTrajectoryUpdate);
      };
    }, []);
    function horizontalToVerticalFOV(hFOV, aspect) {
      const hFOVRad = THREE.MathUtils.degToRad(hFOV);
      const vFOVRad = 2 * Math.atan(Math.tan(hFOVRad / 2) / aspect);
      return THREE.MathUtils.radToDeg(vFOVRad);
    }

    useEffect(() => {
      const container = containerRef.current;
      const scene = new THREE.Scene();
      scene.background = new THREE.Color(0x87ceeb);

      sceneRef.current = scene;
      //  Reattach trajectory line if already created
      if (lineRef.current) {
        scene.add(lineRef.current);
      }
      if (forwardLineRef.current) {
        scene.add(forwardLineRef.current);
      }
      if (backwardLineRef.current) {
        scene.add(backwardLineRef.current);
      }

      const cameraRig = new THREE.Object3D();
      scene.add(cameraRig);

      const renderer = new THREE.WebGLRenderer({
        antialias: true,
        preserveDrawingBuffer: true,
      });
      renderer.setSize(container.clientWidth, container.clientHeight);
      container.appendChild(renderer.domElement);
      rendererRef.current = renderer;

      const aspect = container.clientWidth / container.clientHeight;

      const getVerticalFOV = (hFov, aspect) => {
        return (
          2 *
          Math.atan(Math.tan((hFov * Math.PI) / 360) / aspect) *
          (180 / Math.PI)
        );
      };

      // default FOV (will change later dynamically)
      const perspectiveCamera = new THREE.PerspectiveCamera(
        60,
        aspect,
        0.1,
        100000,
      );
      perspectiveCamera.updateProjectionMatrix();
      perspectiveCamera.position.set(2200, 1200, 2500);
      perspectiveCameraRef.current = perspectiveCamera;

      cameraRig.add(perspectiveCamera);

      const controls = new OrbitControls(
        perspectiveCamera,
        renderer.domElement,
      );

      // ===== CUSTOM FOV ZOOM =====
      renderer.domElement.addEventListener("wheel", (e) => {
        if (cameraModeRef.current !== "HFOV") return;

        e.preventDefault();

        const camera = perspectiveCameraRef.current;

        // adjust sensitivity
        camera.fov += e.deltaY * 0.05;

        // clamp FOV (important)
        camera.fov = THREE.MathUtils.clamp(camera.fov, 20, 100);

        camera.updateProjectionMatrix();
      });

      controls.enableDamping = true;
      controls.target.set(stationCenter.x, 0, stationCenter.z);
      perspectiveCamera.position.set(
        stationCenter.x + 2000,
        1200,
        stationCenter.z + 2500,
      );
      worldFixedConfigRef.current = {
        position: getWorldFixedPosition(getWorldFixedTarget()),
        target: getWorldFixedTarget(),
      };
      controls.update();

      controlsRef.current = controls;

      // Apply theme
      const themeConfig = getTheme(theme);
      scene.background = new THREE.Color(themeConfig.sceneBackground);
      scene.fog = new THREE.Fog(
        themeConfig.fog.color,
        themeConfig.fog.near,
        themeConfig.fog.far
      );

      // Create and store lighting references for dynamic updates
      const ambientLight = new THREE.AmbientLight(
        themeConfig.ambientLight.color,
        themeConfig.ambientLight.intensity
      );
      scene.add(ambientLight);
      ambientLightRef.current = ambientLight;

      const sun = new THREE.DirectionalLight(
        themeConfig.directionalLight.color,
        themeConfig.directionalLight.intensity
      );
      sun.position.set(
        themeConfig.directionalLight.position[0],
        themeConfig.directionalLight.position[1],
        themeConfig.directionalLight.position[2]
      );
      sun.castShadow = true;
      scene.add(sun);
      directionalLightRef.current = sun;

      const hemiLight = new THREE.HemisphereLight(
        themeConfig.hemisphereLight.skyColor,
        themeConfig.hemisphereLight.groundColor,
        themeConfig.hemisphereLight.intensity
      );
      scene.add(hemiLight);
      hemisphereLightRef.current = hemiLight;

      // ======== GROUND SETUP ========

      // 1 Load textures (grass + displacement)
      const textureLoader = new THREE.TextureLoader();

      // Grass color texture
      const grassTexture = textureLoader.load("/textures/grass.jpg");
      grassTexture.wrapS = THREE.RepeatWrapping;
      grassTexture.wrapT = THREE.RepeatWrapping;
      grassTexture.repeat.set(100, 100);
      grassTexture.anisotropy = renderer.capabilities.getMaxAnisotropy();
      grassTexture.colorSpace = THREE.SRGBColorSpace;

      // Displacement map (grayscale image for bumps)
      // You can create a simple black/white heightmap or generate one
      const displacementTexture = textureLoader.load(
        "/textures/grass_displacement.jpg",
      );
      displacementTexture.wrapS = THREE.RepeatWrapping;
      displacementTexture.wrapT = THREE.RepeatWrapping;
      displacementTexture.repeat.set(100, 100);
      displacementTexture.anisotropy = renderer.capabilities.getMaxAnisotropy();

      // 2️ Create material with displacement
      const groundMat = new THREE.MeshStandardMaterial({
        map: grassTexture,
        displacementMap: displacementTexture,
        displacementScale: 20, // bump height
        roughness: 1,
        metalness: 0,
        color: 0xe6c7a1, // ensures texture color shows
        side: THREE.DoubleSide,
      });

      // 3️ Create large plane
      const groundSize = 20000;
      const segments = 200; // high segments needed for displacement
      const groundGeo = new THREE.PlaneGeometry(
        groundSize,
        groundSize,
        segments,
        segments,
      );

      // 4️ Create mesh and rotate
      const ground = new THREE.Mesh(groundGeo, groundMat);
      ground.rotation.x = -Math.PI / 2;
      ground.position.set(stationCenter.x, 0, stationCenter.z);
      ground.receiveShadow = true;
      scene.add(ground);

      // 5️ Adjust camera so everything is visible
      perspectiveCamera.position.set(
        stationCenter.x + 2000,
        1200,
        stationCenter.z + 2500,
      );
      perspectiveCamera.updateProjectionMatrix();
      perspectiveCamera.lookAt(stationCenter);
      controlsRef.current.update();


      // ===== OBSTACLES =====
      // Load obstacles.csv in this effect so building placement doesn't race
      // against the Effect-1 loadObstacles() chain (which runs concurrently).
      const buildingLoader = new GLTFLoader();

      Promise.all([
        new Promise((resolve) => buildingLoader.load("/models/Skyscraper.glb", resolve)),
        fetch('/data/obstacles.csv')
          .then((r) => r.text())
          .then((csvText) => {
            const lines = csvText.trim().split('\n');
            const obstacles = [];
            for (let i = 1; i < lines.length; i++) {
              const parts = lines[i].split(',');
              if (parts.length < 4) continue;
              const [x, y, z, radius] = parts.map(Number);
              if (!isFinite(x) || !isFinite(z)) continue;
              obstacles.push({ id: `OBS-${String(i).padStart(3, '0')}`, x, y, z, radius });
            }
            // Populate the shared ref so Effect-1's loadObstacles is a no-op when it runs
            if (obstacleBuildingsRef.current.length === 0) {
              obstacleBuildingsRef.current = obstacles;
            }
            return obstacles;
          })
          .catch(() => obstacleBuildingsRef.current),
      ]).then(([gltf, obstacles]) => {
        const baseModel = gltf.scene;

        obstacles.forEach((b) => {
          const building = baseModel.clone(true);

          // ===== STEP 1: CENTER MODEL GEOMETRY =====
          const box = new THREE.Box3().setFromObject(building);
          const center = box.getCenter(new THREE.Vector3());

          // shift model so its center becomes (0,0,0)
          building.position.sub(center);

          // ===== STEP 2: SCALE =====
          const scale = 250;
          building.scale.set(scale, scale, scale);

          // ===== STEP 3: PLACE USING CENTER COORDINATES =====
          building.position.set(b.x, b.y, b.z);
          building.rotation.y = 0;

          // ===== STEP 4: STORE REF + APPLY CLASSIFICATION COLOUR =====
          buildingMeshMapRef.current[b.id] = building;
          const cls = obstacleClassMapRef.current[b.id];
          if (cls === 'surprise') {
            building.traverse(child => {
              if (child.isMesh) {
                child.material = child.material.clone();
                child.material.color.setHex(0xff6600);
                child.material.emissive = new THREE.Color(0x1a0800);
                child.material.emissiveIntensity = 0.25;
              }
            });
          }

          scene.add(building);
        });
      });
      //  // ===== MOUNTAINS (GLB MODELS) =====
      //       const mountainList = [
      //         // ===== REFERENCE MOUNTAINS (UNCHANGED CORE) =====
      //         { x: -100, z: 80, scale: 1.6 },
      //         { x: 250, z: 750, scale: 1.8 },
      //         { x: -100, z: 1500, scale: 1.5 },

      //         // ===== LEFT CLUSTER (shifted slightly left) =====
      //         { x: -450, z: 230, scale: 1.2 },
      //         { x: -400, z: 450, scale: 0.8 },
      //         { x: -400, z: 860, scale: 1.4 },

      //         // ===== RIGHT CLUSTER (shifted slightly right) =====
      //         { x: 700, z: 120, scale: 1.3 },
      //         { x: 750, z: 690, scale: 1.6 },
      //         { x: 650, z: 990, scale: 1.2 },

      //         // ===== LOWER AREA (behind path but safe) =====
      //         { x: -500, z: -140, scale: 1.1 },
      //         { x: 400, z: -100, scale: 1.4 },

      //         // ===== UPPER AREA (near landing but offset) =====
      //         { x: -450, z: 1100, scale: 1.3 },
      //         { x: 500, z: 1200, scale: 1.5 },

      //         // ===== DEPTH FILL (keeps hilly look) =====
      //         { x: -100, z: 1000, scale: 1.7 },
      //       ];
      //       const mountainLoader = new GLTFLoader();

      //       mountainLoader.load("/models/mountain.glb", (gltf) => {
      //         const baseModel = gltf.scene;

      //         mountainList.forEach((m, index) => {
      //           const mountain = baseModel.clone(true);

      //           // SCALE VARIATION (height difference)
      //           mountain.scale.set(200 * m.scale, 270 * m.scale, 200 * m.scale);

      //           // CENTER MODEL
      //           const box = new THREE.Box3().setFromObject(mountain);
      //           const center = box.getCenter(new THREE.Vector3());
      //           mountain.position.sub(center);

      //           //  PLACE ON GROUND
      //           const size = box.getSize(new THREE.Vector3());
      //           mountain.position.y += size.y / 2;

      //           // FINAL POSITION
      //           mountain.position.x += m.x;
      //           mountain.position.z += m.z;
      //           mountain.position.y += groundLevel;

      //           scene.add(mountain);
      //         });
      //       });


      const loader = new GLTFLoader();
      birdPathRef.current = Array.isArray(birdData)
        ? birdData
          .filter((point) =>
            Number.isFinite(point?.x)
            && Number.isFinite(point?.y)
            && Number.isFinite(point?.z))
          .map((point) => new THREE.Vector3(point.x, point.y, point.z))
        : [];
      birdIndexRef.current = 0;
      birdDirectionRef.current = 1;

      // GROUND STATION (4 PADS)
      const padOffsets = [
        { x: -90, z: -90 },
        { x: 90, z: -90 },
        { x: -90, z: 90 },
        { x: 90, z: 90 },
      ];
      padOffsets.forEach((offset, idx) => {
        const padPos = new THREE.Vector3(
          stationCenter.x + offset.x,
          0,
          stationCenter.z + offset.z,
        );
        const padMesh = new THREE.Mesh(
          new THREE.CylinderGeometry(75, 75, 8, 32),
          new THREE.MeshStandardMaterial({ color: 0x111111 }),
        );
        padMesh.position.copy(padPos).setY(4);
        scene.add(padMesh);

        if (idx !== 0) {
          loader.load("/models/evtol.glb", (gltf) => {
            gltf.scene.scale.set(2.489, 2.489, 2.489);
            gltf.scene.position.copy(padPos).setY(22 + evtolHeightOffset);
            scene.add(gltf.scene);
          });
        }
      });

      // ===== FENCING AROUND GROUND STATION =====
      const gsFenceHeight = 30;
      const gsFenceThickness = 2;
      const gsFenceLength = 500;
      const gsFenceMaterial = new THREE.MeshStandardMaterial({
        color: 0x555555,
      });

      const gsFencePositions = [
        { x: 0, y: gsFenceHeight / 2, z: gsFenceLength / 2 },
        { x: 0, y: gsFenceHeight / 2, z: -gsFenceLength / 2 },
        { x: gsFenceLength / 2, y: gsFenceHeight / 2, z: 0 },
        { x: -gsFenceLength / 2, y: gsFenceHeight / 2, z: 0 },
      ];

      const groundStationArea = new THREE.Group();
      groundStationArea.position.copy(stationCenter);
      scene.add(groundStationArea);

      gsFencePositions.forEach((pos, index) => {
        let geometry;
        if (index < 2)
          geometry = new THREE.BoxGeometry(
            gsFenceLength,
            gsFenceHeight,
            gsFenceThickness,
          );
        else
          geometry = new THREE.BoxGeometry(
            gsFenceThickness,
            gsFenceHeight,
            gsFenceLength,
          );

        const fenceSegment = new THREE.Mesh(geometry, gsFenceMaterial);
        fenceSegment.position.set(pos.x, pos.y, pos.z);
        groundStationArea.add(fenceSegment);
      });

      // ===== VERTIPORT TEXTURE LOADER =====
      function createVertiportPad(position, isSource) {
        const texLoader = new THREE.TextureLoader();
        texLoader.load(
          "/vertiport.png",
          (tex) => {
            tex.colorSpace = THREE.SRGBColorSpace;
            const padMat = new THREE.MeshStandardMaterial({
              map: tex,
              roughness: 0.6,
              metalness: 0.1,
              color: isSource ? 0xffffff : 0xffffff,
            });
            // Main flat disc
            const padGeo = new THREE.CylinderGeometry(75, 75, 8, 64);
            const padMesh = new THREE.Mesh(padGeo, padMat);
            padMesh.position.copy(position);
            padMesh.position.y = 4;
            scene.add(padMesh);

            // Outer glow ring
            const ringGeo = new THREE.RingGeometry(80, 90, 64);
            const ringMat = new THREE.MeshBasicMaterial({
              color: isSource ? 0x00ff88 : 0xffd700,
              side: THREE.DoubleSide,
              transparent: true,
              opacity: 0.7,
            });
            const ring = new THREE.Mesh(ringGeo, ringMat);
            ring.rotation.x = -Math.PI / 2;
            ring.position.copy(position);
            ring.position.y = 9;
            scene.add(ring);

            // Dashed outer indicator ring
            const outerRingGeo = new THREE.RingGeometry(95, 98, 64);
            const outerRingMat = new THREE.MeshBasicMaterial({
              color: isSource ? 0x00ff88 : 0xffd700,
              side: THREE.DoubleSide,
              transparent: true,
              opacity: 0.4,
            });
            const outerRing = new THREE.Mesh(outerRingGeo, outerRingMat);
            outerRing.rotation.x = -Math.PI / 2;
            outerRing.position.copy(position);
            outerRing.position.y = 9;
            scene.add(outerRing);
          },
          undefined,
          () => {
            // Fallback: plain colored pad if image not found
            const padMat = new THREE.MeshStandardMaterial({ color: isSource ? 0x111111 : 0xffd700 });
            const padMesh = new THREE.Mesh(new THREE.CylinderGeometry(75, 75, 8, 32), padMat);
            padMesh.position.copy(position);
            padMesh.position.y = 4;
            scene.add(padMesh);
          }
        );
      }

      // SOURCE vertiport (ground station takeoff pad)
      createVertiportPad(startPos.clone().setY(0), true);

      // TARGET PAD — vertiport image
      createVertiportPad(landingPadPos.clone(), false);

      // TARGET PAD (keep collision reference mesh, invisible)
      const targetPad = new THREE.Mesh(
        new THREE.CylinderGeometry(75, 75, 8, 32),
        new THREE.MeshStandardMaterial({ color: 0xffd700, transparent: true, opacity: 0 }),
      );
      targetPad.position.copy(landingPadPos);
      targetPad.position.y = 4;
      scene.add(targetPad);

      // ACTIVE MODELS
      const evtolGroup = new THREE.Group();
      evtolGroup.position.copy(getInitialFlightPosition());
      scene.add(evtolGroup);
      evtolRef.current = evtolGroup;

      const fpvOffset = new THREE.Object3D();
      fpvOffset.position.set(0, 25, 80);
      // ↑ tweak values:
      // Y = cockpit height
      // Z = front nose
      fpvRef.current = fpvOffset;
      evtolGroup.add(fpvOffset);

      // ===== DOWNWARD-FACING CAMERA (mounted under eVTOL) =====
      const downCamW = 360;
      const downCamH = 200;
      const downCamRenderer = new THREE.WebGLRenderer({ antialias: false, preserveDrawingBuffer: true });
      downCamRenderer.setSize(downCamW, downCamH);
      downCamRenderer.setPixelRatio(1);
      downCamRendererRef.current = downCamRenderer;

      const downCam = new THREE.PerspectiveCamera(DOWN_CAM_FOV, downCamW / downCamH, 0.1, 50000);
      // Position directly under evtol
      downCam.position.set(0, DOWN_CAM_HEIGHT_OFFSET, 0);
      downCam.rotation.x = -Math.PI / 2; // point straight down
      downCamRef.current = downCam;
      evtolGroup.add(downCam);

      loader.load("/models/evtol.glb", (gltf) => {
        const model = gltf.scene;
        model.scale.set(2.489, 2.489, 2.489);
        const box = new THREE.Box3().setFromObject(model);
        const minY = box.min.y;
        model.position.y -= minY; // align bottom to ground
        evtolGroup.add(model);
      });

      loader.load("/models/bird.glb", (gltf) => {
        const bird = gltf.scene;
        bird.scale.set(0.3, 0.3, 0.3);
        if (birdPathRef.current.length > 0) {
          bird.position.copy(birdPathRef.current[0]);
        } else {
          bird.position.set(-300, 325, -330);
        }
        scene.add(bird);
        birdRef.current = bird;
      });

      //Person
      const pGroup = new THREE.Group();
      const targetPosition = landingPadPos.clone();
      targetPad.position.copy(landingPadPos);
      // place person BESIDE target pad
      pGroup.position.set(
        targetPosition.x - 120, // right side of pad
        0,
        targetPosition.z, // same line
      );
      scene.add(pGroup);
      personRef.current = pGroup;
      loader.load("/models/person.glb", (gltf) => {
        gltf.scene.scale.set(60, 60, 60);
        pGroup.add(gltf.scene);
      });

      //FUEL-STATION(LEFT-SIDE)
      let chimney;

      loader.load("/models/Chimney.glb", (gltf) => {
        chimney = gltf.scene;
        chimney.scale.set(2, 1, 2);

        // ===== FIX ALIGNMENT =====
        const box = new THREE.Box3().setFromObject(chimney);
        const center = box.getCenter(new THREE.Vector3());
        chimney.position.sub(center);
        const size = box.getSize(new THREE.Vector3());
        chimney.position.y += size.y / 2;

        // ===== PLACE IN 4TH QUADRANT =====
        chimney.position.x += stationCenter.x + 5000; // +X
        chimney.position.z += stationCenter.z + 6000; // -Z
        scene.add(chimney);

        // ===== SMOKE =====
        const newBox = new THREE.Box3().setFromObject(chimney);
        const top = newBox.max.clone().add(new THREE.Vector3(0, 5, 0));
      });

      const fuelTankLoader = new GLTFLoader();
      fuelTankLoader.load("/models/fuel_tank.glb", (gltf) => {
        const baseTank = gltf.scene;
        // ===== FUEL AREA CONFIG =====
        const fuelGroup = new THREE.Group();
        scene.add(fuelGroup);
        const baseX = 4000; // fixed X (left-right position)
        const baseZ = 8000; // starting Z position
        const spacing = 500; // distance between tanks
        const tankCount = 4;

        for (let i = 0; i < tankCount; i++) {
          const tank = baseTank.clone(true);
          tank.scale.set(250, 250, 250);

          // ===== VERTICAL LINE ARRANGEMENT =====
          const offset = (i - (tankCount - 1) / 2) * spacing;
          tank.position.set(
            baseX, // keep X fixed
            0,
            baseZ + offset, // spread vertically (Z direction)
          );

          // ===== ROTATION =====
          tank.rotation.y = Math.PI / 2;

          // ===== SHADOWS =====
          tank.traverse((child) => {
            if (child.isMesh) {
              child.castShadow = true;
              child.receiveShadow = true;
            }
          });
          fuelGroup.add(tank);
        }
      });

      const truckLoader1 = new GLTFLoader();
      if (!sceneRef.current) return;
      truckLoader1.load("/models/M939_Truck.glb", (gltf) => {
        const baseTruck = gltf.scene;
        const truckGroup = new THREE.Group();
        sceneRef.current.add(truckGroup);

        const fuelCenterX = 6500;
        const fuelCenterZ = 7500;
        const truckCount = 2;

        for (let i = 0; i < truckCount; i++) {
          const truck = SkeletonUtils.clone(baseTruck);
          truck.scale.set(150, 150, 150);
          truck.position.set(
            fuelCenterX - 450 + i * 30,
            0,
            fuelCenterZ + i * 1000,
          );
          truck.rotation.y = 0;
          truck.traverse((child) => {
            if (child.isMesh) {
              child.castShadow = true;
              child.receiveShadow = true;
            }
          });
          truckGroup.add(truck);
        }
      });

      const fenceLoader1 = new GLTFLoader();
      if (!sceneRef.current) return;
      fenceLoader1.load("/models/Fence.glb", (gltf) => {
        const baseFence = gltf.scene;
        const fenceGroup = new THREE.Group();
        sceneRef.current.add(fenceGroup);

        // FUEL STATION CENTER (your values)
        const centerX = 5000;
        const centerZ = 8000;

        // RECTANGLE SIZE (adjust as needed)
        const width = 2000;
        const depth = 1400;
        const spacing = 300; // distance between fence pieces

        // ================= FRONT SIDE =================
        for (let x = -width; x <= width; x += spacing) {
          const fence = SkeletonUtils.clone(baseFence);
          fence.position.set(centerX + x, 0, centerZ + depth);
          fence.rotation.y = 0;
          fence.scale.set(50, 70, 50);
          fenceGroup.add(fence);
        }

        // ================= BACK SIDE =================
        for (let x = -width; x <= width; x += spacing) {
          const fence = SkeletonUtils.clone(baseFence);
          fence.position.set(centerX + x, 0, centerZ - depth);
          fence.rotation.y = 0;
          fence.scale.set(50, 70, 50);
          fenceGroup.add(fence);
        }

        // ================= LEFT SIDE =================
        for (let z = -depth; z <= depth; z += spacing) {
          const fence = SkeletonUtils.clone(baseFence);
          fence.position.set(centerX - width, 0, centerZ + z);
          fence.rotation.y = Math.PI / 2;
          fence.scale.set(50, 70, 50);
          fenceGroup.add(fence);
        }

        // ================= RIGHT SIDE =================
        for (let z = -depth; z <= depth; z += spacing) {
          const fence = SkeletonUtils.clone(baseFence);
          fence.position.set(centerX + width, 0, centerZ + z);
          fence.rotation.y = Math.PI / 2;
          fence.scale.set(50, 70, 50);
          fenceGroup.add(fence);
        }
      });

      // ================== MILITARY CAMP (RIGHT SIDE) ==================
      // ================== CAMP GROUP ==================
      const campGroup = new THREE.Group();

      campGroup.position.copy(landingPadPos);
      campGroup.position.x += 100;
      campGroup.position.z += 50;

      scene.add(campGroup);

      // ================== MILTARY TENTS ==================
      loader.load("/models/military_tent.glb", (gltf) => {
        const baseTent = gltf.scene;
        baseTent.scale.set(120, 150, 170);

        const tentPositions = [
          { x: 80, z: -100 }, // right
        ];

        tentPositions.forEach((pos) => {
          const tent = baseTent.clone();
          tent.position.set(pos.x, 0, pos.z);
          campGroup.add(tent);
        });
      });
      // ================== MILITARY CAMP FENCE ==================
      const fence2Loader = new GLTFLoader();

      fence2Loader.load("/models/Fence.glb", (gltf) => {
        const baseFence = gltf.scene;

        addCampFence(baseFence);
      });
      const tankLoader = new GLTFLoader();

      tankLoader.load("/models/tank.glb", (gltf) => {
        const baseTank = gltf.scene;

        addTanksInsideFence(baseTank);
      });

      function addCampFence(baseModel) {
        const fenceGroup = new THREE.Group();
        campGroup.add(fenceGroup);

        //  RECTANGLE DIMENSIONS
        const halfWidth = 1580; // X (left-right, smaller)
        const frontDepth = 4800; // Z+ (front side → elongated)
        const backDepth = 1500; // Z- (behind tent → shorter)

        const spacing = 300; // distance between fence pieces

        const positions = [];

        // =========================
        // FRONT SIDE (LONG SIDE )
        // =========================
        for (let x = -halfWidth; x <= halfWidth; x += spacing) {
          positions.push({ x: x, z: frontDepth, rot: 0 });
        }

        // =========================
        // BACK SIDE (SHORT SIDE)
        // =========================
        for (let x = -halfWidth; x <= halfWidth; x += spacing) {
          positions.push({ x: x, z: -backDepth, rot: 0 });
        }

        // =========================
        // LEFT SIDE
        // =========================
        for (let z = -backDepth; z <= frontDepth; z += spacing) {
          positions.push({ x: -halfWidth, z: z, rot: Math.PI / 2 });
        }

        // =========================
        // RIGHT SIDE
        // =========================
        for (let z = -backDepth; z <= frontDepth; z += spacing) {
          positions.push({ x: halfWidth, z: z, rot: Math.PI / 2 });
        }

        // =========================
        // PLACE FENCES
        // =========================
        const offsetX = 2990; //  shift right

        positions.forEach((p) => {
          const fence = baseModel.clone(true);

          fence.scale.set(50, 70, 50);

          const box = new THREE.Box3().setFromObject(fence);
          const center = box.getCenter(new THREE.Vector3());
          fence.position.sub(center);

          const size = box.getSize(new THREE.Vector3());
          fence.position.y += size.y / 2;

          fence.rotation.y = p.rot;

          //  APPLY OFFSET HERE
          fence.position.x = p.x + offsetX;
          fence.position.z = p.z;

          fenceGroup.add(fence);
        });
      }

      function addTanksInsideFence(baseModel) {
        const tankGroup = new THREE.Group();
        campGroup.add(tankGroup);

        const offsetX = 3200;

        const tankCount = 4;

        // FORMATION SETTINGS
        const startX = -1200; // starting left
        const startZ = 1300; // starting depth

        const stepX = 100; // small shift → creates slant
        const stepZ = 950; // big forward spacing (vertical feel)

        for (let i = 0; i < tankCount; i++) {
          const tank = baseModel.clone(true);

          // ===== SCALE =====
          tank.scale.set(90.5, 130, 90.5);

          // ===== FORMATION POSITION (SLANTED LINE) =====
          const x = startX + i * stepX; // slight right shift each tank
          const z = startZ + i * stepZ; // forward progression

          // ===== CENTER MODEL =====
          const box = new THREE.Box3().setFromObject(tank);
          const center = box.getCenter(new THREE.Vector3());
          tank.position.sub(center);

          const minY = box.min.y;

          tank.position.set(x + offsetX, groundLevel - minY, z);

          // ===== ALIGN ROTATION=====
          // All tanks face same direction (clean military look)
          tank.rotation.y = Math.PI * 0.75; // adjust if needed

          tankGroup.add(tank);
        }
      }

      //Truck
      const truckLoader = new GLTFLoader();

      truckLoader.load("/models/Truckk.glb", (gltf) => {
        const baseTruck = gltf.scene;

        addTrucksInsideFence(baseTruck);
      });
      function addTrucksInsideFence(baseModel) {
        const truckGroup = new THREE.Group();
        campGroup.add(truckGroup);

        const offsetX = 2990; // same as fence

        // ===== FORMATION SETTINGS (RIGHT SIDE) =====
        const startX = 700; // right side inside fence
        const startZ = 1500; // near mid area

        const stepZ = 900; // spacing between trucks

        const truckCount = 3;

        for (let i = 0; i < truckCount; i++) {
          const truck = baseModel.clone(true);

          // ===== SCALE =====
          truck.scale.set(80, 80, 80);

          // ===== POSITION (VERTICAL PARKING LINE) =====
          const x = startX; // fixed right side
          const z = startZ + i * stepZ; // forward spacing

          // ===== CENTER MODEL =====
          const box = new THREE.Box3().setFromObject(truck);
          const center = box.getCenter(new THREE.Vector3());
          truck.position.sub(center);

          const minY = box.min.y;

          truck.position.set(x + offsetX, groundLevel - minY, z);

          // ===== PARKING ROTATION (SLIGHT ANGLE) =====
          truck.rotation.y = Math.PI * 0.15; // slight angle

          truckGroup.add(truck);
        }
      }
      //Storage boxes
      const crateLoader = new GLTFLoader();

      crateLoader.load("/models/Crate.glb", (gltf) => {
        const baseCrate = gltf.scene;

        addCrates(baseCrate);
      });

      function addCrates(baseModel) {
        const crateGroup = new THREE.Group();
        campGroup.add(crateGroup);

        const offsetX = 2990; // same as fence/tent alignment

        // 👉 BACK OF TENT AREA (negative Z side)
        const baseX = 1000; // near tent center
        const baseZ = -900; // behind tent

        // =========================
        // 🔲 MAIN STACK (2 BOXES)
        // =========================
        for (let i = 0; i < 2; i++) {
          const crate = baseModel.clone(true);

          crate.scale.set(1700, 1700, 1700);

          const box = new THREE.Box3().setFromObject(crate);
          const center = box.getCenter(new THREE.Vector3());
          crate.position.sub(center);

          const size = box.getSize(new THREE.Vector3());
          const minY = box.min.y;

          crate.position.set(
            baseX + offsetX,
            groundLevel - minY + i * size.y, // 🔥 stacking
            baseZ,
          );

          crateGroup.add(crate);
        }

        // =========================
        // 📦 SIDE CRATES (SCATTER)
        // =========================
        const sideOffsets = [
          { x: -70, z: -450 },
          { x: 170, z: -1300 },
          { x: 190, z: -900 },
        ];

        sideOffsets.forEach((pos) => {
          const crate = baseModel.clone(true);

          crate.scale.set(1700, 1700, 1700);

          const box = new THREE.Box3().setFromObject(crate);
          const center = box.getCenter(new THREE.Vector3());
          crate.position.sub(center);

          const minY = box.min.y;

          crate.position.set(
            baseX + pos.x + offsetX,
            groundLevel - minY,
            baseZ + (pos.z - baseZ),
          );

          crate.rotation.y = Math.random() * Math.PI * 2;

          crateGroup.add(crate);
        });
      }

      // ROAD
      const rockLoader = new GLTFLoader();

      rockLoader.load("/models/Rock Path Round Wide.glb", (gltf) => {
        const baseRock = gltf.scene;

        addRockPath(baseRock);
      });

      function addRockPath(baseModel) {
        const rockGroup = new THREE.Group();
        campGroup.add(rockGroup);

        //  PATH SETTINGS
        const pathStartZ = 0; // tent opening (center)
        const pathEndZ = 4800; // till fence front
        const spacing = 180; // distance between rocks

        const pathWidth = 120; // slight width (not a single line)

        for (let z = pathStartZ; z <= pathEndZ; z += spacing) {
          // add 2–3 rocks per row for width
          for (let i = -1; i <= 1; i++) {
            const rock = baseModel.clone(true);

            //  RANDOM WIDTH OFFSET (natural look)
            const offsetX = i * pathWidth + (Math.random() - 0.5) * 40;

            //  SLIGHT FORWARD VARIATION
            const offsetZ = z + (Math.random() - 0.5) * 30;

            // SCALE variation
            const scale = 100 + Math.random() * 30;
            rock.scale.set(scale, scale, scale);

            // CENTER MODEL
            const box = new THREE.Box3().setFromObject(rock);
            const center = box.getCenter(new THREE.Vector3());
            rock.position.sub(center);

            const minY = box.min.y;

            // POSITION (relative to camp)
            const tentOffsetX = 3100;
            const tentOffsetZ = -50;

            rock.position.set(
              offsetX + tentOffsetX,
              groundLevel - minY,
              offsetZ + tentOffsetZ,
            );

            // RANDOM ROTATION
            rock.rotation.y = Math.random() * Math.PI * 2;

            rockGroup.add(rock);
          }
        }
      }
      // ================== SOLDIERS (LOAD ONCE, CLONE MANY) ==================

      loader.load("/models/guard_soldier.glb", (gltf) => {
        const baseSoldier = gltf.scene;
        const soldierPositions = [
          { x: 1600, z: -300 },
          { x: 1100, z: 300 },
          { x: 1300, z: -500 },
        ];
        soldierPositions.forEach((pos) => {
          const soldier = SkeletonUtils.clone(baseSoldier);
          soldier.scale.set(5, 5, 5);
          soldier.position.set(pos.x, 5, pos.z);
          soldier.rotation.y = Math.PI;
          campGroup.add(soldier);
        });
      });

      // ===== DECORATED HOSPITAL AREA =====
      const hospitalArea = new THREE.Group();
      hospitalArea.position.set(
        stationCenter.x - 900,
        0,
        stationCenter.z + 900,
      );
      hospitalArea.scale.set(1.3, 1.3, 1.3); //  bigger area
      scene.add(hospitalArea);

      // --- Hospital Sign ---
      const sign = new THREE.Mesh(
        new THREE.BoxGeometry(30, 15, 2),
        new THREE.MeshStandardMaterial({ color: 0xff0000 }),
      );
      sign.position.set(-50, 20, 0);
      hospitalArea.add(sign);

      //Hospital
      loader.load("/models/Hospital.glb", (gltf) => {
        const hospital = gltf.scene;

        hospital.scale.set(1.8, 2, 1.8); // adjust as needed
        hospital.position.set(-2500, 198, 1000);
        scene.add(hospital); // attach to hospitalArea
      });

      //radio tower
      loader.load("/models/radiotower.glb", (gltf) => {
        const radio = gltf.scene;

        radio.scale.set(150, 150, 150);
        radio.position.set(
          stationCenter.x + 100,
          stationCenter.y + 340,
          stationCenter.z + -300,
        );

        scene.add(radio);
      });

      // ================== AMBULANCE (LOAD ONCE, CLONE MANY) ==================
      loader.load("/models/Ambulance.glb", (gltf) => {
        const baseAmbulance = gltf.scene;

        const ambulancePositions = [
          { x: stationCenter.x - 700, z: stationCenter.z + 600 },
          { x: stationCenter.x - 500, z: stationCenter.z + 900 },
        ];

        ambulancePositions.forEach((pos, index) => {
          let ambulance;

          if (index === 0) {
            ambulance = baseAmbulance; // first one original
          } else {
            ambulance = baseAmbulance.clone(true); // deep clone
          }

          ambulance.scale.set(11, 11, 11);
          ambulance.position.set(pos.x, 0, pos.z);

          scene.add(ambulance);
        });
      });

      // ==========================
      // SEEDED RANDOM (STABLE)
      // ==========================
      function seededRandom(seed) {
        let x = Math.sin(seed * 9999) * 10000;
        return x - Math.floor(x);
      }

      // ==========================
      // BUILDING POSITIONS
      // ==========================
      const buildingPositions = [];

      const groundHalf = 20000 / 2;

      // LEFT SIDE ZONE ONLY (town area)
      const minX = stationCenter.x - groundHalf + 1500;
      const maxX = stationCenter.x - 2000;

      const minZ = stationCenter.z - groundHalf + 1500;
      const maxZ = stationCenter.z + groundHalf - 1500;

      for (let i = 0; i < 40; i++) {
        let x, z;

        do {
          x = THREE.MathUtils.lerp(minX, maxX, seededRandom(i));
          z = THREE.MathUtils.lerp(minZ, maxZ, seededRandom(i + 50));
        } while (
          z > stationCenter.z + 200 &&
          z < stationCenter.z + 500 // avoid EVTOL corridor
        );

        buildingPositions.push({
          x,
          z,
          scale: 120 + seededRandom(i + 100) * 40,
          rotation: seededRandom(i + 200) * Math.PI * 2,
        });
      }

      // ==========================
      // LOAD BUILDINGS
      // ==========================
      loader.load("/models/large_building.glb", (gltf) => {
        const baseModel = gltf.scene;

        buildingPositions.forEach((pos) => {
          const building = baseModel.clone(true);

          building.scale.set(pos.scale * 1.9, pos.scale * 1.9, pos.scale * 1.9);

          const box = new THREE.Box3().setFromObject(building);
          const center = box.getCenter(new THREE.Vector3());
          building.position.sub(center);

          const size = box.getSize(new THREE.Vector3());
          building.position.y += size.y / 2;

          building.position.x += pos.x;
          building.position.z += pos.z;

          building.rotation.y = pos.rotation;

          scene.add(building);
        });
      });

      // ==========================
      // LOAD TREES
      // ==========================
      loader.load("/models/Tree.glb", (gltf) => {
        const treeBase = gltf.scene;

        // CITY AREA (same as buildings)
        const groundHalf = 20000 / 2;

        const minX = stationCenter.x - groundHalf + 1500;
        const maxX = stationCenter.x - 2000;

        const minZ = stationCenter.z - groundHalf + 1500;
        const maxZ = stationCenter.z + groundHalf - 1500;

        const hospitalCenter = hospitalArea.position;
        const hospitalRadius = 800;

        const treeCount = 100; // ONLY FEW TREES

        for (let i = 0; i < treeCount; i++) {
          let x = THREE.MathUtils.lerp(minX, maxX, Math.random());
          let z = THREE.MathUtils.lerp(minZ, maxZ, Math.random());

          // avoid flight path
          if (z > stationCenter.z + 200 && z < stationCenter.z + 500) continue;

          //  avoid hospital
          const distToHospital = Math.sqrt(
            (x - hospitalCenter.x) ** 2 + (z - hospitalCenter.z) ** 2,
          );
          if (distToHospital < hospitalRadius) continue;

          const tree = treeBase.clone(true);

          const scale = 18 + Math.random() * 6; // smaller variation
          tree.scale.set(scale * 2.93, scale * 2.93, scale * 2.93);

          // ground fix
          const box = new THREE.Box3().setFromObject(tree);
          const minY = box.min.y;

          tree.position.set(x, groundLevel - minY, z);
          tree.rotation.y = Math.random() * Math.PI * 2;

          scene.add(tree);
        }
      });

      //Fencing around hospital
      const fenceLoader = new GLTFLoader();

      fenceLoader.load("/models/wooden_fence.glb", (gltf) => {
        const fenceBase = gltf.scene;

        addFenceAroundHospital(fenceBase);
      });

      function addFenceAroundHospital(baseModel) {
        const half = 450; // since fenceLength = 1000
        const spacing = 80; // distance between fence pieces

        const positions = [];

        // FRONT & BACK (X direction)
        for (let x = -half; x <= half; x += spacing) {
          positions.push({ x, z: -half, rot: 0 }); // front
          positions.push({ x, z: half, rot: 0 }); // back
        }

        // LEFT & RIGHT (Z direction)
        for (let z = -half; z <= half; z += spacing) {
          positions.push({ x: -half, z, rot: Math.PI / 2 }); // left
          positions.push({ x: half, z, rot: Math.PI / 2 }); // right
        }

        positions.forEach((p) => {
          const fence = baseModel.clone(true);

          // scale if needed
          fence.scale.set(70, 230, 70);

          // center model
          const box = new THREE.Box3().setFromObject(fence);
          const center = box.getCenter(new THREE.Vector3());
          fence.position.sub(center);

          const size = box.getSize(new THREE.Vector3());
          fence.position.y += size.y / 2;

          // apply rotation
          fence.rotation.y = p.rot;

          // position relative to hospitalArea
          fence.position.set(p.x, 0, p.z);
          hospitalArea.add(fence);
        });
      }
      //Fuel tanks
      const fuelLoader = new GLTFLoader();

      fuelLoader.load("/models/FuelTank.glb", (gltf) => {
        const baseTank = gltf.scene;

        addFuelTanks(baseTank);
      });
      function addFuelTanks(baseModel) {
        const fuelGroup = new THREE.Group();
        scene.add(fuelGroup);

        // ===== POSITION (1st QUADRANT, FAR FROM CAMP) =====
        const baseX = stationCenter.x + 4000; // far right
        const baseZ = stationCenter.z - 9000; // far forward

        const gapX = 300; // spacing between side-by-side tanks
        const gapZ = 800; // spacing between rows

        const positions = [
          // FRONT ROW (facing forward)
          { x: -gapX, z: 0, rot: 0 },
          { x: gapX, z: 0, rot: 0 },

          // BACK ROW (facing opposite)
          { x: -gapX, z: gapZ, rot: Math.PI },
          { x: gapX, z: gapZ, rot: Math.PI },
        ];

        positions.forEach((p) => {
          const tank = baseModel.clone(true);

          // ===== SCALE =====
          tank.scale.set(180, 180, 180);

          // ===== CENTER MODEL =====
          const box = new THREE.Box3().setFromObject(tank);
          const center = box.getCenter(new THREE.Vector3());
          tank.position.sub(center);

          const minY = box.min.y;

          // ===== FINAL POSITION =====
          tank.position.set(baseX + p.x, groundLevel - minY, baseZ + p.z);

          // ===== ROTATION (IMPORTANT) =====
          tank.rotation.y = p.rot;

          fuelGroup.add(tank);
        });
      }
      // ================== FUEL TANK FENCE ==================
      const fuelFenceLoader = new GLTFLoader();

      fuelFenceLoader.load("/models/Fence End.glb", (gltf) => {
        const baseFence = gltf.scene;

        addFuelFence(baseFence);
      });

      function addFuelFence(baseModel) {
        const fenceGroup = new THREE.Group();
        scene.add(fenceGroup);

        // SAME CENTER AS FUEL TANKS
        const baseX = stationCenter.x + 4000;
        const baseZ = stationCenter.z - 7800;

        //  Fence size (adjust if needed)
        const halfWidth = 2800; // X direction
        const halfDepth = 1900; // Z direction

        const spacing = 230;

        const positions = [];

        // ===== FRONT =====
        for (let x = -halfWidth; x <= halfWidth; x += spacing) {
          positions.push({ x, z: halfDepth, rot: 0 });
        }

        // ===== BACK =====
        for (let x = -halfWidth; x <= halfWidth; x += spacing) {
          positions.push({ x, z: -halfDepth, rot: 0 });
        }

        // ===== LEFT =====
        for (let z = -halfDepth; z <= halfDepth; z += spacing) {
          positions.push({ x: -halfWidth, z, rot: Math.PI / 2 });
        }

        // ===== RIGHT =====
        for (let z = -halfDepth; z <= halfDepth; z += spacing) {
          positions.push({ x: halfWidth, z, rot: Math.PI / 2 });
        }

        // ===== PLACE FENCE =====
        positions.forEach((p) => {
          const fence = baseModel.clone(true);

          // scale
          fence.scale.set(180, 550, 180);

          // center fix
          const box = new THREE.Box3().setFromObject(fence);
          const center = box.getCenter(new THREE.Vector3());
          fence.position.sub(center);

          const minY = box.min.y;

          // final position
          fence.position.set(baseX + p.x, groundLevel - minY, baseZ + p.z);

          fence.rotation.y = p.rot;

          fenceGroup.add(fence);
        });
      }

      // ================== FUEL AREA BUILDINGS ==================
      const fuelBuildingLoader = new GLTFLoader();

      fuelBuildingLoader.load("/models/FuelPort.glb", (gltf) => {
        const baseBuilding = gltf.scene;

        addFuelBuildings(baseBuilding);
      });

      function addFuelBuildings(baseModel) {
        const buildingGroup = new THREE.Group();
        scene.add(buildingGroup);

        // SAME CENTER AS FENCE
        const baseX = stationCenter.x + 4300;
        const baseZ = stationCenter.z - 7800;

        const halfWidth = 2800;

        // 👉 LEFT SIDE (inside fence)
        const offsetFromFence = 800; // distance from fence wall

        const positions = [
          { x: -halfWidth + offsetFromFence, z: -500 },
          { x: -halfWidth + offsetFromFence, z: 800 },
        ];

        positions.forEach((p) => {
          const building = baseModel.clone(true);

          // ===== SCALE =====
          building.scale.set(150, 100, 190);

          // ===== CENTER FIX =====
          const box = new THREE.Box3().setFromObject(building);
          const center = box.getCenter(new THREE.Vector3());
          building.position.sub(center);

          const minY = box.min.y;

          // ===== POSITION =====
          building.position.set(baseX + p.x, groundLevel - minY, baseZ + p.z);

          // ===== FACE TOWARDS TANKS (RIGHT SIDE) =====
          building.rotation.y = 0; // facing +X direction

          buildingGroup.add(building);
        });
      }
      // ==========================
      // RADIO TOWERS (Q2, Q3, Q4)
      // ==========================

      loader.load("/models/transmission_tower.glb", (gltf) => {
        const baseTower = gltf.scene;
        const groundHalf = 20000 / 2;
        const minX = stationCenter.x - groundHalf + 1000;
        const maxX = stationCenter.x + groundHalf - 1000;
        const minZ = stationCenter.z - groundHalf + 1000;
        const maxZ = stationCenter.z + groundHalf - 1000;
        const towers = [];
        const placed = [];
        const MIN_DISTANCE = 800; //  spacing between towers
        function isFarEnough(x, z) {
          return placed.every((p) => {
            const dx = p.x - x;
            const dz = p.z - z;
            return Math.sqrt(dx * dx + dz * dz) > MIN_DISTANCE;
          });
        }
        function generateTowers(count, quadrantFn, seedOffset) {
          let created = 0;
          let attempts = 0;
          while (created < count && attempts < 50) {
            const seed = seedOffset + attempts;
            const { x, z } = quadrantFn(seed);
            if (isFarEnough(x, z)) {
              towers.push({ x, z });
              placed.push({ x, z });
              created++;
            }
            attempts++;
          }
        }

        // -------- 2nd Quadrant (-x, +z) --------

        generateTowers(
          3,
          (seed) => ({
            x: THREE.MathUtils.lerp(
              minX,
              stationCenter.x - 500,
              seededRandom(seed * 1.3),
            ),
            z: THREE.MathUtils.lerp(
              stationCenter.z + 500,
              maxZ,
              seededRandom(seed * 2.1),
            ),
          }),
          10,
        );

        // -------- 3rd Quadrant (-x, -z) --------

        generateTowers(
          3,
          (seed) => ({
            x: THREE.MathUtils.lerp(
              minX,
              stationCenter.x - 500,
              seededRandom(seed * 1.7),
            ),
            z: THREE.MathUtils.lerp(
              minZ,
              stationCenter.z - 500,
              seededRandom(seed * 2.5),
            ),
          }),
          100,
        );

        // -------- Place towers --------
        towers.forEach((pos, index) => {
          const tower = baseTower.clone(true);
          const scale = 150 + seededRandom(index + 300) * 50;
          tower.scale.set(scale, scale, scale);
          const box = new THREE.Box3().setFromObject(tower);
          const center = box.getCenter(new THREE.Vector3());
          tower.position.sub(center);
          const minY = box.min.y;
          tower.position.set(pos.x, groundLevel - minY, pos.z);
          tower.rotation.y = seededRandom(index + 400) * Math.PI * 2;
          scene.add(tower);
        });
      });


      // ===== ANIMATION =====

      let lastTime = 0;

      function animate(time) {
        animationId = requestAnimationFrame(animate);

        const delta = lastTime ? (time - lastTime) * 0.001 : 0;
        lastTime = time;

        const evtol = evtolRef.current;
        const camera = perspectiveCameraRef.current;
        const controls = controlsRef.current;

        const traj = trajectoryRef.current;

        if (!traj || traj.length === 0 || !evtol) {
          renderer.render(scene, camera);
          return;
        }

        // ================== PERSON VISIBILITY ==================
        if (personRef.current) {
          personRef.current.visible = !sim.current.personPicked;
        }

        const bird = birdRef.current;
        const birdPath = birdPathRef.current;
        if (bird && birdPath.length > 1 && sim.current.isRunning) {
          const birdSpeed = 10.0;
          birdIndexRef.current += birdDirectionRef.current * birdSpeed * delta * 60;

          if (birdIndexRef.current >= birdPath.length - 1) {
            birdIndexRef.current = birdPath.length - 1;
            birdDirectionRef.current = -1;
          }

          if (birdIndexRef.current <= 0) {
            birdIndexRef.current = 0;
            birdDirectionRef.current = 1;
          }

          const birdIdx = Math.floor(birdIndexRef.current);
          const birdT = birdIndexRef.current - birdIdx;
          const p1 = birdPath[birdIdx];
          const p2 = birdPath[Math.min(birdIdx + 1, birdPath.length - 1)];

          if (p1 && p2) {
            const birdPos = new THREE.Vector3().lerpVectors(p1, p2, birdT);
            bird.position.copy(birdPos);

            const direction = birdDirectionRef.current === 1
              ? p2.clone().sub(p1).normalize()
              : p1.clone().sub(p2).normalize();

            if (direction.lengthSq() > 0.0001) {
              const angle = Math.atan2(direction.x, direction.z);
              bird.rotation.set(0, angle - Math.PI / 2, 0);
            }

            // ── Real-time dynamic obstacle tracking ──────────────────────────
            // Keep the bird's entry in knownObstaclesRef in sync with its
            // current animated position so the proactive lookahead avoids
            // where the bird IS, not where it was first spotted.
            // We do NOT clear detectedObstacleKeysRef here — the new grid-cell
            // key is naturally unhandled, so the lookahead will pick it up on
            // its next 0.5 s tick without triggering cascading detours.
            const BIRD_ID = "BIRD-DYNAMIC-1";
            const BIRD_RADIUS = 55;
            const existing = knownObstaclesRef.current;
            const birdEntryIdx = existing.findIndex(o => o.id === BIRD_ID);
            if (birdEntryIdx >= 0) {
              const prev = existing[birdEntryIdx];
              const oldKey = makeDetectionKey({ X_world: prev.x, Z_world: prev.z, radius: BIRD_RADIUS });
              const newKey = makeDetectionKey({ X_world: birdPos.x, Z_world: birdPos.z, radius: BIRD_RADIUS });
              if (oldKey !== newKey) {
                // Bird crossed a new 120 m grid cell — update position only.
                const updated = [...existing];
                updated[birdEntryIdx] = { ...prev, x: birdPos.x, y: birdPos.y, z: birdPos.z };
                knownObstaclesRef.current = updated;
              }
            } else {
              // First time we see the bird — add it to the known set.
              knownObstaclesRef.current = [...existing, {
                id: BIRD_ID,
                label: "bird",
                x: birdPos.x,
                y: birdPos.y,
                z: birdPos.z,
                radius: BIRD_RADIUS,
              }];
            }
          }
        }

        // ================== MAIN SIMULATION ==================
        if (sim.current.isRunning) {
          const idx = Math.floor(sim.current.index);
          const frac = sim.current.index - idx;

          // Get 4 points (for smooth curve)
          const p0 = traj[Math.max(idx - 1, 0)];
          const p1 = traj[idx];
          const p2 = traj[Math.min(idx + 1, traj.length - 1)];
          const p3 = traj[Math.min(idx + 2, traj.length - 1)];

          if (
            !p1 || !p2 ||
            !Number.isFinite(p0.x) || !Number.isFinite(p0.y) || !Number.isFinite(p0.z) ||
            !Number.isFinite(p1.x) || !Number.isFinite(p1.y) || !Number.isFinite(p1.z) ||
            !Number.isFinite(p2.x) || !Number.isFinite(p2.y) || !Number.isFinite(p2.z) ||
            !Number.isFinite(p3.x) || !Number.isFinite(p3.y) || !Number.isFinite(p3.z)
          ) {
            sim.current.isRunning = false;
            renderer.render(scene, camera);
            return;
          }

          // ================== CATMULL-ROM INTERPOLATION ==================
          function catmullRom(t, p0, p1, p2, p3) {
            return (
              0.5 *
              (2 * p1 +
                (-p0 + p2) * t +
                (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t +
                (-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t)
            );
          }

          const pos = evtol.position;

          pos.set(
            catmullRom(frac, p0.x, p1.x, p2.x, p3.x),
            catmullRom(frac, p0.y, p1.y, p2.y, p3.y),
            catmullRom(frac, p0.z, p1.z, p2.z, p3.z),
          );

          // ================== SMOOTH TANGENT ==================
          const nextT = Math.min(frac + 0.01, 1);

          const nextPos = new THREE.Vector3(
            catmullRom(nextT, p0.x, p1.x, p2.x, p3.x),
            catmullRom(nextT, p0.y, p1.y, p2.y, p3.y),
            catmullRom(nextT, p0.z, p1.z, p2.z, p3.z),
          );

          const tangent = nextPos.clone().sub(pos).normalize();
          if (!Number.isFinite(pos.x) || !Number.isFinite(pos.y) || !Number.isFinite(pos.z)) {
            sim.current.isRunning = false;
            renderer.render(scene, camera);
            return;
          }

          // ================== LANDING / PICKUP LOGIC ==================
          const distToPad = pos.distanceTo(landingPadPos);
          const dxPad = pos.x - landingPadPos.x;
          const dzPad = pos.z - landingPadPos.z;
          const distToPadXZ = Math.sqrt(dxPad * dxPad + dzPad * dzPad);
          const turnaroundIndex = findMissionTurnaroundIndex(trajectoryVectorsRef.current);
          const approachingPickupZone =
            !sim.current.personPicked &&
            (distToPadXZ <= 120 || Math.abs(idx - turnaroundIndex) <= 18);
          const shouldDescendForPickup =
            !sim.current.personPicked &&
            (distToPadXZ <= 260 || idx >= Math.max(0, turnaroundIndex - 35));

          if (approachingPickupZone) {
            pos.y = THREE.MathUtils.lerp(pos.y, 4, 0.1);
            sim.current.waitTimer += delta;
            sim.current.postPickupAltTimer = 0;

            if (sim.current.waitTimer > 2.5) {
              sim.current.personPicked = true;
              sim.current.index += sim.current.playbackSpeed * 20 * delta;
            }
          } else if (shouldDescendForPickup) {
            sim.current.waitTimer = 0;
            sim.current.postPickupAltTimer = 0;
            sim.current.index += sim.current.playbackSpeed * 50 * delta;

            // smooth descent toward landing altitude
            pos.y = THREE.MathUtils.lerp(pos.y, 4, 0.05);
          } else {
            sim.current.waitTimer = 0;
            // After boarding, blend smoothly from landing altitude back to trajectory altitude
            if (sim.current.personPicked && sim.current.postPickupAltTimer < 1) {
              sim.current.postPickupAltTimer = Math.min(sim.current.postPickupAltTimer + delta * 0.6, 1);
              const trajY = pos.y;
              pos.y = THREE.MathUtils.lerp(4, trajY, sim.current.postPickupAltTimer);
            }
            sim.current.index += sim.current.playbackSpeed * 100 * delta;
          }

          // Hover logic: if replanning is taking too long, stop advancing
          if (replanInFlightRef.current) {
            hoverTimerRef.current += delta;
            if (hoverTimerRef.current > REPLAN_TIMEOUT_S && !isHovering) {
              setIsHovering(true);
              setReplanStatus("HOVERING");
            }
          } else {
            if (isHovering) setIsHovering(false);
            hoverTimerRef.current = 0;
          }

          if (!isHovering) {
            sim.current.index = Math.min(sim.current.index, traj.length - 1);
          }

          const distToStart = pos.distanceTo(startPos);
          const completedReturn =
            sim.current.personPicked &&
            (sim.current.index >= traj.length - 2 || distToStart <= 80);

          if (completedReturn && !sim.current.pathCompleted) {
            sim.current.pathCompleted = true;
            sim.current.isRunning = false;
            autoCaptureRef.current = false;
            setReplanStatus("IDLE");
          }

          // ================== MISSION STATUS ==================
          const missionState = missionStateRef.current;

          // derive state from SAME conditions as flight logic
          if (completedReturn) {
            missionState.setStatus(MissionStatus.IDLE);
          }

          else if (distToPadXZ > 200 && !sim.current.personPicked) {
            missionState.setStatus(MissionStatus.ENROUTE);
          }

          else if (!approachingPickupZone && !sim.current.personPicked) {
            missionState.setStatus(MissionStatus.LANDING);
          }

          else if (approachingPickupZone && !sim.current.personPicked) {
            missionState.setStatus(MissionStatus.PICKING_UP);
          }

          else if (sim.current.personPicked) {
            missionState.setStatus(MissionStatus.RETURN);
          }

          // ===== DRAW PATH PROGRESS =====
          updatePathDrawRanges();

          // ================== ROTATION ==================
          if (tangent.lengthSq() > 0.0001) {
            const flatTangent = tangent.clone();
            flatTangent.y = 0;
            flatTangent.normalize();

            const lookTarget = pos.clone().add(flatTangent);

            const dummy = new THREE.Object3D();
            dummy.position.copy(pos);
            dummy.lookAt(lookTarget);

            // slightly faster rotation for smoother turning
            const turnSpeed = 1; // lower = slower turning

            const turnSharpness = Math.abs(tangent.x) + Math.abs(tangent.z);
            const adaptiveSpeed = THREE.MathUtils.clamp(
              turnSharpness,
              0.5,
              1.5,
            );

            const alpha = 1 - Math.exp(-(turnSpeed / adaptiveSpeed) * delta);

            evtol.quaternion.slerp(dummy.quaternion, alpha);
          }

          // ================== TELEMETRY ==================
          onPositionUpdate?.({
            x: pos.x,
            y: pos.y,
            z: pos.z,
            speed: 220,
            altitude: pos.y,
            heading: Math.atan2(tangent.x, tangent.z) * (180 / Math.PI),
          });

          // ================== PROACTIVE COLLISION LOOKAHEAD (every 0.5 s) ==================
          if (sim.current.isRunning && knownObstaclesRef.current.length > 0) {
            lastProactiveCheckRef.current += delta;
            if (lastProactiveCheckRef.current >= 0.5) {
              lastProactiveCheckRef.current = 0;

              {
                const currentIdx = Math.floor(sim.current.index);
                const traj = trajectoryVectorsRef.current;
                // Scan up to 3000 trajectory points ahead (step 20 = 150 checks).
                // Wide window ensures buildings far along the route are caught early.
                // detectedObstacleKeysRef gates re-triggering per obstacle.
                let collisionAhead = false;
                outer:
                for (let ahead = 20; ahead <= 3000; ahead += 20) {
                  const checkIdx = currentIdx + ahead;
                  if (checkIdx >= traj.length) break;
                  const pt = traj[checkIdx];
                  for (const obs of knownObstaclesRef.current.filter(o => !o.id?.startsWith('KNOWN-'))) {
                    const dx = pt.x - obs.x;
                    const dz = pt.z - obs.z;
                    // Birds: tight 3D sphere — don't over-react to a small animal.
                    // Buildings: scale-250 models extend ~400 m from centre visually.
                    const clearance = obs.label === "bird"
                      ? (obs.radius || 55) + 40
                      : (obs.radius || 155) + 300;
                    const hit = obs.label === "bird"
                      ? Math.sqrt(dx * dx + (pt.y - (obs.y || 0)) ** 2 + dz * dz) < clearance
                      : Math.sqrt(dx * dx + dz * dz) < clearance;
                    if (hit) {
                      const key = makeDetectionKey({ X_world: obs.x, Z_world: obs.z, radius: obs.radius });
                      // Only trigger for obstacles not yet handled; let
                      // applyInPlaceDetour manage the detectedObstacleKeysRef set
                      if (!detectedObstacleKeysRef.current.has(key)) {
                        collisionAhead = true;
                      }
                      break outer;
                    }
                  }
                }
                if (collisionAhead) proactiveReplanRef.current();
              }
            }
          }

          // ================== FRONTEND PROXIMITY DETECTION ==================
          // Detects surprise buildings directly from eVTOL position — no YOLO service needed.
          if (sim.current.isRunning) {
            const PROX_M = 1500; // obstacle at (2100,1200) is 1140m from corridor — needs >1140
            obstacleBuildingsRef.current.forEach(b => {
              if (obstacleClassMapRef.current[b.id] !== 'surprise') return;
              const ddx = pos.x - b.x;
              const ddz = pos.z - b.z;
              if (Math.sqrt(ddx * ddx + ddz * ddz) < PROX_M) {
                markSurpriseDetectedRef.current(b.id, sceneRef.current);
                // Let ingestPlannerObstacles own the knownObstaclesRef update so
                // changed=true and replanTrajectoryFromDetections is called.
                // Guard with coordinate proximity (one makeDetectionKey bucket = 120m).
                const alreadyKnown = knownObstaclesRef.current.some(k => {
                  const dx = k.x - b.x, dz = k.z - b.z;
                  return Math.sqrt(dx * dx + dz * dz) < 120;
                });
                if (!alreadyKnown) {
                  ingestPlannerObstaclesRef.current(
                    [{ X_world: b.x, Y_world: 0, Z_world: b.z,
                       radius: b.radius || 155, label: 'building', confidence: 0.95, source: 'proximity' }],
                    frameIndexRef.current,
                    'PROX'
                  );
                }
              }
            });
          }

          // ================== AUTO-CAPTURE + YOLO (10 fps) ==================
          if (autoCaptureRef.current && rendererRef.current) {
            lastCaptureRef.current += delta;
            if (lastCaptureRef.current >= CAPTURE_INTERVAL) {
              lastCaptureRef.current = 0;
              const currentIndex = frameIndexRef.current++;
              const dataUrl = rendererRef.current.domElement.toDataURL("image/png");
              const W = rendererRef.current.domElement.width;
              const H = rendererRef.current.domElement.height;

              // Push frame immediately (no obstacles yet)
              setScreenshots((prev) => [
                ...prev,
                { dataUrl, frameIndex: currentIndex, obstacles: [], annotatedImg: null },
              ]);

              // Send to YOLO service async
              const evtolPos = evtolRef.current?.position;
              const evtolQuat = evtolRef.current?.quaternion;
              if (evtolPos) {
                if (cameraModeRef.current === "WORLD_FIXED") {
                  const exactObstacles = getExactVisibleObstacles(camera);

                  setLatestObstacles(exactObstacles);

                  if (exactObstacles.length > 0) {
                    setObstacleHistory((prev) => [
                      ...prev,
                      {
                        frameIndex: currentIndex,
                        obstacles: exactObstacles,
                        evtol: { x: evtolPos.x, y: evtolPos.y, z: evtolPos.z },
                      },
                    ]);
                  }

                  setScreenshots((prev) =>
                    prev.map((s) =>
                      s.frameIndex === currentIndex
                        ? { ...s, obstacles: exactObstacles, annotatedImg: null }
                        : s
                    )
                  );
                  ingestPlannerObstaclesRef.current(exactObstacles, currentIndex, "WORLD");
                } else {
                fetch(YOLO_URL, {
                  method: "POST",
                  headers: { "Content-Type": "application/json" },
                  body: JSON.stringify({
                    frame: dataUrl,
                    frameIndex: currentIndex,
                    theme: theme,
                    evtol: {
                      x: evtolPos.x, y: evtolPos.y, z: evtolPos.z,
                      qx: evtolQuat ? evtolQuat.x : 0,
                      qy: evtolQuat ? evtolQuat.y : 0,
                      qz: evtolQuat ? evtolQuat.z : 0,
                      qw: evtolQuat ? evtolQuat.w : 1,
                      heading: Math.atan2(tangent.x, tangent.z) * (180 / Math.PI),
                    },
                    camera: { fov: camera.fov, width: W, height: H },
                  }),
                })
                  .then((r) => r.json())
                    .then((result) => {
                      const obs = result.obstacles || [];
                      const plannerObs = result.plannerObstacles || [];
                      const annotated = result.annotatedImg || null;

                      // DEBUG: Log detection results
                      console.log(`[YOLO] Frame ${currentIndex}: ${obs.length} obstacles detected`, obs);
                      if (result.validation) {
                        console.log(`[YOLO] Validation metrics:`, result.validation);
                      }

                      setLatestObstacles(plannerObs);
                      ingestPlannerObstaclesRef.current(plannerObs, currentIndex, "YOLO");

                      // ── Add newly-confirmed YOLO obstacles to the known set ──────────
                      const validObs = plannerObs.filter(
                        o => Number.isFinite(o.X_world) && Number.isFinite(o.Z_world));
                      if (validObs.length > 0) {
                        const updated = [...knownObstaclesRef.current];
                        let changed = false;
                        for (const o of validObs) {
                          const key = makeDetectionKey(
                            { X_world: o.X_world, Z_world: o.Z_world, radius: o.radius || 155 });
                          const exists = updated.some(k =>
                            makeDetectionKey({ X_world: k.x, Z_world: k.z, radius: k.radius }) === key);
                          if (!exists) {
                            updated.push({
                              id: `YOLO-${Math.round(o.X_world)}-${Math.round(o.Z_world)}`,
                              label: o.label || "structure",
                              x: o.X_world, y: o.Y_world || 0, z: o.Z_world,
                              radius: o.radius || 155,
                            });
                            changed = true;
                          }
                        }
                        if (changed) {
                          knownObstaclesRef.current = updated;
                          console.log(`[KnownObs] Updated: ${updated.length} total`);
                        }
                      }

                      // Replan is already handled by ingestPlannerObstacles.

                      if (plannerObs.length > 0) {
                        setObstacleHistory((prev) => [
                        ...prev,
                        {
                          frameIndex: currentIndex,
                          obstacles: plannerObs,
                          evtol: { x: evtolPos.x, y: evtolPos.y, z: evtolPos.z },
                        },
                      ]);
                    }

                    // Patch the screenshot entry with YOLO results
                    setScreenshots((prev) =>
                      prev.map((s) =>
                        s.frameIndex === currentIndex
                          ? { ...s, obstacles: plannerObs, annotatedImg: annotated }
                          : s
                      )
                    );
                  })
                  .catch((err) => { 
                    console.error(`[YOLO] Error detecting obstacles:`, err);
                    console.warn(`[YOLO] Service may not be running at ${YOLO_URL}`);
                  });
                }
              }
            }
          }
        }
        // ================== CAMERA ==================
        if (camera && controls && evtol) {
          const pos = evtol.position;

          if (cameraModeRef.current === "FPV" && fpvRef.current) {
            const worldPos = new THREE.Vector3();
            const worldQuat = new THREE.Quaternion();

            fpvRef.current.getWorldPosition(worldPos);
            fpvRef.current.getWorldQuaternion(worldQuat);

            camera.position.copy(worldPos);
            camera.quaternion.copy(worldQuat);

            // small forward look (stabilizes view)
            const forward = new THREE.Vector3(0, 0, 1)
              .applyQuaternion(worldQuat)
              .multiplyScalar(50);

            controls.target.copy(worldPos.clone().add(forward));

            controls.enableRotate = false;
            controls.enableZoom = false;
            controls.enablePan = false;
          } else if (cameraModeRef.current === "CHASE") {
            const back = new THREE.Vector3(0, 0, -1)
              .applyQuaternion(evtol.quaternion)
              .normalize()
              .multiplyScalar(350);

            camera.position.copy(
              pos
                .clone()
                .add(back)
                .add(new THREE.Vector3(0, 190, 0)),
            );

            controls.target.copy(pos);

            controls.enableRotate = false;
            controls.enableZoom = false;
            controls.enablePan = false;
          } else if (cameraModeRef.current === "HFOV" && fpvRef.current) {
            const worldPos = new THREE.Vector3();
            const worldQuat = new THREE.Quaternion();

            fpvRef.current.getWorldPosition(worldPos);
            fpvRef.current.getWorldQuaternion(worldQuat);

            camera.position.copy(worldPos);
            camera.quaternion.copy(worldQuat);

            // small forward look (stabilizes view)
            const forward = new THREE.Vector3(0, 0, 1)
              .applyQuaternion(worldQuat)
              .multiplyScalar(50);

            controls.target.copy(worldPos.clone().add(forward));

            controls.enableRotate = false;
            controls.enableZoom = true;
            controls.enablePan = false;
          } else if (cameraModeRef.current === "WORLD_FIXED") {
            const { position, target } = worldFixedConfigRef.current;

            if (position && target) {
              camera.position.copy(position);
              camera.lookAt(target);
              controls.target.copy(target);
            }

            controls.enableRotate = false;
            controls.enableZoom = false;
            controls.enablePan = false;
          } else {
            controls.enableRotate = true;
            controls.enableZoom = true;
            controls.enablePan = true;
          }
        }

        controls.update();

        updatePathDrawRanges();
        // ================== DOWNWARD CAMERA RENDER ==================
        if (downCamRef.current && downCamRendererRef.current && evtolRef.current) {
          downCamRendererRef.current.render(scene, downCamRef.current);

          // Copy to the visible canvas
          if (downCamCanvasRef.current) {
            const ctx = downCamCanvasRef.current.getContext("2d");
            if (ctx) {
              ctx.drawImage(downCamRendererRef.current.domElement, 0, 0, 360, 200);

              // Draw crosshair overlay
              ctx.strokeStyle = "rgba(255,60,60,0.85)";
              ctx.lineWidth = 1.5;
              ctx.beginPath(); ctx.moveTo(180, 90); ctx.lineTo(180, 110); ctx.stroke();
              ctx.beginPath(); ctx.moveTo(170, 100); ctx.lineTo(190, 100); ctx.stroke();

              // Height annotation
              const h = evtolRef.current.position.y;
              ctx.fillStyle = "rgba(0,0,0,0.5)";
              ctx.fillRect(4, 4, 120, 16);
              ctx.fillStyle = "#4ade80";
              ctx.font = "10px monospace";
              ctx.fillText(`ALT: ${h.toFixed(1)} m`, 8, 16);

              // Compute apparent radius of the vertiport circle at current altitude
              // Using pinhole camera projection:
              //   apparentRadius_px = (VERTIPAD_REAL_RADIUS * focalLength_px) / height
              //   focalLength_px  = (canvasHeight / 2) / tan(FOV/2)
              const fovRad = (DOWN_CAM_FOV * Math.PI) / 180;
              const focalLength = (200 / 2) / Math.tan(fovRad / 2);
              const apparentR = h > 1 ? (VERTIPAD_REAL_RADIUS * focalLength) / h : 0;

              ctx.fillStyle = "rgba(0,0,0,0.5)";
              ctx.fillRect(4, 22, 140, 16);
              ctx.fillStyle = "#fbbf24";
              ctx.fillText(`R_apparent: ${apparentR.toFixed(1)} px`, 8, 34);

              // Determine phase
              const distToPadNow = evtolRef.current.position.distanceTo(landingPadPos);
              const phase = sim.current.personPicked ? "RETURN"
                : distToPadNow < 400 ? "LANDING"
                  : "TAKEOFF";

              // Draw estimated circle overlay on camera feed
              if (apparentR > 2 && apparentR < 180) {
                ctx.strokeStyle = "rgba(250,204,21,0.7)";
                ctx.lineWidth = 1;
                ctx.setLineDash([4, 4]);
                ctx.beginPath();
                ctx.arc(180, 100, Math.min(apparentR, 95), 0, Math.PI * 2);
                ctx.stroke();
                ctx.setLineDash([]);
              }

              // Log height-radius data at defined intervals
              if (autoCaptureRef.current && h > 2) {
                const roundedH = Math.round(h / LOG_HEIGHT_STEP) * LOG_HEIGHT_STEP;
                if (lastLogHeightRef.current !== roundedH) {
                  lastLogHeightRef.current = roundedH;
                  setHeightRadiusLog((prev) => {
                    // avoid duplicate at same height+phase
                    const exists = prev.some(
                      (r) => Math.abs(r.height - h) < LOG_HEIGHT_STEP / 2 && r.phase === phase
                    );
                    if (exists) return prev;
                    return [
                      ...prev,
                      {
                        height: parseFloat(h.toFixed(1)),
                        apparentRadius: parseFloat(apparentR.toFixed(2)),
                        ratio: parseFloat((apparentR / h).toFixed(4)),
                        phase,
                      },
                    ];
                  });
                }
              }
            }
          }
        }

        // Pulse detection rings
        const rings = Object.values(detectionRingMapRef.current);
        if (rings.length > 0) {
          const pulse = 0.55 + 0.45 * Math.sin(Date.now() * 0.004);
          rings.forEach(r => { if (r?.material) r.material.opacity = pulse; });
        }

        renderer.render(scene, camera);
      }


      animate();

      return () => {
        cancelAnimationFrame(animationId);

        if (controlsRef.current) controlsRef.current.dispose();
        if (rendererRef.current) rendererRef.current.dispose();
        if (downCamRendererRef.current) downCamRendererRef.current.dispose();

        if (container && renderer.domElement) {
          container.removeChild(renderer.domElement);
        }
      };
    }, [missionMode]);

    // Keep cameraModeRef in sync and reset camera to overview position when switching to OVERVIEW
    useEffect(() => {
      const prev = cameraModeRef.current;
      cameraModeRef.current = cameraMode;

      if (cameraMode === "OVERVIEW" && prev !== "OVERVIEW") {
        const camera = perspectiveCameraRef.current;
        const controls = controlsRef.current;
        if (camera && controls) {
          camera.position.set(
            stationCenter.x + 2000,
            1200,
            stationCenter.z + 2500,
          );
          controls.target.set(stationCenter.x, 0, stationCenter.z);
          controls.update();
        }
      }
    }, [cameraMode]);

    useEffect(() => {
      if (
        !rendererRef.current ||
        !perspectiveCameraRef.current ||
        !containerRef.current
      )
        return;


      setTimeout(() => {
        const width = containerRef.current.clientWidth;
        const height = containerRef.current.clientHeight;

        rendererRef.current.setSize(width, height);

        perspectiveCameraRef.current.aspect = width / height;
        perspectiveCameraRef.current.updateProjectionMatrix();
      }, 260); // 
    }, [showPanel]);

    // ===== THEME UPDATES =====
    useEffect(() => {
      if (!sceneRef.current || !ambientLightRef.current || !directionalLightRef.current || !hemisphereLightRef.current) {
        return;
      }

      const themeConfig = getTheme(theme);

      // Update scene background
      sceneRef.current.background = new THREE.Color(themeConfig.sceneBackground);

      // Update fog
      sceneRef.current.fog = new THREE.Fog(
        themeConfig.fog.color,
        themeConfig.fog.near,
        themeConfig.fog.far
      );

      // Update ambient light
      ambientLightRef.current.color.setHex(themeConfig.ambientLight.color);
      ambientLightRef.current.intensity = themeConfig.ambientLight.intensity;

      // Update directional light
      directionalLightRef.current.color.setHex(themeConfig.directionalLight.color);
      directionalLightRef.current.intensity = themeConfig.directionalLight.intensity;
      directionalLightRef.current.position.set(
        themeConfig.directionalLight.position[0],
        themeConfig.directionalLight.position[1],
        themeConfig.directionalLight.position[2]
      );

      // Update hemisphere light
      hemisphereLightRef.current.color.setHex(themeConfig.hemisphereLight.skyColor);
      hemisphereLightRef.current.groundColor.setHex(themeConfig.hemisphereLight.groundColor);
      hemisphereLightRef.current.intensity = themeConfig.hemisphereLight.intensity;
    }, [theme]);

    return (
      <div
        style={{
          width: "100%",
          height: "100%",
          display: "flex",
          flexDirection: "column",
          position: "relative",
        }}
      >
        {/* 3D Scene */}
        <div
          ref={containerRef}
          style={{
            width: "100%",
            height: showPanel ? "80%" : "100%", // FIXED LOGIC
            transition: "height 0.25s ease",
          }}
        />

        {/* ── FRAME CAPTURE + OBSTACLE PANEL ────────────────────────────── */}

        {/* ── REAL-TIME VERIFICATION PANEL (top-left) ────────────────────── */}
        <div style={{
          position: "absolute", top: "12px", left: "16px", zIndex: 8,
          background: "rgba(8, 14, 30, 0.92)",
          border: "1px solid #1e3a5f",
          borderRadius: "10px", padding: "10px 14px", color: "white",
          fontFamily: "monospace", fontSize: "11px", minWidth: "220px",
          boxShadow: "0 0 16px rgba(0,100,255,0.15)",
        }}>
          <div style={{ color: "#60a5fa", marginBottom: 6, fontWeight: "bold", fontSize: "12px", letterSpacing: "0.05em" }}>
            eVTOL OBSTACLE VERIFICATION
          </div>

          {/* Scene total */}
          <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 3 }}>
            <span style={{ color: "#94a3b8" }}>Scene Total</span>
            <span style={{ color: "#e2e8f0", fontWeight: "bold" }}>
              {backendKnownCount + surpriseObstacles.length}
            </span>
          </div>

          {/* Backend-known (static) */}
          <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 3 }}>
            <span style={{ color: "#94a3b8" }}>
              <span style={{ display: "inline-block", width: 8, height: 8, borderRadius: 2, background: "#e2e8f0", marginRight: 5, verticalAlign: "middle" }} />
              Backend-Known
            </span>
            <span style={{ color: "#4ade80", fontWeight: "bold" }}>
              {backendKnownCount} ✓
            </span>
          </div>

          {/* Surprise obstacles */}
          <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 3 }}>
            <span style={{ color: "#94a3b8" }}>
              <span style={{ display: "inline-block", width: 8, height: 8, borderRadius: 2, background: "#ff6600", marginRight: 5, verticalAlign: "middle" }} />
              Surprise (Uncharted)
            </span>
            <span style={{ color: surpriseObstacles.length > 0 ? "#fb923c" : "#94a3b8", fontWeight: "bold" }}>
              {surpriseObstacles.length} {surpriseObstacles.length > 0 ? "⚠" : ""}
            </span>
          </div>

          {/* YOLO detections */}
          <div style={{
            display: "flex", justifyContent: "space-between", marginBottom: 4,
            borderTop: "1px solid #1e3a5f", paddingTop: 4, marginTop: 4,
          }}>
            <span style={{ color: "#94a3b8" }}>
              <span style={{ display: "inline-block", width: 8, height: 8, borderRadius: "50%", background: detectedSurpriseIds.size > 0 ? "#ff2200" : "#334155", marginRight: 5, verticalAlign: "middle" }} />
              YOLO Detected
            </span>
            <span style={{
              color: detectedSurpriseIds.size >= surpriseObstacles.length && surpriseObstacles.length > 0
                ? "#4ade80" : detectedSurpriseIds.size > 0 ? "#f87171" : "#94a3b8",
              fontWeight: "bold",
            }}>
              {detectedSurpriseIds.size}/{surpriseObstacles.length}
            </span>
          </div>

          {/* Replan status */}
          <div style={{
            padding: "3px 6px", borderRadius: 4, textAlign: "center", fontSize: "11px",
            background:
              replanStatus.includes("REPLANNING") ? "rgba(251,191,36,0.2)" :
              replanStatus.includes("UPDATED")    ? "rgba(74,222,128,0.2)" :
              replanStatus.includes("FAILED")     ? "rgba(248,113,113,0.2)" :
                                                    "rgba(147,197,253,0.1)",
            color:
              replanStatus.includes("REPLANNING") ? "#fbbf24" :
              replanStatus.includes("UPDATED")    ? "#4ade80" :
              replanStatus.includes("FAILED")     ? "#f87171" :
                                                    "#93c5fd",
            border: "1px solid",
            borderColor:
              replanStatus.includes("REPLANNING") ? "#fbbf24" :
              replanStatus.includes("UPDATED")    ? "#4ade80" :
              replanStatus.includes("FAILED")     ? "#f87171" :
                                                    "#1e3a5f",
          }}>
            {replanStatus.includes("REPLANNING") ? "⟳ REPLANNING TRAJECTORY" :
             replanStatus.includes("UPDATED")    ? "✓ TRAJECTORY UPDATED" :
             replanStatus.includes("FAILED")     ? "✕ REPLAN FAILED" :
                                                   `▶ ${replanStatus}`}
          </div>

          {/* Frames + WS status */}
          <div style={{ marginTop: 5, color: "#475569", fontSize: "10px", display: "flex", justifyContent: "space-between" }}>
            <span>Frames: {screenshots.length} | Detections: {obstacleHistory.length}</span>
            <span style={{ color: wsConnected ? "#4ade80" : "#f87171", fontWeight: "bold" }}>
              {wsConnected ? "WS ●" : "WS ○"}
            </span>
          </div>
          {isHovering && <div style={{ color: "#f97316", fontSize: "10px", marginTop: 2 }}>⚠ HOVERING</div>}
        </div>

        {/* Legend */}
        <div style={{
          position: "absolute", top: "12px", right: "16px", zIndex: 8,
          background: "rgba(8,14,30,0.88)", border: "1px solid #1e3a5f",
          borderRadius: "8px", padding: "8px 12px", fontFamily: "monospace",
          fontSize: "10px", color: "#94a3b8",
        }}>
          <div style={{ color: "#60a5fa", fontWeight: "bold", marginBottom: 4 }}>BUILDING LEGEND</div>
          <div style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 3 }}>
            <span style={{ display: "inline-block", width: 18, height: 3, background: "#00ff88", borderRadius: 2 }} />
            Forward path
          </div>
          <div style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 3 }}>
            <span style={{ display: "inline-block", width: 18, height: 3, background: "#3b82ff", borderRadius: 2 }} />
            Return path
          </div>
          <div style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 3 }}>
            <span style={{ display: "inline-block", width: 12, height: 12, background: "#d1d5db", borderRadius: 2 }} />
            Backend-Known (static)
          </div>
          <div style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 3 }}>
            <span style={{ display: "inline-block", width: 12, height: 12, background: "#ff6600", borderRadius: 2 }} />
            Surprise (uncharted)
          </div>
          <div style={{ display: "flex", alignItems: "center", gap: 6 }}>
            <span style={{ display: "inline-block", width: 12, height: 12, borderRadius: "50%", border: "2px solid #ff2200", background: "transparent" }} />
            YOLO Detected
          </div>
          {cameraMode !== "WORLD_FIXED" && (
            <div style={{
              marginTop: 6, paddingTop: 5, borderTop: "1px solid #1e3a5f",
              color: wsConnected ? "#4ade80" : "#f87171",
              fontWeight: "bold",
            }}>
              WS: {wsConnected ? "● LIVE" : "○ OFFLINE"}
            </div>
          )}
        </div>

        {showPanel && (
          <div style={{
            height: "22%", minHeight: "140px", background: "#0a0f1e",
            borderTop: "2px solid #1e293b", display: "flex",
            flexDirection: "column", fontFamily: "monospace"
          }}>

            {/* Header */}
            <div style={{
              padding: "4px 10px", color: "white", fontSize: "12px",
              borderBottom: "1px solid #1e293b", display: "flex",
              justifyContent: "space-between", alignItems: "center",
              background: "#0f172a"
            }}>
              <span style={{ color: "#94a3b8" }}>
                📸 <strong style={{ color: "#e2e8f0" }}>{screenshots.length}</strong> frames
                {obstacleHistory.length > 0 && (
                  <span style={{ color: "#f87171", marginLeft: 10 }}>
                    ⚠ <strong>{obstacleHistory.length}</strong> with obstacles
                  </span>
                )}
                {autoCaptureRef.current && (
                  <span style={{ color: "#4ade80", marginLeft: 10 }}>● REC 10fps</span>
                )}
              </span>

              <div style={{ display: "flex", gap: "6px", alignItems: "center" }}>
                {screenshots.length > 0 && (
                  <button onClick={() => screenshots.forEach((s) => {
                    const a = document.createElement("a");
                    a.href = s.annotatedImg || s.dataUrl;
                    a.download = `evtol_frame_${String(s.frameIndex).padStart(6, "0")}.png`;
                    a.click();
                  })} style={panelBtn("#2563eb")} title="Download all">⬇ All</button>
                )}
                {screenshots.length > 0 && (
                  <button onClick={() => { setScreenshots([]); setLatestObstacles([]); setObstacleHistory([]); frameIndexRef.current = 0; }}
                    style={panelBtn("#475569")} title="Clear all">🗑</button>
                )}
                <span onClick={() => setShowPanel(false)} style={{ cursor: "pointer", color: "#94a3b8", fontSize: "16px" }}>⬇</span>
              </div>
            </div>

            {/* Body: frame strip + obstacle log */}
            <div style={{ flex: 1, display: "flex", overflow: "hidden" }}>

              {/* Frame strip */}
              <div style={{
                flex: 1, display: "flex", gap: "5px", padding: "6px",
                overflowX: "auto", overflowY: "hidden", alignItems: "center"
              }}>
                {screenshots.map((s) => (
                  <div key={s.frameIndex} style={{ position: "relative", height: "100%", flexShrink: 0 }}>
                    <img
                      src={s.annotatedImg || s.dataUrl}
                      alt={`frame-${s.frameIndex}`}
                      onClick={() => setSelectedImage(s)}
                      style={{
                        height: "100%", aspectRatio: "16/9", objectFit: "cover", cursor: "pointer",
                        border: s.obstacles.length > 0 ? "2px solid #ef4444" : "1px solid #334155",
                        borderRadius: "4px", display: "block"
                      }}
                    />
                    <span style={badge("bottom", "left")}>#{s.frameIndex}</span>
                    {s.obstacles.length > 0 && (
                      <span style={{ ...badge("top", "left"), background: "rgba(239,68,68,0.85)" }}>
                        ⚠{s.obstacles.length}
                      </span>
                    )}
                    <a href={s.annotatedImg || s.dataUrl}
                      download={`evtol_frame_${String(s.frameIndex).padStart(6, "0")}.png`}
                      onClick={(e) => e.stopPropagation()}
                      style={{ ...badge("top", "right"), textDecoration: "none", color: "white" }}>⬇</a>
                  </div>
                ))}
              </div>

              {/* Obstacle log sidebar */}
              <div style={{
                width: "250px", borderLeft: "1px solid #1e293b", overflowY: "auto",
                padding: "4px 6px", background: "#06090f", fontSize: "10px", color: "#94a3b8"
              }}>
                <div style={{ color: "#f87171", fontWeight: "bold", marginBottom: 4, fontSize: "11px" }}>⚠ OBSTACLE LOG</div>

                {latestObstacles.length > 0 && (
                  <div style={{
                    marginBottom: 6, padding: 4, background: "#1a0000",
                    borderRadius: 4, border: "1px solid #7f1d1d"
                  }}>
                    <div style={{ color: "#fca5a5", fontWeight: "bold", fontSize: "10px" }}>LATEST FRAME</div>
                    {latestObstacles.map((o, i) => (
                      <div key={i} style={{ marginTop: 3, color: "#fef2f2" }}>
                        <span style={{ color: "#f87171" }}>▶ {o.id || o.label}</span> ({(o.confidence * 100).toFixed(0)}%)<br />
                        <span style={{ color: "#94a3b8" }}>
                          W:({o.X_world},{o.Y_world},{o.Z_world})<br />
                          {o.u !== undefined && <>Px:({o.u},{o.v})<br /></>}
                          D:{o.distance}m R:{o.radius}m
                        </span>
                      </div>
                    ))}
                  </div>
                )}

                {obstacleHistory.length === 0 && latestObstacles.length === 0 && (
                  <div style={{ color: "#334155", marginTop: 8 }}>No obstacles detected yet</div>
                )}

                {obstacleHistory.slice().reverse().map((entry, i) => (
                  <div key={i} style={{
                    marginBottom: 5, padding: "3px 5px", background: "#0f172a",
                    borderRadius: 3, borderLeft: "2px solid #ef4444"
                  }}>
                    <div style={{ color: "#fbbf24" }}>Frame #{entry.frameIndex}</div>
                    <div style={{ color: "#64748b", fontSize: "9px" }}>
                      eVTOL:({entry.evtol.x.toFixed(0)},{entry.evtol.y.toFixed(0)},{entry.evtol.z.toFixed(0)})
                    </div>
                    {entry.obstacles.map((o, j) => (
                      <div key={j} style={{ color: "#fca5a5", marginTop: 1 }}>
                        {(o.id || o.label)} — W({o.X_world},{o.Y_world},{o.Z_world})
                        {o.u !== undefined && ` Px(${o.u},${o.v})`} D:{o.distance}m
                      </div>
                    ))}
                  </div>
                ))}
              </div>
            </div>
          </div>
        )}

        {/* Floating Up Arrow */}
        {!showPanel && (
          <div
            onClick={() => setShowPanel(true)}
            style={{
              position: "absolute",
              bottom: "10px",
              left: "50%",
              transform: "translateX(-50%)",
              background: "#0f172a",
              color: "white",
              padding: "6px 12px",
              borderRadius: "6px",
              cursor: "pointer",
              border: "1px solid #334155",
              zIndex: 10,
            }}
          >
            ⬆
          </div>
        )}

        {/* VERTIPORT DATA PANEL */}
        <VertiportDataPanel
          heightRadiusLog={heightRadiusLog}
          downCamCanvasRef={downCamCanvasRef}
        />

        {/* FULLSCREEN VIEWER */}
        {selectedImage && (
          <div style={{
            position: "fixed", top: 0, left: 0, width: "100vw", height: "100vh",
            background: "rgba(0,0,0,0.92)", display: "flex", alignItems: "center",
            justifyContent: "center", zIndex: 9999, flexDirection: "column", gap: 12
          }}>
            <div onClick={() => setSelectedImage(null)}
              style={{
                position: "absolute", top: 20, right: 30, fontSize: 22, color: "red",
                cursor: "pointer", fontWeight: "bold"
              }}>Close</div>

            <img src={
              visionMode === "annotated"
                ? (selectedImage.annotatedImg || selectedImage.dataUrl)
                : (visionPreviewUrl || selectedImage.dataUrl)
            } alt="full"
              style={{ maxWidth: "80%", maxHeight: "75%", border: "3px solid white" }} />

            <div style={{ display: "flex", gap: 8, flexWrap: "wrap", justifyContent: "center" }}>
              <button onClick={() => { setVisionMode("original"); setVisionPreviewUrl(null); }} style={panelBtn("#334155")}>Original</button>
              <button onClick={() => setVisionMode("annotated")} style={panelBtn("#2563eb")}>Annotated</button>
              <button onClick={() => runVision("edge", theme)} style={panelBtn("#7c3aed")}>Edge</button>
              <button onClick={() => runVision("threshold", theme)} style={panelBtn("#059669")}>Threshold</button>
              <button onClick={() => runVision("bg-reveal", theme)} style={panelBtn("#f97316")}>BG Reveal</button>
              <button onClick={() => runVision("remove-bg")} style={panelBtn("#dc2626")}>BG Removal</button>
            </div>

            {selectedImage.obstacles?.length > 0 ? (
              <div style={{
                background: "#0f172a", border: "1px solid #ef4444", borderRadius: 8,
                padding: "10px 16px", color: "white", fontFamily: "monospace",
                fontSize: 12, maxWidth: "80%", width: "100%"
              }}>
                <div style={{ color: "#f87171", fontWeight: "bold", marginBottom: 6 }}>
                  ⚠ {selectedImage.obstacles.length} OBSTACLE(S) — Frame #{selectedImage.frameIndex}
                </div>
                <div style={{ display: "flex", gap: 16, flexWrap: "wrap" }}>
                  {selectedImage.obstacles.map((o, i) => (
                    <div key={i} style={{
                      background: "#1a0000", padding: "6px 10px",
                      borderRadius: 6, border: "1px solid #7f1d1d"
                    }}>
                      <div style={{ color: "#f87171" }}>{o.id || o.label} ({(o.confidence * 100).toFixed(0)}%)</div>
                      <div style={{ color: "#fca5a5" }}>
                        World: ({o.X_world}, {o.Y_world}, {o.Z_world})<br />
                        {o.X_camera !== undefined && <>Camera: ({o.X_camera}, {o.Y_camera}, {o.Z_camera})<br /></>}
                        {o.u !== undefined && <>Pixel: ({o.u}, {o.v})<br /></>}
                        Distance: {o.distance}m | Radius: {o.radius}m
                      </div>
                    </div>
                  ))}
                </div>
              </div>
            ) : (
              <div style={{ color: "#4ade80", fontFamily: "monospace", fontSize: 12 }}>
                ✓ No obstacles in this frame
              </div>
            )}
          </div>
        )}
      </div>
    );
  },
);

// ── Panel helper styles ───────────────────────────────────────────────────────
const panelBtn = (bg) => ({
  background: bg, border: "none", borderRadius: "4px", color: "white",
  fontSize: "11px", padding: "3px 8px", cursor: "pointer",
  fontFamily: "monospace", whiteSpace: "nowrap",
});

const badge = (vSide, hSide) => ({
  position: "absolute", [vSide]: "2px", [hSide]: "3px",
  background: "rgba(0,0,0,0.65)", color: "white", fontSize: "9px",
  padding: "1px 3px", borderRadius: "2px", pointerEvents: "none",
});

export default UnifiedVisualizationScene;
