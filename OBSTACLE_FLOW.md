# Obstacle Data Flow

This project now uses one clear obstacle workflow for real-time testing.

## 1. Frontend environment obstacles

File:
`frontend/public/data/obstacles.csv`

Purpose:
- Drives the 3D buildings shown in the environment
- Acts as the ground-truth list for validation
- You can add coordinates here manually and a building will appear in the scene

Important:
- This full file is **not** the backend's live planner file
- It is used to place buildings in the 3D environment

## 2. Imageprocessing obstacle feed

File:
`imageprocessing/data/obstacles.csv`

Purpose:
- Acts as the backend's external obstacle feed
- Mirrors the full obstacle world for validation and demo bootstrap
- Represents the file the backend "gets from imageprocessing"

Important:
- The backend now reads this file instead of reading the frontend CSV directly

## 3. Backend seed obstacles

File:
`backend/data/seed_obstacles.csv`

Purpose:
- Contains only a small starter set of known obstacles
- Used by the backend trajectory planner at startup

Current rule:
- Keep only 2-3 trusted obstacles here
- Everything else should remain unknown until YOLO detects it or a demo reveal endpoint adds it

## 4. Backend detected obstacles

File:
`backend/data/detected_obstacles.csv`

Purpose:
- Filled at runtime from YOLO detections
- Merged with the seed file for backend obstacle snapshots and validation

Important:
- This file should start empty except for the CSV header

## 5. Validation

Ground truth / imageprocessing feed:
- `imageprocessing/data/obstacles.csv`

Detected:
- `backend/data/detected_obstacles.csv`

Goal:
- Compare runtime detections against the imageprocessing obstacle feed

## Static + surprise obstacle verification

Current demo dataset:
- Full environment obstacles: 7
- Backend seed obstacles: 3
- Surprise obstacles hidden from backend at startup: 4

This lets you verify two things:
- Static avoidance: backend already avoids the first 3 seed obstacles
- Surprise avoidance: the route changes when later obstacles are injected into detected obstacles

Helpful demo endpoints:
- `POST /api/obstacles/demo/bootstrap-static?seedCount=3`
  Resets backend to a static-known scenario using only the first 3 imageprocessing obstacles
- `POST /api/obstacles/demo/reveal-surprises?count=1`
  Moves the next unseen imageprocessing obstacle into `backend/data/detected_obstacles.csv`

## Intended real-time workflow

1. Add all real buildings to `frontend/public/data/obstacles.csv`
2. Export the same obstacle world to `imageprocessing/data/obstacles.csv`
3. Keep only 2-3 starter obstacles in `backend/data/seed_obstacles.csv`
4. Start the frontend, backend, and YOLO service
5. Let YOLO discover the remaining buildings during flight
6. Store detections in `backend/data/detected_obstacles.csv`
7. Replan the path when detected obstacles appear
8. Validate detected obstacles against the imageprocessing obstacle feed

## Removed confusion

The duplicate runtime file at repo root:
`data/detected_obstacles.csv`

has been removed so the project uses only:
`backend/data/detected_obstacles.csv`
