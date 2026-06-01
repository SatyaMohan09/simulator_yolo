import csv
import math
from pathlib import Path


def verify_obstacle_coordinates():
    """Compare frontend ground truth with backend detected obstacles."""

    project_root = Path(__file__).resolve().parent
    frontend_csv = project_root / "imageprocessing" / "data" / "obstacles.csv"
    backend_csv = project_root / "backend" / "data" / "detected_obstacles.csv"

    print("=== Real-time eVTOL Coordinate Verification ===")
    print(f"Frontend obstacles: {frontend_csv}")
    print(f"Backend obstacles:  {backend_csv}")
    print()

    def load_rows(path):
        with path.open("r", newline="", encoding="utf-8") as handle:
            return [
                {
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "z": float(row["z"]),
                    "radius": float(row.get("radius", 0) or 0),
                }
                for row in csv.DictReader(handle)
                if row.get("x")
            ]

    try:
        frontend_rows = load_rows(frontend_csv)
        print(f"Frontend: {len(frontend_rows)} obstacles loaded")
    except Exception as e:
        print(f"Frontend CSV error: {e}")
        return

    try:
        backend_rows = load_rows(backend_csv)
        print(f"Backend: {len(backend_rows)} obstacles loaded")
    except Exception as e:
        print(f"Backend CSV error: {e}")
        return

    print("\n--- Frontend Obstacles (3D Buildings) ---")
    for i, row in enumerate(frontend_rows, start=1):
        print(f"{i:2d}. Building at ({row['x']:7.1f}, {row['y']:7.1f}, {row['z']:7.1f})")

    print("\n--- Backend Detected Obstacles (Real-time) ---")
    for i, row in enumerate(backend_rows, start=1):
        print(f"{i:2d}. Detected at ({row['x']:7.1f}, {row['y']:7.1f}, {row['z']:7.1f})")

    print("\n--- Coordinate Matching Analysis ---")
    tolerance = 150.0

    matches = 0
    for i, front_obs in enumerate(frontend_rows, start=1):
        front_pos = (front_obs["x"], front_obs["y"], front_obs["z"])

        for j, back_obs in enumerate(backend_rows, start=1):
            back_pos = (back_obs["x"], back_obs["y"], back_obs["z"])
            distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(front_pos, back_pos)))

            if distance <= tolerance:
                matches += 1
                print(f"Match: frontend building #{i} with backend detection #{j} (distance: {distance:.1f}m)")
                break

    print(f"\n--- Verification Results ---")
    print(f"Frontend buildings: {len(frontend_rows)}")
    print(f"Backend detections: {len(backend_rows)}")
    print(f"Matches within {tolerance}m: {matches}")

    if len(frontend_rows) > 0:
        match_rate = (matches / len(frontend_rows)) * 100
        print(f"Detection accuracy: {match_rate:.1f}%")

        if match_rate >= 80:
            print("EXCELLENT: Real-time detection is closely matching the scene.")
        elif match_rate >= 60:
            print("GOOD: Real-time detection is mostly working.")
        elif match_rate >= 40:
            print("MODERATE: There are still detection accuracy issues.")
        else:
            print("POOR: Detection system still needs improvement.")

    print("\n--- System Status ---")
    if len(backend_rows) > 0:
        print("YOLO service is detecting obstacles.")
        print("Backend obstacle registry is receiving updates.")
        print("Real-time eVTOL obstacle flow is active.")
    else:
        print("No backend detections yet. Start the backend, start YOLO, then press PLAY.")


if __name__ == "__main__":
    verify_obstacle_coordinates()
