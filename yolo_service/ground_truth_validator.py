"""
Ground truth validator for YOLO obstacle detections.
Compares detected coordinates against the frontend ground-truth obstacle CSV.
"""

import csv
import numpy as np
from typing import List, Dict
from scipy.spatial.distance import cdist


class GroundTruthValidator:
    """Load and validate detections against ground truth."""
    
    def __init__(self, obstacles_csv_path: str):
        """Load obstacles from CSV."""
        self.ground_truth = self._load_obstacles(obstacles_csv_path)
        self.matches_history = []
    
    @staticmethod
    def _load_obstacles(csv_path: str) -> np.ndarray:
        """
        Load the frontend obstacle CSV into a numpy array.
        
        Format:
            x,y,z,radius
            -300,0,100,155
            250,0,800,155
            ...
        """
        obstacles = []
        try:
            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if not row.get('x'):
                        continue
                    obstacles.append([
                        float(row['x']),
                        float(row['y']),
                        float(row['z']),
                        float(row.get('radius', 0)),
                    ])
            return np.array(obstacles)
        except Exception as e:
            print(f"Error loading obstacles: {e}")
            return np.array([])
    
    def match_detections(self, detections: List[Dict]) -> List[Dict]:
        """
        Match detected obstacles to ground truth.
        
        Args:
            detections: List of detected obstacles with X_world, Y_world, Z_world
            
        Returns:
            List of matches with error metrics
        """
        if len(detections) == 0 or len(self.ground_truth) == 0:
            return []
        
        # Extract coordinates
        det_points = np.array([
            [d['X_world'], d['Y_world'], d['Z_world']]
            for d in detections
        ])
        
        gt_points = self.ground_truth[:, :3]
        
        # Compute distances
        distances = cdist(det_points, gt_points, metric='euclidean')
        
        matches = []
        matched_gt_indices = set()
        
        # Match each detection to nearest ground truth
        for i, det in enumerate(detections):
            nearest_gt_idx = np.argmin(distances[i])
            nearest_distance = distances[i, nearest_gt_idx]
            
            # Only match if within reasonable threshold (50m)
            if nearest_distance > 80:
                matches.append({
                    'detection': det,
                    'ground_truth': None,
                    'matched': False,
                    'distance_error_m': nearest_distance,
                    'notes': 'No ground truth match within 80m'
                })
                continue
            
            gt = self.ground_truth[nearest_gt_idx]
            matched_gt_indices.add(nearest_gt_idx)
            
            matches.append({
                'detection': det,
                'ground_truth': {
                    'x': float(gt[0]),
                    'y': float(gt[1]),
                    'z': float(gt[2]),
                    'radius': float(gt[3]),
                },
                'matched': True,
                'distance_error_m': float(nearest_distance),
                'error_x': float(det['X_world'] - gt[0]),
                'error_y': float(det['Y_world'] - gt[1]),
                'error_z': float(det['Z_world'] - gt[2]),
            })
        
        # Find unmatched ground truth
        for i, gt in enumerate(self.ground_truth):
            if i not in matched_gt_indices:
                matches.append({
                    'detection': None,
                    'ground_truth': {
                        'x': float(gt[0]),
                        'y': float(gt[1]),
                        'z': float(gt[2]),
                        'radius': float(gt[3]),
                    },
                    'matched': False,
                    'distance_error_m': None,
                    'notes': 'Missed detection'
                })
        
        self.matches_history.append(matches)
        return matches
    
    def get_metrics(self, matches: List[Dict]) -> Dict:
        """Compute accuracy metrics from matches."""
        matched = [m for m in matches if m['matched']]
        
        if len(matched) == 0:
            return {
                'total_detections': len([m for m in matches if m['detection']]),
                'total_ground_truth': len([m for m in matches if m['ground_truth']]),
                'matched_count': 0,
                'missed_count': len([m for m in matches if m.get('notes') == 'Missed detection']),
                'mean_error_m': None,
                'std_error_m': None,
                'max_error_m': None,
                'rmse_m': None,
                'precision': 0.0,
                'recall': 0.0,
            }
        
        errors = np.array([m['distance_error_m'] for m in matched])
        
        return {
            'total_detections': len([m for m in matches if m['detection']]),
            'total_ground_truth': len([m for m in matches if m['ground_truth']]),
            'matched_count': len(matched),
            'missed_count': len([m for m in matches if m.get('notes') == 'Missed detection']),
            'mean_error_m': float(np.mean(errors)),
            'std_error_m': float(np.std(errors)),
            'max_error_m': float(np.max(errors)),
            'min_error_m': float(np.min(errors)),
            'rmse_m': float(np.sqrt(np.mean(errors**2))),
            'percentile_90': float(np.percentile(errors, 90)),
            'percentile_95': float(np.percentile(errors, 95)),
            'precision': len(matched) / max(len([m for m in matches if m['detection']]), 1),
            'recall': len(matched) / max(len([m for m in matches if m['ground_truth']]), 1),
        }
