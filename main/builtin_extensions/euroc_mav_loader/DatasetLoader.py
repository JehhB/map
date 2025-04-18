import os
from glob import glob
from typing import Dict, List, Optional, TypedDict, final

import cv2
import yaml


class ImagePair(TypedDict):
    left: str
    right: str
    timestamp: int
    filename: str


class TBSData(TypedDict):
    cols: int
    rows: int
    data: List[float]


class SensorData(TypedDict):
    sensor_type: str
    comment: str
    T_BS: TBSData
    rate_hz: int
    resolution: List[int]
    camera_model: str
    intrinsics: List[float]
    distortion_model: str
    distortion_coefficients: List[float]


class Metadata(TypedDict):
    timestamp: int
    filename: str
    left_cam_info: Optional[SensorData]
    right_cam_info: Optional[SensorData]


@final
class DatasetLoader:
    def __init__(self, dataset_path: str):
        self.dataset_path = dataset_path
        self.left_image_dir = os.path.join(dataset_path, "mav0", "cam0", "data")
        self.right_image_dir = os.path.join(dataset_path, "mav0", "cam1", "data")

        # Load timestamps
        self.left_data_csv = os.path.join(dataset_path, "mav0", "cam0", "data.csv")
        self.right_data_csv = os.path.join(dataset_path, "mav0", "cam1", "data.csv")

        # Load calibration
        self.left_yaml = os.path.join(dataset_path, "mav0", "cam0", "sensor.yaml")
        self.right_yaml = os.path.join(dataset_path, "mav0", "cam1", "sensor.yaml")

        self._load_metadata()
        self._index_images()
        self.current_idx = 0

    def _load_metadata(self):
        """Load calibration and timestamp metadata"""
        # Load camera calibration
        try:
            with open(self.left_yaml, "r") as f:
                self.left_cam_info: Optional[SensorData] = yaml.safe_load(f)

            with open(self.right_yaml, "r") as f:
                self.right_cam_info: Optional[SensorData] = yaml.safe_load(f)
        except (FileNotFoundError, yaml.YAMLError) as e:
            print(f"Error loading calibration files: {e}")
            self.left_cam_info = None
            self.right_cam_info = None

        # Load timestamps
        self.timestamps: Dict[str, int] = {}
        try:
            with open(self.left_data_csv, "r") as f:
                # Skip header
                _ = next(f)

                for line in f:
                    if line.strip():
                        ts, filename = line.strip().split(",")
                        filename = os.path.basename(filename)
                        self.timestamps[filename] = int(ts)

        except FileNotFoundError as e:
            print(f"Error loading timestamp file: {e}")

    def _index_images(self):
        """Index all images in the dataset"""
        # Get all image files and sort them
        self.left_images = sorted(glob(os.path.join(self.left_image_dir, "*.png")))
        self.right_images = sorted(glob(os.path.join(self.right_image_dir, "*.png")))

        # Create a mapping from index to filenames
        self.image_pairs: List[ImagePair] = []

        # Match left and right images based on timestamps
        left_files_dict = {os.path.basename(f): f for f in self.left_images}
        right_files_dict = {os.path.basename(f): f for f in self.right_images}

        # Find common image names (should be the same for stereo pairs)
        common_filenames = sorted(
            set(left_files_dict.keys()).intersection(right_files_dict.keys())
        )

        for filename in common_filenames:
            self.image_pairs.append(
                {
                    "left": left_files_dict[filename],
                    "right": right_files_dict[filename],
                    "timestamp": self.timestamps.get(filename, 0),
                    "filename": filename,
                }
            )

    def __len__(self):
        """Return the number of stereo image pairs in the dataset"""
        return len(self.image_pairs)

    def __getitem__(self, idx: int):
        """
        Get a stereo image pair by index.

        Args:
            idx (int): Index of the image pair to retrieve

        Returns:
            tuple: (left_img, right_img, metadata)
                left_img: OpenCV matrix for left camera image
                right_img: OpenCV matrix for right camera image
                metadata: Dictionary containing timestamp and other info
        """
        if idx < 0 or idx >= len(self):
            raise IndexError(
                f"Index {idx} out of range for dataset with {len(self)} items"
            )

        pair = self.image_pairs[idx]

        # Load images
        left_img = cv2.imread(pair["left"], cv2.IMREAD_UNCHANGED)
        right_img = cv2.imread(pair["right"], cv2.IMREAD_UNCHANGED)

        # Prepare metadata
        metadata: Metadata = {
            "timestamp": pair["timestamp"],
            "filename": pair["filename"],
            "left_cam_info": self.left_cam_info,
            "right_cam_info": self.right_cam_info,
        }

        return (left_img, right_img, metadata)

    def __iter__(self):
        """Iterator for the dataset"""
        self.current_idx = 0
        return self

    def __next__(self):
        """Get the next stereo image pair"""
        if self.current_idx < len(self):
            result = self[self.current_idx]
            self.current_idx += 1
            return result
        else:
            raise StopIteration

    def get_calibration(self):
        """Return the stereo camera calibration parameters"""
        return {"left": self.left_cam_info, "right": self.right_cam_info}

    def get_timestamps(self):
        """Return a list of all timestamps in the dataset"""
        return [pair["timestamp"] for pair in self.image_pairs]
