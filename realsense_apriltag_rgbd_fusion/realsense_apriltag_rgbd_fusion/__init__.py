"""
realsense_apriltag_rgbd_fusion package.
Exposes public interfaces for modular perception and control integration.
"""

from realsense_apriltag_rgbd_fusion.fusion_engine import RgbdTagFusionEngine
from realsense_apriltag_rgbd_fusion.types import CameraIntrinsics, FusionOutput
from realsense_apriltag_rgbd_fusion.visualizer import TagVisualizer

__version__ = "1.0.0"

__all__ = [
    "CameraIntrinsics",
    "FusionOutput",
    "RgbdTagFusionEngine",
    "TagVisualizer",
]
