import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional

from ultralytics import YOLO
import torch

@dataclass
class DetectionResult:
    object_name: str
    confidence: float
    x: float
    y: float
    z: float
    annotated_frame: np.ndarray


class ObjectDetector:
    """
    Detects objects in a stereo frame pair using YOLO and estimates
    3-D position (X, Y, Z in metres, camera frame) via SGM disparity.

    Config keys (merged from object_detection.yaml + cameras.yaml):
        model_path            : str   path to YOLO weights / ncnn model dir
        confidence_threshold  : float (default 0.5)
        iou_threshold         : float (default 0.45)
        classes               : list  target class names; empty = all classes
        downscale_width       : int   width used for stereo matching (default 640)
        min_valid_pixels      : int   minimum valid disparity pixels (default 20)
        baseline              : float stereo baseline in metres
        focal_length_pixels   : float focal length in pixels
        disparity_offset      : float empirical disparity correction offset
    """

    def __init__(self, config: dict):
        self._conf_thresh: float = float(config.get('confidence_threshold', 0.5))
        self._iou_thresh: float = float(config.get('iou_threshold', 0.45))
        self._classes: list = config.get('classes', [])
        self._downscale_width: int = int(config.get('downscale_width', 640))
        self._min_valid_pixels: int = int(config.get('min_valid_pixels', 20))

        self._baseline: float = float(config.get('baseline', 0.13))
        self._focal_length: float = float(config.get('focal_length_pixels', 783.0))
        self._disparity_offset: float = float(config.get('disparity_offset', 42.14))

        model_path: str = config.get('model_path', 'yolov8n.pt')
        self._model = YOLO(model_path, task='detect')
        if torch.cuda.is_available():
            self._model.to('cuda')
        self._stereo, self._num_disp = self._init_stereo_matcher()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def detect(self, left_frame: np.ndarray, right_frame: np.ndarray) -> list:
        """
        Run YOLO on *left_frame*, depth-estimate each accepted detection via
        the stereo pair, and return a list of DetectionResult objects.
        The annotated_frame inside every result is the same YOLO-annotated
        left image (drawn once and shared across all results in a frame).
        """
        results = self._model(
            left_frame,
            conf=self._conf_thresh,
            iou=self._iou_thresh,
            verbose=False,
        )
        annotated = results[0].plot()

        h, w = left_frame.shape[:2]
        c_x, c_y = w / 2.0, h / 2.0

        scale = self._downscale_width / w
        new_h = int(h * scale)

        left_gray = cv2.cvtColor(
            cv2.resize(left_frame, (self._downscale_width, new_h)),
            cv2.COLOR_BGR2GRAY,
        )
        right_gray = cv2.cvtColor(
            cv2.resize(right_frame, (self._downscale_width, new_h)),
            cv2.COLOR_BGR2GRAY,
        )
        adj_focal = self._focal_length * scale

        detections: list = []
        for box in results[0].boxes:
            cls_name = self._model.names[int(box.cls)]
            conf = float(box.conf)

            if self._classes and cls_name not in self._classes:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            xyz = self._compute_xyz(
                left_gray, right_gray,
                x1, y1, x2, y2,
                cx, cy, c_x, c_y,
                scale, adj_focal, new_h,
            )
            if xyz is None:
                continue

            X, Y, Z = xyz
            detections.append(DetectionResult(
                object_name=cls_name,
                confidence=conf,
                x=X, y=Y, z=Z,
                annotated_frame=annotated,
            ))

        return detections

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _init_stereo_matcher(self):
        window_size = 5
        num_disp = 16 * 10
        stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disp,
            blockSize=window_size,
            P1=8 * 3 * window_size ** 2,
            P2=32 * 3 * window_size ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        return stereo, num_disp

    def _compute_xyz(
        self,
        left_gray: np.ndarray,
        right_gray: np.ndarray,
        x1: int, y1: int, x2: int, y2: int,
        cx: int, cy: int,
        c_x: float, c_y: float,
        scale: float,
        adj_focal: float,
        new_h: int,
    ) -> Optional[tuple]:
        """
        Compute (X, Y, Z) in metres for a detected bounding box using the
        SGBM disparity map cropped to the detection ROI.
        Returns None when there are insufficient valid disparity pixels or
        when the corrected disparity is too small to give a reliable depth.
        """
        sx1, sy1 = int(x1 * scale), int(y1 * scale)
        sx2, sy2 = int(x2 * scale), int(y2 * scale)

        margin = 20
        crop_y1 = max(0, sy1 - margin)
        crop_y2 = min(new_h, sy2 + margin)
        crop_x2 = min(self._downscale_width, sx2 + margin)
        crop_x1 = max(0, sx1 - self._num_disp - margin)

        # Ensure the crop is wide enough for SGBM
        min_width = self._num_disp + 10
        if (crop_x2 - crop_x1) < min_width:
            crop_x2 = min(self._downscale_width, crop_x1 + min_width)
            if (crop_x2 - crop_x1) < min_width:
                crop_x1 = max(0, crop_x2 - min_width)

        if (crop_y2 <= crop_y1
                or crop_x2 <= crop_x1
                or (crop_x2 - crop_x1) <= self._num_disp):
            return None

        disp = (
            self._stereo
            .compute(
                left_gray[crop_y1:crop_y2, crop_x1:crop_x2],
                right_gray[crop_y1:crop_y2, crop_x1:crop_x2],
            )
            .astype(np.float32) / 16.0
        )

        dcx = int(cx * scale) - crop_x1
        dcy = int(cy * scale) - crop_y1

        rw = max(int((x2 - x1) * scale * 0.3), 15)
        rh = max(int((y2 - y1) * scale * 0.3), 15)

        roi = disp[
            max(0, dcy - rh): min(disp.shape[0], dcy + rh),
            max(0, dcx - rw): min(disp.shape[1], dcx + rw),
        ]
        valid = roi[(roi > 1.0) & (roi < self._num_disp)]

        if len(valid) < self._min_valid_pixels:
            return None

        sorted_v = np.sort(valid)
        trim = int(len(sorted_v) * 0.2)
        avg_disp = np.median(sorted_v[trim:-trim] if trim > 0 else sorted_v)

        corrected = avg_disp - self._disparity_offset
        if corrected <= 0.5:
            return None

        Z = (adj_focal * self._baseline) / corrected
        X = ((cx - c_x) * Z) / self._focal_length
        Y = ((cy - c_y) * Z) / self._focal_length

        return round(X, 4), round(Y, 4), round(Z, 4)
