import os
import time
import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class MonoFrame:
    raw: np.ndarray
    calibrated: np.ndarray


@dataclass
class StereoFrame:
    left_raw: np.ndarray
    left_calibrated: np.ndarray
    right_raw: np.ndarray
    right_calibrated: np.ndarray


class CameraStreamer:
    _OPEN_RETRY_INTERVAL = 0.5   # seconds between retries
    _OPEN_TIMEOUT = 10.0         # total seconds before giving up

    def __init__(self, name: str, config: dict):
        self._name = name
        self._config = config
        self._is_stereo: bool = config.get('is_stereo', False)

        self._cap: Optional[cv2.VideoCapture] = None

        # Mono calibration maps
        self._mono_map1: Optional[np.ndarray] = None
        self._mono_map2: Optional[np.ndarray] = None

        # Stereo calibration maps
        self._left_map1: Optional[np.ndarray] = None
        self._left_map2: Optional[np.ndarray] = None
        self._right_map1: Optional[np.ndarray] = None
        self._right_map2: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def name(self) -> str:
        return self._name

    @property
    def is_stereo(self) -> bool:
        return self._is_stereo

    def open(self) -> bool:
        """
        Opens the camera capture, retrying every 0.5 s for up to 10 s.
        Applies codec, resolution and FPS from config.
        Returns True on success, False if the camera could not be opened
        within the timeout.
        """
        index = self._config.get('index', 0)
        deadline = time.time() + self._OPEN_TIMEOUT

        while time.time() < deadline:
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                self._cap = cap
                self._apply_config()
                self._load_calibration()
                return True
            cap.release()
            time.sleep(self._OPEN_RETRY_INTERVAL)

        return False

    def read(self) -> Optional[object]:
        """
        Reads the next frame(s).
        Returns a MonoFrame for normal cameras or a StereoFrame for stereo
        cameras. Returns None if the read fails.
        """
        if self._cap is None or not self._cap.isOpened():
            return None

        ret, frame = self._cap.read()
        if not ret or frame is None:
            return None

        if self._is_stereo:
            return self._process_stereo(frame)
        else:
            return self._process_mono(frame)

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _apply_config(self) -> None:
        fmt = self._config.get('format', 'MJPG')
        if len(fmt) == 4:
            fourcc = cv2.VideoWriter_fourcc(*fmt)
            self._cap.set(cv2.CAP_PROP_FOURCC, fourcc)

        width = self._config.get('width')
        height = self._config.get('height')
        fps = self._config.get('fps')

        if width:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        if height:
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        if fps:
            self._cap.set(cv2.CAP_PROP_FPS, float(fps))

    def _load_calibration(self) -> None:
        cal_value = self._config.get('calibration', 'NONE')
        cal_type = self._config.get('calibration_type', 'STANDARD')

        if str(cal_value).upper() == 'NONE':
            return

        if cal_type != 'STANDARD':
            # Unsupported calibration type — passthrough mode
            return

        # Resolve path: absolute or relative to the calibration folder
        if os.path.isabs(str(cal_value)):
            cal_path = str(cal_value)
        else:
            try:
                from utils.UtilityMethods import UtilityMethods
                cal_dir = UtilityMethods.getPackageCalibration('cv')
                cal_path = os.path.join(cal_dir, str(cal_value))
            except FileNotFoundError:
                return

        if not os.path.exists(cal_path):
            return

        try:
            data = np.load(cal_path)
            if self._is_stereo:
                self._load_stereo_calibration(data)
            else:
                self._load_mono_calibration(data)
        except Exception:
            pass

    def _load_mono_calibration(self, data: np.lib.npyio.NpzFile) -> None:
        required = {'K', 'D'}
        if not required.issubset(data.files):
            return

        K = data['K']
        D = data['D']
        img_size = self._get_image_size(data)
        if img_size is None:
            return

        self._mono_map1, self._mono_map2 = cv2.initUndistortRectifyMap(
            K, D, None, K, img_size, cv2.CV_32FC1
        )

    def _load_stereo_calibration(self, data: np.lib.npyio.NpzFile) -> None:
        required = {
            'K_left', 'D_left', 'K_right', 'D_right',
            'R_left', 'R_right', 'P_left', 'P_right',
        }
        if not required.issubset(data.files):
            return

        img_size = self._get_image_size(data)
        if img_size is None:
            return

        self._left_map1, self._left_map2 = cv2.initUndistortRectifyMap(
            data['K_left'], data['D_left'], data['R_left'],
            data['P_left'], img_size, cv2.CV_32FC1
        )
        self._right_map1, self._right_map2 = cv2.initUndistortRectifyMap(
            data['K_right'], data['D_right'], data['R_right'],
            data['P_right'], img_size, cv2.CV_32FC1
        )

    def _get_image_size(self, data: np.lib.npyio.NpzFile) -> Optional[tuple]:
        if 'image_size' in data.files:
            s = data['image_size']
            return (int(s[0]), int(s[1]))
        if self._cap is not None and self._cap.isOpened():
            w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if self._is_stereo:
                w = w // 2
            if w > 0 and h > 0:
                return (w, h)
        return None

    def _process_mono(self, frame: np.ndarray) -> MonoFrame:
        if self._mono_map1 is not None:
            calibrated = cv2.remap(frame, self._mono_map1, self._mono_map2,
                                   cv2.INTER_LINEAR)
        else:
            calibrated = frame
        return MonoFrame(raw=frame, calibrated=calibrated)

    def _process_stereo(self, frame: np.ndarray) -> StereoFrame:
        mid = frame.shape[1] // 2
        left_raw = frame[:, :mid]
        right_raw = frame[:, mid:]

        if self._left_map1 is not None:
            left_cal = cv2.remap(left_raw, self._left_map1, self._left_map2,
                                 cv2.INTER_LINEAR)
            right_cal = cv2.remap(right_raw, self._right_map1, self._right_map2,
                                  cv2.INTER_LINEAR)
        else:
            left_cal = left_raw
            right_cal = right_raw

        return StereoFrame(
            left_raw=left_raw,
            left_calibrated=left_cal,
            right_raw=right_raw,
            right_calibrated=right_cal,
        )
