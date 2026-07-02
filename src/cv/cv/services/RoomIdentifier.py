import torch
import numpy as np
from dataclasses import dataclass
from ultralytics import YOLO


@dataclass
class RoomResult:
    room_name: str
    confidence: float
    annotated_frame: np.ndarray


class RoomIdentifier:
    """
    Identifies the room visible in a frame by scoring YOLO detections
    against a configurable object→room mapping.

    Config keys (from room_identification.yaml):
        model_path           : str   YOLO weights path
        confidence_threshold : float minimum detection confidence (default 0.5)
        room_scores          : dict  room_name → [yolo_class_names]
                               If omitted, a built-in default is used.
    """

    _DEFAULT_ROOM_SCORES: dict = {
        'bedroom':     ['bed', 'pillow', 'lamp'],
        'bathroom':    ['toilet', 'sink', 'toothbrush'],
        'kitchen':     ['oven', 'microwave', 'refrigerator',
                        'cup', 'bottle', 'bowl', 'fork', 'knife', 'spoon'],
        'living_room': ['couch', 'tv', 'chair', 'remote', 'book'],
    }

    def __init__(self, config: dict):
        self._conf_thresh: float = float(config.get('confidence_threshold', 0.5))

        model_path: str = config.get('model_path', 'yolov8n.pt')
        self._model = YOLO(model_path)
        if torch.cuda.is_available():
            self._model.to('cuda')

        # Load room_scores from config, fall back to built-in defaults
        room_scores: dict = config.get('room_scores', self._DEFAULT_ROOM_SCORES)

        # Build reverse lookup: yolo_class_name → room_name
        self._class_to_room: dict[str, str] = {}
        for room, classes in room_scores.items():
            for cls in classes:
                # If a class appears in multiple rooms, last-write wins;
                # arrange yaml in priority order to control this.
                self._class_to_room[cls] = room

        self._rooms: list[str] = list(room_scores.keys())

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def identifyRoom(self, frame: np.ndarray) -> RoomResult:
        """
        Run YOLO on *frame*, accumulate per-room confidence scores from
        matched detections, and return a RoomResult with the best room,
        its total score, and the YOLO-annotated frame.
        Returns 'unknown' with confidence 0.0 when no known object is found.
        """
        results = self._model(frame, verbose=False, conf=self._conf_thresh)
        annotated = results[0].plot()

        scores: dict[str, float] = {room: 0.0 for room in self._rooms}

        for box in results[0].boxes:
            cls_name = self._model.names[int(box.cls)]
            conf = float(box.conf)
            room = self._class_to_room.get(cls_name)
            if room is not None:
                scores[room] += conf

        best_room = max(scores, key=scores.get)
        best_conf = scores[best_room]

        if best_conf == 0.0:
            best_room = 'unknown'

        return RoomResult(
            room_name=best_room,
            confidence=best_conf,
            annotated_frame=annotated,
        )
