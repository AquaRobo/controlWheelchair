from ultralytics import YOLO

class ObjectDetector:
    def __init__(self, model_path: str = 'yolov8n.pt'):
        self.model = YOLO(model_path)

    def detectObjects(self, frame) -> str:
        results = self.model(frame)
        detections = results[0].boxes
        detected_object = self.__getDetectedObjects(detections)
        return detected_object
    
    def __getDetectedObjects(self, detections) -> str:
        for box in detections:
            cls_id = int(box.cls)
            cls_name = self.model.names[cls_id]
            conf = float(box.conf)
            
            if cls_name in ['cup', 'bottle'] and conf > 0.5:  
                return cls_name
    