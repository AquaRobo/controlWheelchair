from ultralytics import YOLO

class RoomIdentifier:
    def __init__(self, model_path: str = 'yolov8n.pt'):
        self.model = YOLO(model_path)

    def identifyRoom(self, frame) -> str:
        results = self.model(frame, verbose=False)
        detections = results[0].boxes
        room = self.__classifyRoomFromDetections(detections)
        return room
    
    def __classifyRoomFromDetections(self, detections) -> str:
        room_scores = {'bedroom': 0, 'bathroom': 0, 'kitchen': 0, 'living_room': 0, 'unknown': 0}
        
        for box in detections:
            cls_id = int(box.cls)  # Class ID from YOLO
            conf = float(box.conf)  # Confidence score
            cls_name = self.model.names[cls_id]  # Get class name (e.g., 'bottle', 'bed')
            
            if cls_name in ['bed','couch', 'tv'] and conf > 0.5:  
                 room_scores['bedroom'] += conf
            elif cls_name in ['toilet', 'sink'] and conf > 0.5:  
                 room_scores['bathroom'] += conf
            elif cls_name in ['oven', 'microwave', 'refrigerator'] and conf > 0.5:  
                 room_scores['kitchen'] += conf
        
        
        # Return the room with the highest score, or 'unknown' if none
        best_room = max(room_scores, key=room_scores.get)
        return best_room if room_scores[best_room] > 0 else 'unknown'
