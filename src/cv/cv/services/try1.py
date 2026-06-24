import cv2
import numpy as np
import time
from ultralytics import YOLO

class BottleVisionNode:
    def __init__(self):
        # ==================== HARDWARE CONSTRAINTS ====================
        self.BASELINE = 0.13  
        self.FOCAL_LENGTH_PIXELS = 783  
        self.DISPARITY_OFFSET = 42.14   
        self.DOWNSCALE_WIDTH = 640
        self.MIN_VALID_PIXELS = 20

        print("[INFO] Initializing Vision Node...")
        self.model = YOLO('yolov8n.pt', task='detect')
        self.cap = self._initialize_camera()
        self.stereo, self.num_disp = self._initialize_stereo_matcher()
        
        # Warm up the camera
        for _ in range(10):
            self.cap.read()
            
        print("[INFO] Vision Node Ready.")

    def _initialize_camera(self):
        """Your MJPEG Initialization Logic"""
        cap = cv2.VideoCapture("/dev/video2", cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        time.sleep(0.5)
        
        resolutions = [(640, 480), (1280, 720), (1920, 1080), (3840, 1080)]
        for w, h in resolutions:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            time.sleep(0.3)
            
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        cap.set(cv2.CAP_PROP_EXPOSURE, -6)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        return cap

    def _initialize_stereo_matcher(self):
        window_size = 5
        min_disp = 0
        nDispFactor = 10
        num_disp = 16 * nDispFactor - min_disp
        
        stereo = cv2.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=window_size,
            P1=8 * 3 * window_size**2,
            P2=32 * 3 * window_size**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        return stereo, num_disp

    def get_bottle_coordinates(self, max_attempts=10):
        """
        Attempts to find a bottle and returns its (X, Y, Z) coordinates in meters relative to the camera lens.
        Returns None if no bottle is found after 'max_attempts' frames.
        """
        attempts = 0
        
        # Clear buffer to ensure we get the latest frame
        for _ in range(3):
            self.cap.grab()

        while attempts < max_attempts:
            ret, frame = self.cap.read()
            if not ret:
                break
                
            attempts += 1
            h_orig, w_orig, _ = frame.shape
            mid = w_orig // 2
            
            left_img = frame[:, :mid]
            right_img = frame[:, mid:]
            
            # Optical centers (assuming standard centered pinhole camera)
            c_x = left_img.shape[1] / 2.0
            c_y = left_img.shape[0] / 2.0
            
            # Downscale
            scale_factor = self.DOWNSCALE_WIDTH / left_img.shape[1]
            new_height = int(left_img.shape[0] * scale_factor)
            left_proc = cv2.resize(left_img, (self.DOWNSCALE_WIDTH, new_height))
            right_proc = cv2.resize(right_img, (self.DOWNSCALE_WIDTH, new_height))
            
            left_gray = cv2.cvtColor(left_proc, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_proc, cv2.COLOR_BGR2GRAY)
            
            # Inference
            results = self.model(left_img, verbose=False)
            detections = results[0].boxes
            
            adjusted_focal_length = self.FOCAL_LENGTH_PIXELS * scale_factor
            
            for box in detections:
                cls_id = int(box.cls)
                cls_name = self.model.names[cls_id]
                conf = float(box.conf)
                
                if cls_name == 'bottle' and conf > 0.5:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    # Crop Logic
                    scaled_x1 = int(x1 * scale_factor)
                    scaled_y1 = int(y1 * scale_factor)
                    scaled_x2 = int(x2 * scale_factor)
                    scaled_y2 = int(y2 * scale_factor)
                    
                    margin_y = 20
                    crop_y1 = max(0, scaled_y1 - margin_y)
                    crop_y2 = min(new_height, scaled_y2 + margin_y)
                    
                    margin_x = 20
                    crop_x2 = min(self.DOWNSCALE_WIDTH, scaled_x2 + margin_x)
                    crop_x1 = max(0, scaled_x1 - self.num_disp - margin_x)
                    
                    if (crop_x2 - crop_x1) < self.num_disp + 10:
                        crop_x2 = min(self.DOWNSCALE_WIDTH, crop_x1 + self.num_disp + 10)
                        if (crop_x2 - crop_x1) < self.num_disp + 10:
                            crop_x1 = max(0, crop_x2 - self.num_disp - 10)
                    
                    if crop_y2 <= crop_y1 or crop_x2 <= crop_x1 or (crop_x2 - crop_x1) <= self.num_disp:
                        continue 
                    
                    left_crop = left_gray[crop_y1:crop_y2, crop_x1:crop_x2]
                    right_crop = right_gray[crop_y1:crop_y2, crop_x1:crop_x2]
                    
                    # Disparity
                    disparity_raw = self.stereo.compute(left_crop, right_crop)
                    disparity_float = disparity_raw.astype(np.float32) / 16.0
                    
                    disp_center_x = int(center_x * scale_factor) - crop_x1
                    disp_center_y = int(center_y * scale_factor) - crop_y1
                    
                    box_width = int((x2 - x1) * scale_factor)
                    box_height = int((y2 - y1) * scale_factor)
                    roi_width = max(int(box_width * 0.6) // 2, 15)
                    roi_height = max(int(box_height * 0.6) // 2, 15)
                    
                    roi_y_start = max(0, disp_center_y - roi_height)
                    roi_y_end = min(disparity_float.shape[0], disp_center_y + roi_height)
                    roi_x_start = max(0, disp_center_x - roi_width)
                    roi_x_end = min(disparity_float.shape[1], disp_center_x + roi_width)
                    
                    roi = disparity_float[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
                    valid_pixels = roi[(roi > 1.0) & (roi < self.num_disp)]
                    
                    if len(valid_pixels) >= self.MIN_VALID_PIXELS:
                        sorted_pixels = np.sort(valid_pixels)
                        trim_count = int(len(sorted_pixels) * 0.2)
                        trimmed_pixels = sorted_pixels[trim_count:-trim_count] if trim_count > 0 else sorted_pixels
                        avg_disp = np.median(trimmed_pixels)
                        
                        corrected_disp = avg_disp - self.DISPARITY_OFFSET
                        
                        if corrected_disp > 0.5: 
                            # 1. Calculate Z (Depth in meters)
                            Z = (adjusted_focal_length * self.BASELINE) / corrected_disp
                            
                            # 2. Calculate X (Horizontal offset in meters)
                            # Positive X is right of the camera center, negative is left
                            X = ((center_x - c_x) * Z) / self.FOCAL_LENGTH_PIXELS
                            
                            # 3. Calculate Y (Vertical offset in meters)
                            # Positive Y is below the camera center, negative is above
                            Y = ((center_y - c_y) * Z) / self.FOCAL_LENGTH_PIXELS
                            
                            return (round(X, 4), round(Y, 4), round(Z, 4))
                            
        return None # Return None if no bottle was found or depth was invalid

    def release(self):
        self.cap.release()

# ==================== HOW TO USE THIS IN YOUR MAIN ROS NODE ====================
if __name__ == "__main__":
    # 1. Initialize the node ONCE at startup
    vision = BottleVisionNode()
    
    print("\n--- Requesting Coordinates ---")
    
    # 2. Call this whenever the robotic arm needs the coordinates
    coords = vision.get_bottle_coordinates()
    
    if coords:
        x, y, z = coords
        print(f"Bottle Found! 3D Coordinates relative to camera lens:")
        print(f"X (Left/Right): {x} m")
        print(f"Y (Up/Down):    {y} m")
        print(f"Z (Depth):      {z} m")
    else:
        print("Bottle not found or depth unstable.")
        
    # 3. Release when shutting down the system
    vision.release()