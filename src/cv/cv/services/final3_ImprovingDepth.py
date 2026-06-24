import cv2
import numpy as np
import time
from ultralytics import YOLO
from collections import deque
import torch


BASELINE = 0.265  
FOCAL_LENGTH_PIXELS = 900
USE_DOWNSCALING = True
DOWNSCALE_WIDTH = 640  
DEPTH_HISTORY_SIZE = 10  
MIN_VALID_PIXELS = 20  
USE_MEDIAN_FILTER = True  

def initialize_stereo_camera_mjpeg():
    """Initialize stereo camera with MJPEG encoding for high FPS"""
    camera_index = "/dev/video2"  # Change this to your stereo camera index
    
    print(f"\n[1/7] Opening Camera {camera_index} with DirectShow...")
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print("[X] ERROR: Camera not found!")
        return None
    
    print("[OK] Camera opened")
    

    print("\n[2/7] Setting MINIMUM resolution (160x120)...")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    print("[3/7] Flushing camera buffer...")
    for i in range(5):
        cap.read()
        time.sleep(0.1)
    

    print("\n[4/7] Forcing MJPEG codec (Method 1)...")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    time.sleep(0.5)
    
   
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    codec_str = "".join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
    print(f"   Codec after Method 1: {codec_str}")
    
    if codec_str != "MJPG":
        print("\n[4b/7] Trying alternate MJPEG code...")
        cap.set(cv2.CAP_PROP_FOURCC, 1196444237)  # MJPG in decimal
        time.sleep(0.3)
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        codec_str = "".join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
        print(f"   Codec after Method 2: {codec_str}")
   
    print("\n[5/7] Gradually increasing resolution...")
    resolutions = [(640, 480), (1280, 720), (1920, 1080), (3840, 1080)]
    
    
    for w, h in resolutions:
        print(f"   Trying {w}x{h}...")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        time.sleep(0.3)
        
      
        real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        if real_w == w and real_h == h:
            print(f"   [OK] Camera accepted {w}x{h}")
        else:
            print(f"   [!] Camera gave {real_w}x{real_h} instead")
            if w == 3840:  # If we can't get 3840, that's okay
                print(f"   [NOTE] Using fallback resolution {real_w}x{real_h}")
            break
    
    
    final_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    final_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    final_fps = int(cap.get(cv2.CAP_PROP_FPS))
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    codec_str = "".join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
    
    print(f"\n[6/7] Final Camera Configuration:")
    print(f"   Resolution: {final_w}x{final_h}")
    print(f"   Target FPS: {final_fps}")
    print(f"   Codec: {codec_str}")
    
    if codec_str != "MJPG":
        print("\n   [!!] WARNING: MJPEG NOT ACTIVE!")
        print("   Expected FPS: 5-10 (uncompressed mode)")
    else:
        print("\n   [OK] MJPEG is active - should get 25-30 FPS")
    

    print("\n[7/7] Optimizing exposure settings...")
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, -6)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    
    print("\n" + "=" * 60)
    print("Camera initialization complete!")
    print("=" * 60 + "\n")
    
    return cap

def initialize_yolo():
    """Initialize YOLO model with GPU if available"""
    print("Loading YOLO model...")
    
    # if torch.cuda.is_available():
    #     print(f"✅ GPU DETECTED: {torch.cuda.get_device_name(0)}")
    #     model = YOLO('yolov8l.pt')  
    #     model.to('cuda')
    # else:
    print("⚠ WARNING: GPU not found. Running on CPU.")
    model = YOLO('yolov8n.pt')
    
    return model

def initialize_stereo_matcher():
    """Initialize StereoSGBM matcher"""
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

def main():
    cap = initialize_stereo_camera_mjpeg()
    if cap is None:
        return
    
    model = initialize_yolo()
    stereo, num_disp = initialize_stereo_matcher()
    
    depth_history = {
        'cup': deque(maxlen=DEPTH_HISTORY_SIZE),
        'bottle': deque(maxlen=DEPTH_HISTORY_SIZE)
    }
    
    print("System Initialized. Starting detection...")
    print("Press 'q' to quit\n")
    
    fps_list = []
    prev_time = time.time()
    frame_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        prev_time = curr_time
        fps_list.append(fps)
        frame_count += 1
        
        h_orig, w_orig, _ = frame.shape
        mid = w_orig // 2
        
        left_img = frame[:, :mid]
        right_img = frame[:, mid:]
        
        # 1. Downscale full images 
        if USE_DOWNSCALING:
            scale_factor = DOWNSCALE_WIDTH / left_img.shape[1]
            new_height = int(left_img.shape[0] * scale_factor)
            left_proc = cv2.resize(left_img, (DOWNSCALE_WIDTH, new_height))
            right_proc = cv2.resize(right_img, (DOWNSCALE_WIDTH, new_height))
        else:
            scale_factor = 1.0
            left_proc = left_img
            right_proc = right_img
            new_height = left_img.shape[0]
        
        left_gray = cv2.cvtColor(left_proc, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_proc, cv2.COLOR_BGR2GRAY)
        
  
        results = model(left_img)
        detections = results[0].boxes
        
        adjusted_focal_length = FOCAL_LENGTH_PIXELS * scale_factor
        detected_objects = set()
        
        for box in detections:
            cls_id = int(box.cls)
            cls_name = model.names[cls_id]
            conf = float(box.conf)
            
            if cls_name in ['cup', 'bottle'] and conf > 0.5:
                detected_objects.add(cls_name)
                
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
               
                scaled_x1 = int(x1 * scale_factor)
                scaled_y1 = int(y1 * scale_factor)
                scaled_x2 = int(x2 * scale_factor)
                scaled_y2 = int(y2 * scale_factor)
                
                margin_y = 20
                crop_y1 = max(0, scaled_y1 - margin_y)
                crop_y2 = min(new_height, scaled_y2 + margin_y)
                
                margin_x = 20
                crop_x2 = min(DOWNSCALE_WIDTH, scaled_x2 + margin_x)
                crop_x1 = max(0, scaled_x1 - num_disp - margin_x)
      
                if (crop_x2 - crop_x1) < num_disp + 10:
                    crop_x2 = min(DOWNSCALE_WIDTH, crop_x1 + num_disp + 10)
                    if (crop_x2 - crop_x1) < num_disp + 10:
                        crop_x1 = max(0, crop_x2 - num_disp - 10)
             
                if crop_y2 <= crop_y1 or crop_x2 <= crop_x1 or (crop_x2 - crop_x1) <= num_disp:
                    continue 
             
                left_crop = left_gray[crop_y1:crop_y2, crop_x1:crop_x2]
                right_crop = right_gray[crop_y1:crop_y2, crop_x1:crop_x2]
               
                disparity_raw = stereo.compute(left_crop, right_crop)
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
                
                
                valid_pixels = roi[(roi > 1.0) & (roi < num_disp)]
                
                if len(valid_pixels) >= MIN_VALID_PIXELS:
                    if USE_MEDIAN_FILTER:
                        sorted_pixels = np.sort(valid_pixels)
                        trim_count = int(len(sorted_pixels) * 0.2)
                        if trim_count > 0:
                            trimmed_pixels = sorted_pixels[trim_count:-trim_count]
                        else:
                            trimmed_pixels = sorted_pixels
                        avg_disp = np.median(trimmed_pixels)
                        print(f"Raw avg_disp: {avg_disp}")
                    else:
                        avg_disp = np.mean(valid_pixels)
                        print(f"Raw avg_disp: {avg_disp}")
                    
                    current_depth = (adjusted_focal_length * BASELINE) / avg_disp
                    depth_history[cls_name].append(current_depth)
                    
                    if len(depth_history[cls_name]) >= 3:
                        depth_array = np.array(depth_history[cls_name])
                        filtered_depth = np.median(depth_array)
                        std_dev = np.std(depth_array)
                        is_stable = std_dev < 0.05
                    else:
                        filtered_depth = current_depth
                        is_stable = False
                    
   
                    if is_stable:
                        color = (0, 255, 0)
                    elif filtered_depth < 0.5:
                        color = (0, 0, 255)
                    elif filtered_depth < 2.0:
                        color = (0, 255, 255)
                    else:
                        color = (0, 255, 0)
                    
                    thickness = 3 if is_stable else 2
                    cv2.rectangle(left_img, (x1, y1), (x2, y2), color, thickness)
                    cv2.circle(left_img, (center_x, center_y), 8, color, -1)
                    
                    stability_indicator = " [STABLE]" if is_stable else ""
                    label = f"{cls_name} | {filtered_depth:.3f}m{stability_indicator}"
                    
                    (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    cv2.rectangle(left_img, (x1, y1 - text_h - 10), (x1 + text_w + 10, y1), color, -1)
                    cv2.putText(left_img, label, (x1 + 5, y1 - 5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                else:
                    cv2.rectangle(left_img, (x1, y1), (x2, y2), (128, 128, 128), 2)
                    cv2.putText(left_img, f"{cls_name} | Insufficient data", (x1, y1 - 5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 2)
        
        for obj_type in depth_history.keys():
            if obj_type not in detected_objects:
                depth_history[obj_type].clear()
        
        if frame_count % 30 == 0:
            avg_fps = sum(fps_list[-30:]) / len(fps_list[-30:])
            print(f"[OK] FPS: {avg_fps:.1f}")
        
        fps_color = (0, 255, 0) if fps > 20 else (0, 165, 255) if fps > 10 else (0, 0, 255)
        cv2.putText(left_img, f"FPS: {int(fps)}", (30, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, fps_color, 3)
        
        height, width = left_img.shape[:2]
        display_frame = cv2.resize(left_img, (width // 2, height // 2))
        cv2.imshow("Stereo Depth Detection (Cropped SGBM)", display_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()