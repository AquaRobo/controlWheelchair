import cv2
import time


BASELINE = 0.265  
FOCAL_LENGTH_PIXELS = 900
USE_DOWNSCALING = False
DOWNSCALE_WIDTH = 640  

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

def main():
    cap = initialize_stereo_camera_mjpeg()
    if cap is None:
        return

    print("System Initialized.")
    print("Press 'q' to quit\n")
        
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
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

        cv2.imshow("Left Camera", left_proc)
        cv2.imshow("Right Camera", right_proc)
    
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()