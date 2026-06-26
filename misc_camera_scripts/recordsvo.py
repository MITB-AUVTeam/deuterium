import pyzed.sl as sl
import cv2

# ==========================
# INIT ZED
# ==========================

zed = sl.Camera(1)

init_params = sl.InitParameters()

init_params.camera_resolution = sl.RESOLUTION.HD2K
init_params.camera_fps = 15

# Optional custom calibration
init_params.optional_opencv_calibration_file = \
"/home/cupcake/Downloads/new_calib_front.yaml"

# Disable self calibration
init_params.camera_disable_self_calib = True

status = zed.open(init_params)

if status != sl.ERROR_CODE.SUCCESS:
    print("[ERROR] Camera open failed:", status)
    exit()

print("[INFO] Camera opened")


# ==========================
# ENABLE SVO RECORDING
# ==========================

recording_params = sl.RecordingParameters(
    "empty_pool_2.svo2",
    sl.SVO_COMPRESSION_MODE.H264
)

err = zed.enable_recording(recording_params)

if err != sl.ERROR_CODE.SUCCESS:
    print("[ERROR] Recording failed:", err)
    zed.close()
    exit()

print("[INFO] SVO recording started")


# ==========================
# SETUP
# ==========================

runtime = sl.RuntimeParameters()

image = sl.Mat()

frame_count = 0

# Preview scale
preview_scale = 0.45

# Create preview window
cv2.namedWindow(
    "ZED Preview",
    cv2.WINDOW_NORMAL
)


# ==========================
# MAIN LOOP
# ==========================

try:

    while True:

        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:

            # Get left camera image
            zed.retrieve_image(
                image,
                sl.VIEW.LEFT
            )

            frame = image.get_data()

            # Convert BGRA -> BGR
            frame = cv2.cvtColor(
                frame,
                cv2.COLOR_BGRA2BGR
            )

            # Resize preview only
            h, w = frame.shape[:2]

            display_frame = cv2.resize(
                frame,
                (
                    int(w * preview_scale),
                    int(h * preview_scale)
                ),
                interpolation=cv2.INTER_AREA
            )

            # Show preview
            cv2.imshow(
                "ZED Preview",
                display_frame
            )

            frame_count += 1

            if frame_count % 30 == 0:

                print(
                    f"[INFO] Frames recorded: {frame_count}"
                )

            key = cv2.waitKey(1)

            if key == ord('q'):
                break

        else:

            print("[WARN] Grab failed")

except KeyboardInterrupt:

    print("\nStopping...")


# ==========================
# CLEANUP
# ==========================

cv2.destroyAllWindows()

zed.disable_recording()

zed.close()

print("[INFO] Saved:")
print("underwater_custom.svo2")
