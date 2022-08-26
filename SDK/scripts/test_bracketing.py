import sys
import pyzed.sl as sl
from signal import signal, SIGINT
import cv2

cam = sl.Camera()
exposures = [2, 4, 8, 16, 32]

def handler(signal_received, frame):
    cam.disable_recording()
    cam.close()
    sys.exit(0)

signal(SIGINT, handler)

def main():

    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 15
    init.depth_mode = sl.DEPTH_MODE.NONE

    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit(1)

    #path_output = sys.argv[1]
    #recording_param = sl.RecordingParameters(path_output, sl.SVO_COMPRESSION_MODE.H264)
    # err = cam.enable_recording(recording_param)
    # if err != sl.ERROR_CODE.SUCCESS:
    #     print(repr(status))
    #     exit(1)

    runtime = sl.RuntimeParameters()
    print("SVO is Recording, use Ctrl-C to stop.")
    frames_recorded = 0

    i=0
    while True:
        image = sl.Mat()
        i+=1
        cam.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, exposures[i%len(exposures)])
        if cam.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            cam.retrieve_image(image, sl.VIEW.SIDE_BY_SIDE) # Retrieve the left image
            image_cv = image.get_data()
            if (i % 100 == 0):
                cv2.imwrite(f"./{i}.png", image_cv)


if __name__ == "__main__":
    main()