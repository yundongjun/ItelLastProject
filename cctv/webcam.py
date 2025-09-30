import cv2

# gstreamer 파이프라인을 사용하여 웹캠을 초기화합니다.
# Jetson Nano에서는 성능 향상을 위해 gstreamer를 사용하는 것이 좋습니다.
def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def show_camera():
    # gstreamer 파이프라인으로 VideoCapture 객체를 생성합니다.
    # /dev/video0 장치를 직접 사용하는 대신 gstreamer를 사용합니다.
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

    if cap.isOpened():
        window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
        # Window의 핸들을 얻고, 열려있는 동안 루프를 계속합니다.
        while cv2.getWindowProperty("CSI Camera", 0) >= 0:
            ret_val, img = cap.read()
            if not ret_val:
                break
            cv2.imshow("CSI Camera", img)

            # "q" 키를 누르면 종료합니다.
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("웹캠을 열 수 없습니다.")

if __name__ == "__main__":
    show_camera()
