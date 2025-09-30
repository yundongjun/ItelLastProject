import cv2

def show_usb_camera():
    # /dev/video0 또는 /dev/video1 과 같은 USB 웹캠을 엽니다.
    # 보통 0부터 시작합니다.
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("USB 웹캠을 열 수 없습니다. 카메라가 연결되어 있는지, 다른 프로그램이 사용하고 있지 않은지 확인하세요.")
        return

    window_name = "USB Camera"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("카메라에서 프레임을 읽어올 수 없습니다.")
            break

        cv2.imshow(window_name, frame)

        # "q" 키를 누르면 종료합니다.
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    show_usb_camera()
