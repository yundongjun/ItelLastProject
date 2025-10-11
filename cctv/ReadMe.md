### 서버 사용법
## 1. rpi 에서 웹캡 실행
cd /home/pi/mjpeg-streamer/mjpg-streamer-experimnetal

./mjpg_streamer -i "input_uvc.so -d /dev/video0 -r 800x600 -f 15" -o "output_http.so -p 8080 -w ./www"

## 2. rpi 에서 어플 실행 (화재,사람 감지되면 우분투가 신호를준다)
sudo insmod buzzer_device.ko

sudo ./buzzer_app

## 3. 우분투에서 서버 실행
python3 server.py

