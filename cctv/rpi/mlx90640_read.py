import time, board, busio, socket
import adafruit_mlx90640

# --- UDP 설정 (app.c 로 전송) ---
UDP_IP = "127.0.0.1"
UDP_PORT = 6000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- MLX90640 초기화 ---
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ

frame = [0] * 768  # 32x24 pixels

print("✅ MLX90640 ready, sending temperature to app.c every 5s...")

while True:
    try:
        mlx.getFrame(frame)
        tmin = min(frame)
        tmax = max(frame)
        print(f"🌡 Min: {tmin:.1f}°C, Max: {tmax:.1f}°C")

        # --- UDP 전송 ---
        msg = f"{tmax:.2f}".encode()
        sock.sendto(msg, (UDP_IP, UDP_PORT))

        time.sleep(5)
    except Exception as e:
        print("Error:", e)
        time.sleep(1)

