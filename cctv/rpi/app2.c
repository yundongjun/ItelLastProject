#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <signal.h>
#include <sys/wait.h>
#include <time.h>
#include <errno.h>

#define BUZZER_DEV "/dev/buzzer"
#define LOCAL_UDP_PORT 6000       // Python → C (온도 수신)
#define UBUNTU_TCP_PORT 5000      // Ubuntu ← C (온도 전송)
#define BUZZER_HOLD_SEC 10.0      // 최소 유지 시간 (초)
#define TEMP_THRESHOLD 60.0       // 온도 임계값 (℃)

int buzzer_fd = -1;
int udp_sock = -1;
int tcp_server = -1;
int tcp_client = -1;
pid_t mjpg_pid = -1;
int buzzer_active = 0;
time_t last_buzzer_on_time = 0;
double current_temp = 0.0;

// ------------------------------------------------------------
// 종료 및 정리
// ------------------------------------------------------------
void cleanup(int sig) {
    printf("\n🧹 Cleaning up...\n");
    if (udp_sock >= 0) close(udp_sock);
    if (tcp_client >= 0) close(tcp_client);
    if (tcp_server >= 0) close(tcp_server);
    if (buzzer_fd >= 0) close(buzzer_fd);

    if (mjpg_pid > 0) {
        kill(mjpg_pid, SIGTERM);
        waitpid(mjpg_pid, NULL, 0);
        printf("📸 mjpg_streamer stopped.\n");
    }
    exit(0);
}

// ------------------------------------------------------------
// Ubuntu 연결 대기
// ------------------------------------------------------------
int wait_for_ubuntu() {
    struct sockaddr_in cli_addr;
    socklen_t cli_len = sizeof(cli_addr);
    printf("💻 Waiting for Ubuntu client on port %d...\n", UBUNTU_TCP_PORT);

    int client = accept(tcp_server, (struct sockaddr*)&cli_addr, &cli_len);
    if (client >= 0) {
        printf("✅ Ubuntu connected: %s\n", inet_ntoa(cli_addr.sin_addr));
    } else {
        perror("accept");
    }
    return client;
}

// ------------------------------------------------------------
// 메인 함수
// ------------------------------------------------------------
int main(void) {
    signal(SIGINT, cleanup);
    signal(SIGPIPE, SIG_IGN); // 클라이언트 끊김 시 종료 방지

    // --- /dev/buzzer 열기 ---
    buzzer_fd = open(BUZZER_DEV, O_WRONLY);
    if (buzzer_fd < 0) {
        perror("open buzzer");
        return 1;
    }

    // --- mjpg_streamer 실행 ---
    mjpg_pid = fork();
    if (mjpg_pid == 0) {
        execlp("mjpg_streamer", "mjpg_streamer",
               "-i", "input_uvc.so -d /dev/video0 -r 640x480 -f 10",
               "-o", "output_http.so -p 8080 -w /usr/local/www",
               NULL);
        perror("execlp mjpg_streamer");
        exit(1);
    } else if (mjpg_pid > 0) {
        printf("📸 mjpg_streamer started (PID=%d)\n", mjpg_pid);
        sleep(2);
    }

    // --- UDP 리시버 (Python → RPi 온도 수신) ---
    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in udp_addr = {0};
    udp_addr.sin_family = AF_INET;
    udp_addr.sin_addr.s_addr = INADDR_ANY;
    udp_addr.sin_port = htons(LOCAL_UDP_PORT);

    if (bind(udp_sock, (struct sockaddr*)&udp_addr, sizeof(udp_addr)) < 0) {
        perror("bind udp_sock");
        cleanup(0);
    }
    printf("📡 Listening for temperature data on UDP port %d...\n", LOCAL_UDP_PORT);

    // --- TCP 서버 (Ubuntu 연결 대기) ---
    tcp_server = socket(AF_INET, SOCK_STREAM, 0);
    int reuse = 1;
    setsockopt(tcp_server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in serv = {0};
    serv.sin_family = AF_INET;
    serv.sin_addr.s_addr = INADDR_ANY;
    serv.sin_port = htons(UBUNTU_TCP_PORT);

    if (bind(tcp_server, (struct sockaddr*)&serv, sizeof(serv)) < 0) {
        perror("bind tcp_server");
        cleanup(0);
    }

    if (listen(tcp_server, 1) < 0) {
        perror("listen");
        cleanup(0);
    }
    printf("📶 TCP server started on port %d\n", UBUNTU_TCP_PORT);

    tcp_client = wait_for_ubuntu();

    char buf[64];
    time_t last_send_time = 0;

    // ========================================================
    // 🔁 메인 루프
    // ========================================================
    while (1) {
        // --- UDP 수신 (Python 온도) ---
        ssize_t n = recv(udp_sock, buf, sizeof(buf) - 1, MSG_DONTWAIT);
        if (n > 0) {
            buf[n] = '\0';
            current_temp = atof(buf);
            printf("🌡 Received: %.2f °C\n", current_temp);

            time_t now = time(NULL);

            // 🔥 버저 제어
            if (current_temp >= TEMP_THRESHOLD) {
                if (!buzzer_active) {
                    write(buzzer_fd, "1", 1);
                    buzzer_active = 1;
                    last_buzzer_on_time = now;
                    printf("🔔 Buzzer ON (TEMP=%.2f°C)\n", current_temp);
                }
            } else {
                if (buzzer_active) {
                    double elapsed = difftime(now, last_buzzer_on_time);
                    if (elapsed >= BUZZER_HOLD_SEC) {
                        write(buzzer_fd, "0", 1);
                        buzzer_active = 0;
                        printf("🔕 Buzzer OFF after %.1f sec\n", elapsed);
                    }
                }
            }

            // --- Ubuntu로 TEMP 값 전송 ---
            if (tcp_client >= 0 && difftime(now, last_send_time) >= 1.0) {
                char msg[64];
                snprintf(msg, sizeof(msg), "TEMP:%.2f\n", current_temp);
                ssize_t sent = send(tcp_client, msg, strlen(msg), 0);

                if (sent < 0 && (errno == EPIPE || errno == ECONNRESET)) {
                    printf("⚠️ Ubuntu disconnected, waiting...\n");
                    close(tcp_client);
                    tcp_client = wait_for_ubuntu(); // 재접속 대기
                } else if (sent > 0) {
                    printf("📤 Sent to Ubuntu: %s", msg);
                }
                last_send_time = now;
            }
        }

        // --- mjpg_streamer 프로세스 상태 확인 ---
        int status;
        pid_t result = waitpid(mjpg_pid, &status, WNOHANG);
        if (result == mjpg_pid) {
            printf("⚠️ mjpg_streamer exited unexpectedly.\n");
            mjpg_pid = -1;
        }

        usleep(100000); // 0.1초 주기
    }

    cleanup(0);
    return 0;
}

