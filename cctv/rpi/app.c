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
#define SERVER_PORT 5000      // Ubuntu 연결용
#define LOCAL_UDP_PORT 6000   // Python → C 내부 UDP
#define BUZZER_HOLD_SEC 10.0  // 최소 유지 시간 (초)

int buzzer_fd = -1;
int tcp_client = -1;
int tcp_server = -1;
int udp_sock = -1;
pid_t mjpg_pid = -1;
int buzzer_active = 0;
time_t last_buzzer_on_time = 0;

void cleanup(int sig) {
    printf("\n🧹 Cleaning up...\n");
    if (tcp_client >= 0) close(tcp_client);
    if (tcp_server >= 0) close(tcp_server);
    if (udp_sock >= 0) close(udp_sock);
    if (buzzer_fd >= 0) close(buzzer_fd);

    if (mjpg_pid > 0) {
        kill(mjpg_pid, SIGTERM);
        waitpid(mjpg_pid, NULL, 0);
        printf("📸 mjpg_streamer stopped.\n");
    }
    exit(0);
}

int wait_for_client() {
    printf("💻 Waiting for Ubuntu client on port %d...\n", SERVER_PORT);
    int client = accept(tcp_server, NULL, NULL);
    if (client >= 0)
        printf("✅ Ubuntu connected.\n");
    return client;
}

int main(void) {
    signal(SIGINT, cleanup);
    signal(SIGPIPE, SIG_IGN);  // 💡 클라이언트 끊김 시 종료 방지

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

    // --- TCP 서버 (Ubuntu 연결용) ---
    tcp_server = socket(AF_INET, SOCK_STREAM, 0);
    int reuse = 1;
    setsockopt(tcp_server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in serv = {0};
    serv.sin_family = AF_INET;
    serv.sin_addr.s_addr = INADDR_ANY;
    serv.sin_port = htons(SERVER_PORT);
    bind(tcp_server, (struct sockaddr*)&serv, sizeof(serv));
    listen(tcp_server, 1);

    // --- UDP 리시버 (Python → C, 온도수신) ---
    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in udp_addr = {0};
    udp_addr.sin_family = AF_INET;
    udp_addr.sin_addr.s_addr = INADDR_ANY;
    udp_addr.sin_port = htons(LOCAL_UDP_PORT);
    bind(udp_sock, (struct sockaddr*)&udp_addr, sizeof(udp_addr));
    printf("📡 Listening for temperature data on UDP port %d...\n", LOCAL_UDP_PORT);

    char buf[64];
    char cmd[64];
    double current_temp = 0.0;

    // --- 최초 클라이언트 대기 ---
    tcp_client = wait_for_client();

    while (1) {
        // --- Python → C (UDP) 수신 ---
        ssize_t n = recv(udp_sock, buf, sizeof(buf) - 1, MSG_DONTWAIT);
        if (n > 0) {
            buf[n] = '\0';
            current_temp = atof(buf);
            printf("🌡 Received from Python: %.2f °C\n", current_temp);

            // Ubuntu로 전송 시도
            if (tcp_client >= 0) {
                char msg[64];
                snprintf(msg, sizeof(msg), "TEMP:%.2f\n", current_temp);
                ssize_t s = send(tcp_client, msg, strlen(msg), 0);
                if (s < 0 && (errno == EPIPE || errno == ECONNRESET)) {
                    printf("⚠️ Client disconnected (on send)\n");
                    close(tcp_client);
                    tcp_client = wait_for_client();
                    continue;
                }
            }
        }

        // --- Ubuntu → RPi (버저 제어 명령 수신) ---
        int r = recv(tcp_client, cmd, sizeof(cmd) - 1, MSG_DONTWAIT);
        if (r == 0) {
            printf("🔌 Ubuntu client disconnected.\n");
            close(tcp_client);
            tcp_client = wait_for_client();
            continue;
        } else if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("recv");
            close(tcp_client);
            tcp_client = wait_for_client();
            continue;
        }

        if (r > 0) {
            cmd[r] = '\0';
            // 🔹 모든 개행/공백 제거
            for (int i = 0; cmd[i] != '\0'; i++) {
                if (cmd[i] == '\n' || cmd[i] == '\r' || cmd[i] == ' ')
                    cmd[i] = 0;
            }

            printf("📩 Received from Ubuntu: '%s'\n", cmd);

            if (strcmp(cmd, "1") == 0) {
                if (!buzzer_active) {
                    if (buzzer_fd >= 0) {
                        ssize_t w = write(buzzer_fd, "1", 1);
                        printf("🔔 Buzzer ON (write=%zd)\n", w);
                    }
                    buzzer_active = 1;
                    last_buzzer_on_time = time(NULL);
                } else {
                    printf("⏳ Ignored duplicate ON command (already active)\n");
                }
            }
            else if (strcmp(cmd, "0") == 0) {
                double elapsed = difftime(time(NULL), last_buzzer_on_time);
                if (buzzer_active && elapsed < BUZZER_HOLD_SEC) {
                    printf("⏳ Ignored OFF (only %.1f sec passed)\n", elapsed);
                } else {
                    if (buzzer_fd >= 0) {
                        ssize_t w = write(buzzer_fd, "0", 1);
                        printf("🔕 Buzzer OFF (write=%zd)\n", w);
                    }
                    buzzer_active = 0;
                }
            }
            else {
                printf("⚠️ Unknown command: '%s'\n", cmd);
            }
        }

        // --- mjpg_streamer 프로세스 체크 ---
        int status;
        pid_t result = waitpid(mjpg_pid, &status, WNOHANG);
        if (result == mjpg_pid) {
            printf("⚠️ mjpg_streamer exited unexpectedly.\n");
            mjpg_pid = -1;
        }

        usleep(100000); // 0.1초 슬립
    }

    cleanup(0);
    return 0;
}

