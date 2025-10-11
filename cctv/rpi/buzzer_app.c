#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <signal.h>

#define PORT 5000
#define BUZZER_DEV "/dev/buzzer"

int main(void) {
    int server_fd, client_fd = -1;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    char buf[32];
    int buzzer_fd;
    int opt = 1;

    // SIGPIPE 무시(예: 끊긴 소켓에 잘못 쓰기로 인한 종료 방지)
    signal(SIGPIPE, SIG_IGN);

    // /dev/buzzer 열기(계속 유지)
    buzzer_fd = open(BUZZER_DEV, O_WRONLY);
    if (buzzer_fd < 0) {
        perror("open buzzer device");
        return 1;
    }

    // 서버 소켓 생성
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket");
        close(buzzer_fd);
        return 1;
    }

    // TIME_WAIT 재사용 허용
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#ifdef SO_REUSEPORT
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

    // 바인드
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind");
        close(server_fd);
        close(buzzer_fd);
        return 1;
    }

    // 리슨
    if (listen(server_fd, 4) < 0) {
        perror("listen");
        close(server_fd);
        close(buzzer_fd);
        return 1;
    }

    printf("Buzzer server listening on port %d...\n", PORT);

    // ==== 메인 accept 루프: 끊겨도 계속 대기 ====
    for (;;) {
        client_len = sizeof(client_addr);
        client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
        if (client_fd < 0) {
            perror("accept");
            continue; // 다음 연결 대기
        }
        printf("Client connected.\n");

        // === 클라이언트 처리 루프 ===
        for (;;) {
            ssize_t n = read(client_fd, buf, sizeof(buf) - 1);
            if (n <= 0) {
                // 0: 정상 종료, <0: 에러 — 어떤 경우든 이 클라이언트만 닫고 다음 연결로
                if (n < 0) perror("read");
                printf("Client disconnected.\n");
                close(client_fd);
                client_fd = -1;
                break; // 내부 루프 종료 → 다시 accept로
            }
            buf[n] = '\0';
            printf("Received: %s\n", buf);

            // /dev/buzzer에 쓰기
            if (write(buzzer_fd, buf, (size_t)n) < 0) {
                perror("write buzzer");
            }
        }
        // 내부 루프를 빠져나오면 자동으로 다음 클라이언트 accept
    }

    // (사실 for(;;)에서 빠질 일은 없지만, 정리 코드)
    if (client_fd >= 0) close(client_fd);
    close(server_fd);
    close(buzzer_fd);
    return 0;
}

