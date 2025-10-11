#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define PORT 5000
#define BUZZER_DEV "/dev/buzzer"

int main() {
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    char buf[32];
    int buzzer_fd;

    // /dev/buzzer 열기
    buzzer_fd = open(BUZZER_DEV, O_WRONLY);
    if (buzzer_fd < 0) {
        perror("open buzzer device");
        return 1;
    }

    // 소켓 생성
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket");
        return 1;
    }

    // 바인드
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind");
        return 1;
    }

    // 리슨
    if (listen(server_fd, 1) < 0) {
        perror("listen");
        return 1;
    }

    printf("Buzzer server listening on port %d...\n", PORT);

    // 클라이언트 연결 대기
    client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
    if (client_fd < 0) {
        perror("accept");
        return 1;
    }
    printf("Client connected.\n");

    // 메시지 수신 및 buzzer 제어
    while (1) {
        int n = read(client_fd, buf, sizeof(buf) - 1);
        if (n <= 0) {
            printf("Client disconnected.\n");
            break;
        }
        buf[n] = '\0';
        printf("Received: %s\n", buf);

        // /dev/buzzer에 쓰기
        if (write(buzzer_fd, buf, strlen(buf)) < 0) {
            perror("write buzzer");
        }
    }

    close(client_fd);
    close(server_fd);
    close(buzzer_fd);
    return 0;
}

