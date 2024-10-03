#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>

#define UART_DEVICE "/dev/ttyS0"  // 사용할 UART 장치
#define LOG_FILE "uart_log.txt"   // 로그를 저장할 파일
#define BUFFER_SIZE 256

int uart_fd;
FILE *log_file;
volatile sig_atomic_t data_received = 0;  // 수신 데이터 플래그
pthread_mutex_t lock;       // 송수신 간 동기화를 위한 뮤텍스
char current_input[BUFFER_SIZE] = {0};  // 현재 입력 중인 메시지

// 터미널 설정을 non-canonical 모드로 변경하는 함수
void enableNonCanonicalMode() 
{
    struct termios tty;
    
    // 현재 터미널 속성을 가져옴
    tcgetattr(STDIN_FILENO, &tty);
    
    // canonical 모드 비활성화
    tty.c_lflag &= ~(ICANON);
    
    // 변경된 속성을 적용
    tcsetattr(STDIN_FILENO, TCSANOW, &tty);
}

// 터미널 설정을 원래대로 복구하는 함수
void restoreCanonicalMode() 
{
    struct termios tty;
    
    // 현재 터미널 속성을 가져옴
    tcgetattr(STDIN_FILENO, &tty);
    
    // canonical 모드 활성화
    tty.c_lflag |= (ICANON | ECHO);
    
    // 변경된 속성을 적용
    tcsetattr(STDIN_FILENO, TCSANOW, &tty);
}

// 시간 가져오기 함수
void get_current_time(char *buffer, size_t buffer_size) 
{
    time_t raw_time;
    struct tm *time_info;

    time(&raw_time);  // 현재 시간 가져오기
    time_info = localtime(&raw_time);

    // 시간을 "[YYYY-MM-DD HH:MM:SS]" 형식으로 변환
    strftime(buffer, buffer_size, "[%Y-%m-%d %H:%M:%S]", time_info);
}

// 수신 thread 함수
void* receive_thread_func() 
{
    char buffer[BUFFER_SIZE];
    int bytes_read;
    char time_str[64];

    while (1) 
    {
        // UART에서 데이터를 읽음 (Non-blocking read)
        bytes_read = read(uart_fd, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) 
        {
            // 수신 buffer 마지막이 enter가 아니면 enter 추가
            if(buffer[bytes_read-1] != 0x0A)
            {
                buffer[bytes_read] = 0x0A;
                bytes_read += 1;
            }
            buffer[bytes_read] = '\0'; // Null-terminate for safe printing

            // 수신 시간 가져오기
            get_current_time(time_str, sizeof(time_str));

            // 뮤텍스 잠금 (동기화)
            pthread_mutex_lock(&lock);

            // 수신 메시지 출력 전에 현재 입력 중인 메시지를 지우기 위해 커서를 앞으로 이동
            printf("\r\033[K");  // 현재 줄을 지움

            // 수신된 메시지 출력
            printf("Received: %s", buffer);

            // 로그 파일에 시간과 함께 기록
            if (log_file) 
            {
                fprintf(log_file, "%s [RECEIVED] %s", time_str, buffer);
                fflush(log_file);  // 즉시 파일에 기록
            }

            printf("Send: ");
            // 입력 중이던 메시지 복원
            if (strlen(current_input) > 0)
            {
                printf("%s", current_input);  // 입력 중이던 메시지를 다시 출력
            }
            fflush(stdout);  // 즉시 화면에 반영

            pthread_mutex_unlock(&lock);  // 뮤텍스 해제
        } 
        else if (bytes_read < 0 && errno != EAGAIN) 
        {
            perror("UART read error");
        }

        usleep(10000);  // CPU 과부하를 방지하기 위해 10ms 대기
    }

    return NULL;
}

// 송신 thread 함수
void* send_thread_func() 
{
    char time_str[64];
    char c;
    int idx;

    printf("UART transmission started. Waiting for input to send...\n");
    printf("Enter a message to send (or type 'exit' to quit)\n");
    while (1) 
    {
        // 값 초기화
        idx = 0;
        c = 0x00;

        // 사용자 입력을 받아 송신
        printf("Send: ");
        while (c != 0x0A)
        {
            c = getchar();
            // 백스페이스 처리 (ASCII 127 혹은 '\b')
            if (c == 127 || c == '\b') 
            {
                // 버퍼에 문자가 있을 때만 지움
                if (idx > 0) 
                {  
                    idx -= 1;
                    current_input[idx] = 0x00;
                    // 터미널에서 문자를 지우는 동작
                    printf("\b\b\b   \b\b\b");  // 커서를 뒤로 이동하고 공백 출력 후 다시 뒤로 이동
                }
            } 
            else 
            {
                // 버퍼 크기 제한
                if (idx < BUFFER_SIZE - 1) 
                {
                    // input 값 입력
                    current_input[idx] = c;
                    idx += 1;
                }
            }
        }

        // 'exit' 입력 시 프로그램 종료
        if (strncmp(current_input, "exit", 4) == 0)
        {
            break;
        }

        // current_input 크기 측정
        size_t buf_len = strlen(current_input);

        // 아무 입력도 없으면 continue
        if (buf_len <= 1)
            continue;

        // 송신 시간 가져오기
        get_current_time(time_str, sizeof(time_str));

        // UART로 메시지 송신
        int bytes_written = write(uart_fd, current_input, buf_len);
        if (bytes_written < 0) 
        {
            perror("UART send error");
        } 
        else
        {
            // 뮤텍스 잠금 (동기화)
            pthread_mutex_lock(&lock);

            // 로그 파일에 시간과 함께 기록
            if (log_file) 
            {
                fprintf(log_file, "%s [SENT] %s", time_str, current_input);
                fflush(log_file);  // 즉시 파일에 기록
            }

            pthread_mutex_unlock(&lock);  // 뮤텍스 해제
        }

        // 메시지 송신 후 current_input을 초기화
        memset(current_input, 0, sizeof(current_input));

        usleep(10000);  // CPU 과부하를 방지하기 위해 10ms 대기
    }

    return NULL;
}

int main() 
{
    struct termios uart_options;
    pthread_t send_thread, receive_thread;

    // UART 장치 열기
    uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart_fd < 0) 
    {
        perror("Unable to open UART");
        return 1;
    }

    // UART 설정
    tcgetattr(uart_fd, &uart_options);
    cfsetispeed(&uart_options, B115200); // Baud rate 115200으로 설정
    cfsetospeed(&uart_options, B115200);
    uart_options.c_cflag |= (CLOCAL | CREAD); // UART 수신 및 송신 활성화
    uart_options.c_cflag &= ~PARENB;  // No parity
    uart_options.c_cflag &= ~CSTOPB;  // 1 stop bit
    uart_options.c_cflag &= ~CSIZE;
    uart_options.c_cflag |= CS8;      // 8 data bits
    uart_options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Non-canonical mode
    uart_options.c_iflag &= ~(IXON | IXOFF | IXANY);         // Disable flow control
    tcsetattr(uart_fd, TCSANOW, &uart_options);

    // non-canonical 모드로 변경
    enableNonCanonicalMode();

    // 로그 파일 열기
    log_file = fopen(LOG_FILE, "a");
    if (!log_file)
    {
        perror("Unable to open log file");
        close(uart_fd);
        return 1;
    }

    // 뮤텍스 초기화
    pthread_mutex_init(&lock, NULL);

    // 수신 및 송신 쓰레드 생성
    pthread_create(&send_thread, NULL, send_thread_func, NULL);
    pthread_create(&receive_thread, NULL, receive_thread_func, NULL);

    // 송신 쓰레드가 종료될 때까지 대기
    pthread_join(send_thread, NULL);

    // 수신 쓰레드는 무한 루프를 돌기 때문에, 프로그램이 종료되면 UART와 로그 파일을 닫음
    fclose(log_file);
    close(uart_fd);

    // 뮤텍스 제거
    pthread_mutex_destroy(&lock);

    // 터미널을 원래 모드로 복구
    restoreCanonicalMode();
    return 0;
}