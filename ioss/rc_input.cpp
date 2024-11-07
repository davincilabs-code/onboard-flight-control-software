#include "rc_input.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <chrono>
#include <deque>
#include <thread>
#include <vector>

#define SBUS_FRAME_SIZE 35
#define START_BYTE 0x0F

static int serial_port;
static uint16_t channels[16];           // 16채널 값을 저장할 배열
static std::deque<uint8_t> data_buffer; // 최신 데이터를 저장할 버퍼

// 시리얼 포트 설정 함수
static int configureSerial(const std::string& port, int baudrate) {
    serial_port = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port == -1) {
        perror("Failed to open serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(serial_port, &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    tcsetattr(serial_port, TCSANOW, &options);

    return serial_port;
}

// RC 입력 초기화 함수
void initRC(const std::string& port, int baudRate) {
    // 올바르게 초기화되지 않았을 경우 반복적으로 시도
    while (true) {
        if (configureSerial(port, baudRate) == -1) {
            std::cerr << "Failed to initialize RC input. Retrying..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1)); // 1초 대기 후 재시도
            continue;
        }
        break;
    }
}

// 최신 RC 채널 값을 읽고 업데이트하는 함수
int readRCChannel(int channel) {
    if (channel < 1 || channel > 16) {
        std::cerr << "Invalid channel number: " << channel << std::endl;
        return -1;
    }

    // 시리얼 포트에서 데이터 읽어 버퍼에 추가
    uint8_t byte;
    while (read(serial_port, &byte, 1) > 0) {
        data_buffer.push_back(byte);

        // 오래된 데이터를 삭제하여 버퍼 크기를 제한
        if (data_buffer.size() > SBUS_FRAME_SIZE * 10) {
            data_buffer.pop_front();
        }
    }

    // 버퍼에서 최신 프레임을 찾아 데이터 갱신
    while (data_buffer.size() >= SBUS_FRAME_SIZE) {
        // 버퍼에서 프레임 추출
        std::vector<uint8_t> frame(data_buffer.begin(), data_buffer.begin() + SBUS_FRAME_SIZE);

        // 시작 바이트 확인
        if (frame[0] != START_BYTE) {
            data_buffer.pop_front();
            continue;
        }

        // 체크섬 검증
        uint8_t xor_checksum = 0;
        for (int i = 1; i < SBUS_FRAME_SIZE - 1; ++i) {
            xor_checksum ^= frame[i];
        }

        if (xor_checksum != frame[SBUS_FRAME_SIZE - 1]) {
            data_buffer.pop_front(); // 잘못된 프레임을 버림
            continue;
        }

        // 유효한 프레임이면 채널 데이터 업데이트
        for (int i = 0; i < 16; ++i) {
            channels[i] = (frame[1 + i * 2] << 8) | frame[2 + i * 2];
        }

        // 프레임을 버퍼에서 제거
        data_buffer.erase(data_buffer.begin(), data_buffer.begin() + SBUS_FRAME_SIZE);
        break;
    }

    // 요청된 채널 값을 반환
    return channels[channel - 1];
}

// int main() {
//     const std::string port = "/dev/ttyAMA0";
//     const int baudRate = B115200;

//     while (true) {
//         try {
//             initRC(port, baudRate);

//             while (true) {
//                 std::cout << "\r";  // 커서를 줄의 처음으로 이동

//                 for (int channel = 1; channel <= 5; ++channel) {
//                     int value = readRCChannel(channel);
//                     if (value != -1) {
//                         std::cout << "Channel " << channel << ": " << value << " ";
//                     } else {
//                         std::cerr << "Error reading channel " << channel << " ";
//                     }
//                 }

//                 std::cout << std::flush;  // 출력 즉시 반영
//                 std::this_thread::sleep_for(std::chrono::milliseconds(100));
//             }
//         } catch (const std::exception& e) {
//             std::cerr << "Exception during initialization: " << e.what() << ". Retrying..." << std::endl;
//             std::this_thread::sleep_for(std::chrono::seconds(1)); // 1초 대기 후 재시도
//         }
//     }

//     return 0;
// }

// #include <fcntl.h>
// #include <termios.h>
// #include <unistd.h>
// #include <iostream>
// #include <cstdint>
// #include <cstring>
// #include <iomanip>

// #define SERIAL_PORT "/dev/ttyAMA0"
// #define BAUDRATE B115200
// #define SBUS_FRAME_SIZE 35
// #define START_BYTE 0x0F

// int main() {
//     int serial_port = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
//     if (serial_port == -1) {
//         std::cerr << "Error opening serial port: " << SERIAL_PORT << std::endl;
//         return 1;
//     }

//     termios tty;
//     memset(&tty, 0, sizeof(tty));

//     if (tcgetattr(serial_port, &tty) != 0) {
//         std::cerr << "Error getting terminal attributes." << std::endl;
//         close(serial_port);
//         return 1;
//     }

//     cfsetispeed(&tty, BAUDRATE);
//     cfsetospeed(&tty, BAUDRATE);

//     tty.c_cflag &= ~PARENB;
//     tty.c_cflag &= ~CSTOPB;
//     tty.c_cflag &= ~CSIZE;
//     tty.c_cflag |= CS8;

//     tty.c_cflag &= ~CRTSCTS;
//     tty.c_cflag |= CREAD | CLOCAL;
//     tty.c_lflag &= ~ICANON;
//     tty.c_lflag &= ~(ECHO | ECHOE | ISIG);

//     tty.c_iflag &= ~(IXON | IXOFF | IXANY);
//     tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP);
//     tty.c_oflag &= ~OPOST;

//     tty.c_cc[VMIN] = SBUS_FRAME_SIZE;
//     tty.c_cc[VTIME] = 1;

//     if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
//         std::cerr << "Error setting terminal attributes." << std::endl;
//         close(serial_port);
//         return 1;
//     }

//     uint8_t sbus_data[SBUS_FRAME_SIZE];

//     while (true) {
//         int num_bytes = read(serial_port, &sbus_data, SBUS_FRAME_SIZE);
//         if (num_bytes == SBUS_FRAME_SIZE && sbus_data[0] == START_BYTE) {
//             uint8_t xor_checksum = 0;
//             for (int i = 1; i < SBUS_FRAME_SIZE - 1; ++i) {
//                 xor_checksum ^= sbus_data[i];
//             }

//             if (xor_checksum != sbus_data[SBUS_FRAME_SIZE - 1]) {
//                 std::cerr << "Checksum error. Discarding frame." << std::endl;
//                 continue;
//             }

//             uint16_t channels[16];
//             for (int i = 0; i < 16; ++i) {
//                 channels[i] = (sbus_data[1 + i * 2] << 8) | sbus_data[2 + i * 2];
//             }

//             // Print channels 1 to 5 on the same line, updating in place
//             std::cout << "\r";
//             for (int i = 0; i < 5; i++) {
//                 std::cout << "Channel " << (i + 1) << ": " << std::setw(4) << channels[i] << " ";
//             }
//             std::cout << std::flush;

//             // Handle flags (if needed for other purposes)
//             uint8_t flags = sbus_data[33];
//             bool ch17 = flags & 0x80;
//             bool ch18 = flags & 0x40;
//             bool frame_lost = flags & 0x20;
//             bool failsafe = flags & 0x10;
//         }
//     }

//     close(serial_port);
//     return 0;
// }