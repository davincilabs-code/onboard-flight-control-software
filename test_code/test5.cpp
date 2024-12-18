#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>          // 메모리 잠금
#include <linux/i2c-dev.h>
#include <cstdint>
#include <cmath>               // 수학 함수 사용 (atan2, sqrt, M_PI)
#include <Eigen/Dense>         // Eigen 라이브러리
#include <pthread.h>           // POSIX 스레드
#include <time.h>              // 고해상도 타이머
#include "rc_input.h"          // RC 입력 헤더 파일
#include "imu_sensor.h"        // IMU 센서 헤더 포함
#include "imu_calibration.h"   // IMU 캘리브레이션 헤더 파일
#include <termios.h>           // B115200 설정
#include <algorithm>           // std::clamp 함수 사용
#include <chrono>              // 시간 측정을 위한 라이브러리
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <sched.h>              // 리얼타임 우선순위 확인 및 설정

#define PCA9685_ADDR 0x40      // PCA9685 I2C 주소
#define MODE1 0x00             // 모드1 레지스터
#define PRESCALE 0xFE          // 프리스케일 레지스터
#define LED0_ON_L 0x06         // 첫 번째 채널 ON 낮은 바이트 레지스터
#define LED0_OFF_L 0x08        // 첫 번째 채널 OFF 낮은 바이트 레지스터

const int RC_MIN = 172;
const int RC_MAX = 1811;
const int RC_MID = 991;
const int PWM_MIN = 210;
const int PWM_MAX = 405;
const int MAX_ADJUSTMENT = 10; // 각 제어 입력의 최대 PWM 조정 값
const int I2C_RETRY_LIMIT = 3; // I2C 오류 시 재시도 횟수
const int SAFE_PWM = PWM_MIN;  // 초기화 및 안전한 PWM 값
const float TOLERANCE_ROLL = 1;   // 롤 허용 오차 (0.45도)
const float TOLERANCE_PITCH = 1;  // 피치 허용 오차 (0.45도)

void setRealtimePriority(pthread_t thread, int priority) {
    struct sched_param param;
    param.sched_priority = priority; // 우선순위 1~99
    if (pthread_setschedparam(thread, SCHED_FIFO, &param) != 0) {
        perror("Failed to set real-time priority");
    }
}
class PCA9685 {
public:
    PCA9685(int address = PCA9685_ADDR) {
        char filename[20];
        snprintf(filename, 19, "/dev/i2c-1");
        fd = open(filename, O_RDWR);
        if (fd < 0) {
            std::cerr << "Failed to open the i2c bus" << std::endl;
            exit(1);
        }
        if (ioctl(fd, I2C_SLAVE, address) < 0) {
            std::cerr << "Failed to acquire bus access and/or talk to slave" << std::endl;
            exit(1);
        }
        reset();
        setPWMFreq(50);  // Set frequency to 50Hz for motor control
        // setPWMFreq(100);
        initializeMotors(); // 모든 모터를 초기 안전 PWM 값으로 설정
    }

    ~PCA9685() {
        stopAllMotors(); // 종료 시 모든 모터를 정지
        if (fd >= 0) {
            close(fd);
        }
    }

    void setPWM(int channel, int on, int off) {
        writeRegister(LED0_ON_L + 4 * channel, on & 0xFF);
        writeRegister(LED0_ON_L + 4 * channel + 1, on >> 8);
        writeRegister(LED0_OFF_L + 4 * channel, off & 0xFF);
        writeRegister(LED0_OFF_L + 4 * channel + 1, off >> 8);
    }

    void setMotorSpeed(int channel, int pwm_value) {
        if (pwm_value < PWM_MIN || pwm_value > PWM_MAX) {
            std::cerr << "PWM value out of range (" << PWM_MIN << "-" << PWM_MAX << ")" << std::endl;
            return;
        }
        setPWM(channel, 0, pwm_value);
    }

    void setAllMotorsSpeeds(const int pwm_values[4]) {
        uint8_t buffer[17];
        buffer[0] = LED0_ON_L;

        for (int i = 0; i < 4; i++) {
            int pwm = std::clamp(pwm_values[i], PWM_MIN, PWM_MAX);
            buffer[1 + i * 4] = 0;
            buffer[2 + i * 4] = 0;
            buffer[3 + i * 4] = pwm & 0xFF;
            buffer[4 + i * 4] = pwm >> 8;
        }

        asyncWrite(buffer, sizeof(buffer));
    }

private:
    int fd;

    void reset() {
        writeRegister(MODE1, 0x00);
    }

    void setPWMFreq(int freq) {
        uint8_t prescale = static_cast<uint8_t>(25000000.0 / (4096.0 * freq) - 1.0);
        uint8_t oldmode = readRegister(MODE1);
        uint8_t newmode = (oldmode & 0x7F) | 0x10;
        writeRegister(MODE1, newmode);
        writeRegister(PRESCALE, prescale);
        writeRegister(MODE1, oldmode);
        usleep(5000);
        writeRegister(MODE1, oldmode | 0xA1);
    }

    void writeRegister(uint8_t reg, uint8_t value) {
        uint8_t buffer[2] = {reg, value};
        int retries = 0;
        while (write(fd, buffer, 2) != 2) {
            if (++retries >= I2C_RETRY_LIMIT) {
                std::cerr << "Failed to write to the i2c bus after retries" << std::endl;
                exit(1);
            }
            usleep(1000); // 1ms 대기 후 재시도
        }
    }

    uint8_t readRegister(uint8_t reg) {
        int retries = 0;
        while (write(fd, &reg, 1) != 1) {
            if (++retries >= I2C_RETRY_LIMIT) {
                std::cerr << "Failed to write to the i2c bus after retries" << std::endl;
                exit(1);
            }
            usleep(1000);
        }
        uint8_t value;
        if (read(fd, &value, 1) != 1) {
            std::cerr << "Failed to read from the i2c bus" << std::endl;
            exit(1);
        }
        return value;
    }

    void initializeMotors() {
        for (int i = 0; i < 4; ++i) {
            setMotorSpeed(i, SAFE_PWM);
        }
    }

    void stopAllMotors() {
        for (int i = 0; i < 4; ++i) {
            setMotorSpeed(i, SAFE_PWM);
        }
        std::cout << "All motors stopped safely." << std::endl;
    }
};

struct PIDController {
    float kp, ki, kd;
    float prev_error;
    float integral;
    float integral_limit;
    float output_limit;
    float feedforward;
    float filtered_derivative; 
    float alpha;               

    // outlimit 설정 400->10로 기본 세팅
    PIDController(float p, float i, float d, float ff = 0.0f, float i_limit = 10.0f, float out_limit = 10.0f, float filter_alpha = 0.1f)
        : kp(p), ki(i), kd(d), feedforward(ff), prev_error(0.0f), integral(0.0f),
          integral_limit(i_limit), output_limit(out_limit), filtered_derivative(0.0f), alpha(filter_alpha) {}

    void reset() {
        prev_error = 0.0f;
        integral = 0.0f;
        filtered_derivative = 0.0f;
    }

    float calculate(float setpoint, float measurement, float dt) {
        if (dt <= 0.0f) {
            // dt가 0 이하일 경우, 계산을 중단
            return 0.0f;
        }

        // 오차 계산
        float error = setpoint - measurement;

        // 비례 항
        float pTerm = kp * error;

        // 적분 항 (적분 제한 적용)
        integral += error * dt;
        integral = std::clamp(integral, -integral_limit, integral_limit);
        float iTerm = ki * integral;

        // 미분 항 (필터링 적용)
        float derivative = (error - prev_error) / dt;
        filtered_derivative = alpha * derivative + (1.0f - alpha) * filtered_derivative;
        float dTerm = kd * filtered_derivative;

        // 이전 오차 업데이트
        prev_error = error;

        // PID 출력 (피드포워드 포함)
        float output = feedforward * setpoint + pTerm + iTerm + dTerm;

        // 출력 제한
        output = std::clamp(output, -output_limit, output_limit);

        return output;
    }
};

// 스로틀 값을 0.0 ~ 1.0 범위로 매핑하는 함수
double mapThrottle(int value) {
    if (value <= RC_MIN) return 0.0;
    if (value >= RC_MAX) return 1.0;
    return static_cast<double>(value - RC_MIN) / (RC_MAX - RC_MIN);
}

double mapControlInput(int value) {
    if (value < RC_MIN || value > RC_MAX) {
        return 0.0;
    }
    if (value < RC_MID) return static_cast<double>(value - RC_MID) / (RC_MID - RC_MIN);
    if (value > RC_MID) return static_cast<double>(value - RC_MID) / (RC_MAX - RC_MID);
    return 0.0;
}

int computeThrottlePWM(double throttle_normalized) {
    return static_cast<int>(PWM_MIN + throttle_normalized * (PWM_MAX - PWM_MIN));
}

int computeAdjustment(double control_normalized) {
    return static_cast<int>(control_normalized * MAX_ADJUSTMENT);
}

int clamp(int value, int min_value, int max_value) {
    return value < min_value ? min_value : (value > max_value ? max_value : value);
}

// IMU 데이터와 mutex 정의
IMUData imuData;
std::mutex imuQueueMutex;
std::queue<IMUData> imuDataQueue;
std::condition_variable imuDataCv;

void *sendIMURequestThread(void *arg) {
    while (true) {
        sendIMURequest();  // IMU 데이터 요청 전송
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 요청 주기 조정
    }
    return nullptr;
}

// IMU 스레드 함수
void *imuThread(void *arg) {
    initIMU("/dev/ttyUSB0", B115200);
    IMUCalibrationData calibrationData = calibrateIMU();
    float offsetGyroZ = calibrationData.offsetGyroZ;

    while (true) {
        IMUData localIMUData = readIMU();
        localIMUData.gyroZ -= offsetGyroZ; // 보정된 자이로 Z값

        // IMU 데이터를 큐에 추가
        {
            std::lock_guard<std::mutex> lock(imuQueueMutex);
            imuDataQueue.push(localIMUData);
        }
        imuDataCv.notify_one(); // 데이터 추가 알림
    }
    return nullptr;
}

// PID 계산 스레드 함수
void *controlLoop(void *arg) {
    PCA9685 pca9685;
    initRC("/dev/ttyAMA0", B115200);

    PIDController rollPID(1.5f, 0.0f, 1.0f);
    PIDController pitchPID(2.0f, 0.5f, 0.2f);
    PIDController yawPID(1.2f, 0.5f, 0.5f);

    float roll_com = 0;
    float pitch_com = 0;
    auto previousTime = std::chrono::steady_clock::now();

    while (true) {
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<float> elapsed = currentTime - previousTime;
        float dt = elapsed.count();
        previousTime = currentTime;

        IMUData localIMUData;
        {
            std::lock_guard<std::mutex> lock(imuMutex);
            localIMUData = imuData;
        }

        int throttle_value = readRCChannel(3);
        int aileron_value = readRCChannel(1);
        int elevator_value = readRCChannel(2);
        int rudder_value = readRCChannel(4);

        double throttle_normalized = mapThrottle(throttle_value);
        double aileron_normalized = mapControlInput(aileron_value);
        double elevator_normalized = mapControlInput(elevator_value);
        double rudder_normalized = mapControlInput(rudder_value);

        int roll_adj = 0;
        int pitch_adj = 0;
        int yaw_adj = 0;
        if (std::abs(roll_com - localIMUData.roll_angle) > TOLERANCE_ROLL) {
            roll_adj = rollPID.calculate(roll_com, localIMUData.roll_angle, dt);
        }
        if (std::abs(pitch_com - localIMUData.pitch_angle) > TOLERANCE_PITCH) {
            pitch_adj = pitchPID.calculate(pitch_com, localIMUData.pitch_angle, dt);
        }
        // int yaw_adj = yawPID.calculate(rudder_normalized, correctedGyroZ, dt);
        int throttle_PWM = computeThrottlePWM(throttle_normalized);
        int motor1_PWM, motor2_PWM, motor3_PWM, motor4_PWM;

        
        if (throttle_PWM <= PWM_MIN) {
            pca9685.setMotorSpeed(0, PWM_MIN);
            pca9685.setMotorSpeed(1, PWM_MIN);
            pca9685.setMotorSpeed(2, PWM_MIN);
            pca9685.setMotorSpeed(3, PWM_MIN);
            continue;
        } else {
            int aileron_adj_total = computeAdjustment(aileron_normalized) + roll_adj;
            int elevator_adj_total = computeAdjustment(elevator_normalized) + pitch_adj;

            int motor1_adj = -aileron_adj_total + elevator_adj_total + yaw_adj;
            int motor2_adj = aileron_adj_total - elevator_adj_total - yaw_adj;
            int motor3_adj = aileron_adj_total + elevator_adj_total + yaw_adj;
            int motor4_adj = -aileron_adj_total - elevator_adj_total - yaw_adj;

            motor1_PWM = clamp(throttle_PWM + motor1_adj, PWM_MIN, PWM_MAX);
            motor2_PWM = clamp(throttle_PWM + motor2_adj, PWM_MIN, PWM_MAX);
            motor3_PWM = clamp(throttle_PWM + motor3_adj, PWM_MIN, PWM_MAX);
            motor4_PWM = clamp(throttle_PWM + motor4_adj, PWM_MIN, PWM_MAX);
        }

        int motor_pwm[4] = {motor1_PWM, motor2_PWM, motor3_PWM, motor4_PWM};

        pca9685.setMotorSpeed(0, motor1_PWM);
        pca9685.setMotorSpeed(1, motor2_PWM);
        pca9685.setMotorSpeed(2, motor3_PWM);
        pca9685.setMotorSpeed(3, motor4_PWM);
    }
    return nullptr;
}

int main() {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cerr << "Failed to lock memory!" << std::endl;
        return -1;
    }

    pthread_t imuThreadHandle, controlThreadHandle;

    if (pthread_create(&imuThreadHandle, nullptr, imuThread, nullptr) != 0) {
        std::cerr << "Failed to create IMU thread!" << std::endl;
        return -1;
    }

    if (pthread_create(&controlThreadHandle, nullptr, controlLoop, nullptr) != 0) {
        std::cerr << "Failed to create Control thread!" << std::endl;
        return -1;
    }

    pthread_join(imuThreadHandle, nullptr);
    pthread_join(controlThreadHandle, nullptr);

    return 0;
}
