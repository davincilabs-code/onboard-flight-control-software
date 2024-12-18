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
#include <mutex>

#define PCA9685_ADDR 0x40      // PCA9685 I2C 주소
#define MODE1 0x00             // 모드1 레지스터
#define PRESCALE 0xFE          // 프리스케일 레지스터
#define LED0_ON_L 0x06         // 첫 번째 채널 ON 낮은 바이트 레지스터
#define LED0_OFF_L 0x08        // 첫 번째 채널 OFF 낮은 바이트 레지스터

const int RC_MIN = 172;
const int RC_MAX = 1811;
const int RC_MID = 991;
// const int PWM_MIN = 210; // 50Hz 기준
// const int PWM_MAX = 405; // 50Hz 기준
const int PWM_MIN = 1680;  // 400Hz 기준
const int PWM_MAX = 3240;  // 400Hz 기준
const int MAX_ADJUSTMENT = 10; // 각 제어 입력의 최대 PWM 조정 값
const int I2C_RETRY_LIMIT = 3; // I2C 오류 시 재시도 횟수
const int SAFE_PWM = PWM_MIN;  // 초기화 및 안전한 PWM 값
const int LOOP_DELAY_US = 26000; // 주기적인 대기 시간 (10ms)
const float MAX_ANGLE = 90.0f;           // 최대 각도 (예시)
const float TOLERANCE_ROLL = 0.05f * MAX_ANGLE;   // 롤 허용 오차 (0.45도)
const float TOLERANCE_PITCH = 0.01f * MAX_ANGLE;  // 피치 허용 오차 (0.45도)

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
        // setPWMFreq(50);

        // setPWMFreq(100);  // Set frequency to 100Hz for motor control
        setPWMFreq(400);  // Set frequency to 400Hz for motor control
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
    
    void setAllMotorsSpeed(const std::vector<int>& pwm_values) {
        if (pwm_values.size() > 16) { // PCA9685는 최대 16 채널까지 지원
            std::cerr << "Too many channels. PCA9685 supports up to 16 channels." << std::endl;
            return;
        }

        uint8_t buffer[69]; // 최대 16 채널 (4 bytes * 16 = 64) + 1 byte (start register address)
        buffer[0] = LED0_ON_L; // 시작 레지스터 (LED0_ON_L)

        // 각 모터 채널에 대해 PWM 데이터를 작성
        for (size_t i = 0; i < pwm_values.size(); ++i) {
            if (pwm_values[i] < PWM_MIN || pwm_values[i] > PWM_MAX) {
                std::cerr << "PWM value out of range for channel " << i
                        << " (" << PWM_MIN << "-" << PWM_MAX << ")" << std::endl;
                return;
            }

            int on = 0; // ON 시간은 항상 0으로 설정 (PWM 신호 시작 시점 고정)
            int off = pwm_values[i]; // OFF 시간만 설정 (PWM 듀티 비율 조절)

            buffer[1 + i * 4] = on & 0xFF;             // LEDn_ON_L
            buffer[2 + i * 4] = (on >> 8) & 0xFF;     // LEDn_ON_H
            buffer[3 + i * 4] = off & 0xFF;           // LEDn_OFF_L
            buffer[4 + i * 4] = (off >> 8) & 0xFF;    // LEDn_OFF_H
        }

        // 블록 전송으로 모든 채널 업데이트
        int retries = 0;
        while (write(fd, buffer, 1 + pwm_values.size() * 4) != 1 + pwm_values.size() * 4) {
            if (++retries >= I2C_RETRY_LIMIT) {
                std::cerr << "Failed to write to the i2c bus after retries" << std::endl;
                exit(1);
            }
            usleep(1000); // 1ms 대기 후 재시도
        }
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
            // usleep(1000); // 1ms 대기 후 재시도
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
    float filtered_derivative; // 필터링된 미분 항
    float alpha;               // 필터 계수

    PIDController(float p, float i, float d, float ff = 0.0f, float i_limit = 10.0f, float out_limit = (PWM_MAX - PWM_MIN)*0.1f, float filter_alpha = 0.1f)
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
IMUData imuData = {};
std::mutex imuMutex;
// IMU 스레드 함수
void *imuThread(void *arg) {
    initIMU("/dev/ttyUSB0", B115200);
    IMUCalibrationData calibrationData = calibrateIMU();
    float offsetGyroZ = calibrationData.offsetGyroZ;

    while (true) {
        IMUData localIMUData = readIMU(imuData);
        localIMUData.gyroZ -= offsetGyroZ; // 보정된 자이로 Z값

        // Mutex로 IMU 데이터 보호
        {
            std::lock_guard<std::mutex> lock(imuMutex);
            imuData = localIMUData;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return nullptr;
}

// PID 계산 스레드 함수
void *controlLoop(void *arg) {
    PCA9685 pca9685;
    initRC("/dev/ttyAMA0", B115200);

    PIDController rollPID(1.0f, 0.01f, 0.5f);
    PIDController pitchPID(0.0f, 0.0f, 0.0f);
    PIDController yawPID(1.2f, 0.5f, 0.5f);

    float roll_com = 0;
    float pitch_com = 0;
    auto previousTime = std::chrono::high_resolution_clock::now();

    int roll_adj = 0;
    int pitch_adj = 0;
    int yaw_adj = 0;

    while (true) {
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        IMUData localIMUData;
        {
            std::lock_guard<std::mutex> lock(imuMutex);
            localIMUData = imuData;
        }
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float, std::milli> elapsed = currentTime - previousTime; // 경과 시간 계산
        float dt = elapsed.count(); // 밀리초로 변환
        previousTime = currentTime;
        // std::cout << "dt: " << dt << std::endl;

        dt = dt / 1000.0f;

        int throttle_value = readRCChannel(3);
        int aileron_value = readRCChannel(1);
        int elevator_value = readRCChannel(2);
        int rudder_value = readRCChannel(4);

        double throttle_normalized = mapThrottle(throttle_value);
        double aileron_normalized = mapControlInput(aileron_value);
        double elevator_normalized = mapControlInput(elevator_value);
        double rudder_normalized = mapControlInput(rudder_value);

        if (std::abs(roll_com - localIMUData.roll_angle) > TOLERANCE_ROLL) {
            roll_adj = rollPID.calculate(roll_com, localIMUData.roll_angle, dt);
        }
        if (std::abs(pitch_com - localIMUData.pitch_angle) > TOLERANCE_PITCH) {
            pitch_adj = pitchPID.calculate(pitch_com, localIMUData.pitch_angle, dt);
        }
  
        // int yaw_adj = yawPID.calculate(rudder_normalized, correctedGyroZ, dt);
        int throttle_PWM = computeThrottlePWM(throttle_normalized);
        int motor1_PWM, motor2_PWM, motor3_PWM, motor4_PWM;
        std::cout << "localIMUData.roll_angle: " << localIMUData.roll_angle << std::endl;
        std::cout << "roll_adj: " << roll_adj << std::endl;
        if (throttle_PWM <= PWM_MIN) {
            std::vector<int> motor_min_pwm = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
            pca9685.setAllMotorsSpeed(motor_min_pwm);
            continue;
        } 
        else {
            int aileron_adj_total = computeAdjustment(aileron_normalized) + roll_adj;
            // int elevator_adj_total = computeAdjustment(elevator_normalized) + pitch_adj;
            int elevator_adj_total = 0;
            //std::cout << "adj: " << roll_adj << "total" << aileron_adj_total << std::endl;
            int motor1_adj = -aileron_adj_total + elevator_adj_total + yaw_adj;
            int motor2_adj = aileron_adj_total - elevator_adj_total - yaw_adj;
            int motor3_adj = aileron_adj_total + elevator_adj_total + yaw_adj;
            int motor4_adj = -aileron_adj_total - elevator_adj_total - yaw_adj;
            
            motor1_PWM = clamp(throttle_PWM + motor1_adj, PWM_MIN, PWM_MAX);
            motor2_PWM = clamp(throttle_PWM + motor2_adj, PWM_MIN, PWM_MAX);
            motor3_PWM = clamp(throttle_PWM + motor3_adj, PWM_MIN, PWM_MAX);
            motor4_PWM = clamp(throttle_PWM + motor4_adj, PWM_MIN, PWM_MAX);
            
        }
        std::vector<int> motor_pwm = {motor1_PWM, motor2_PWM, motor3_PWM, motor4_PWM};
        pca9685.setAllMotorsSpeed(motor_pwm);
        
        pca9685.setMotorSpeed(0, motor1_PWM);
        pca9685.setMotorSpeed(1, motor2_PWM);
        pca9685.setMotorSpeed(2, motor3_PWM);
        pca9685.setMotorSpeed(3, motor4_PWM);

        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
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
