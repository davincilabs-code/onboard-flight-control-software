#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include <array>
#include <Eigen/Dense>
#include <unistd.h>

const int PWM_MIN = 215;
const int PWM_MAX = 410;

// PID 제어기 클래스 정의함
class PIDController {
public:
    PIDController(float kp, float ki, float kd)
        : _kp(kp), _ki(ki), _kd(kd), _prev_error(0.0f), _integral(0.0f) {}

    float update(float target, float current, float dt) {
        float error = target - current;
        _integral += error * dt;
        float derivative = (error - _prev_error) / dt;
        _prev_error = error;
        return (_kp * error) + (_ki * _integral) + (_kd * derivative);
    }

    // 리셋 함수 추가
    void reset() {
        _prev_error = 0.0f;  // 이전 오차 초기화
        _integral = 0.0f;     // 적분 값 초기화
    }

private:
    float _kp, _ki, _kd;
    float _prev_error, _integral;
};

// controlAttitude 함수 선언
std::array<float, 4> controlAttitude(float dt);

#endif // ATTITUDE_CONTROLLER_H
