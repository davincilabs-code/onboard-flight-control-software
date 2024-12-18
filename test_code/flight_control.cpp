#include "flight_control.h"
#include "rc_input.h"
#include "gps_sensor.h"
#include "imu_sensor.h"
#include <iostream>
#include <termios.h> // B115200 보드레이트 상수 정의를 위해 필요.

// 모든 장치를 초기화하는 함수
void flight_control_init() {
    // RC 초기화
    std::cout << "Initializing RC input..." << std::endl;
    initRC("/dev/ttyAMA0", B115200);

    // GPS 초기화
    std::cout << "Initializing GPS..." << std::endl;
    initGPS("/dev/ttyUSB1", B115200);

    // IMU 초기화
    std::cout << "Initializing IMU..." << std::endl;
    initIMU("/dev/ttyUSB0", B115200);

    std::cout << "Flight control system initialized." << std::endl;
}

