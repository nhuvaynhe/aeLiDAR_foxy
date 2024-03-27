#include "arduino_comms.h"
#include <iostream>

ArduinoComms arduino_;

int main() {
    int32_t baudrate = 115200;
    std::string serial_port = "/dev/ttyACM0";
    int32_t timeout = 1000;

    arduino_.setup(serial_port, baudrate, timeout);
    std::cout << "Connect!" << std::endl;

    while(1){
        arduino_.setMotorValues(100, 100);
    }

    return 0;
}
