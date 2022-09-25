
#define TELLO_DEBUG     // This can be used to enable verbose logging
#include "tello.hpp"
#include <iostream>
a
void fly() {
	
    Tello tello;
    if (!tello.connect()) {
        return;
    }
	
    PRINTF_WARN("Tello is connected and about to take off and fly around! Are you ready? [Press Enter]");
    std::cin.get();

    tello.takeoff();
    
    tello.move_right(20);
    tello.move_forward(20);
    tello.move_left(40);
    tello.move_back(40);
    tello.move_right(40);
    tello.move_forward(20);
    tello.move_left(20);
    
    tello.land();
}

int main() {
    fly();
    PRINTF_INFO("[Press Enter to exit]");
    std::cin.get();
    return 0;
}
