
#define TELLO_DEBUG
#include "tello.hpp"
#include "tello.h"

#include <iostream>

void fly() {
    Tello tello;

    if (!tello.connect()) {
        return;
    }

    tello.takeoff();

    tello.move_to_position(50, 50, 50, 10);
    tello.move_to_position(-50, -50, -50, 10);

    tello.land();
}

int main() {
    fly();
    printf("Waiting for ENTER");
    std::cin.get();
    return 0;
}



//tello.move_right(30);
//tello.move_forward(30);
//tello.move_left(60);
//tello.move_back(60);
//tello.move_right(60);
//tello.move_forward(30);
//tello.move_left(30);