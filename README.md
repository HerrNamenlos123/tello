<img src="assets/tello.png" alt="Tello drone" align="right" width=40%>

# Tello

C++14 single-header library to control a DJI Ryze Tello drone using the Tello SDK 2.0

## Build process

There is no build process. This is a single-header library, which means you only need to include `tello.hpp` and that's it. Look at the examples to see how to use it.

Alternatively, there is also a `CMakeLists.txt` available, so if you want to keep it easy to update you might also use this repository as a git submodule. Then you would simply include the library in cmake. Take a look at the [examples](examples) folder.

## Examples

### Simple example

```c++
Tello tello;
if (!tello.connect()) {     // This can take a different Tello IP address if needed
    return;
}

tello.takeoff();

// Do something, fly around

tello.land();
```

### Most important functions

These are your basic functions for controlling the Tello drone
```c++
tello.connect();
tello.takeoff();
tello.land();
```

Basic incremental movement functions
```c++
tello.set_speed(cm/s)
tello.move_up(cm);
tello.move_down(cm);
tello.move_left(cm);
tello.move_right(cm);
tello.move_forward(cm);
tello.move_back(cm);
tello.turn_right(degrees);
tello.turn_left(degrees);
tello.move_by(x, y, z, speed);
```

Main function for more sophisticated controllers: This function basically simulates an RC controller, so it can be called in a loop to continuously set the speed in all directions
```c++
tello.move(left/right, forward/backward, up/down, yaw);
```

Sensor readings
```c++
tello.get_speed();
tello.get_battery_level();
tello.get_flight_time();
tello.get_wifi_snr();
tello.get_sdk_version();
tello.get_serial_number();
```

Disclaimer: Only the functions which you are most likely to use are listed here. Not all are documented.


## Licensing📃

This library is released under the MIT License. This means you are allowed and even encouraged to copy and paste `tello.hpp` directly into your project. No installation or build process is necessary.

## Acknowledgements💡

This library uses the [UDPsocket](https://github.com/barczynsky/UDPsocket) single-header library, embedded into `tello.hpp` itself.
