<img src="assets/tello.png" alt="Tello drone" align="right" width=40%>

# Tello

C++14 single-header library to control a DJI Ryze Tello drone using the Tello SDK 2.0

## Build process

There is no build process. This is a single-header library, which means you only need to include `tello.hpp` and that's it. Look at the examples to see how to use it.

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

### Main functions

```
```

## Licensing📃

This library is released under the MIT License. This means you are allowed and even encouraged to copy and paste `tello.hpp` directly into your project. No installation or build process is necessary.

## Acknowledgements💡

This library uses the [UDPsocket](https://github.com/barczynsky/UDPsocket) single-header library, embedded into `tello.hpp` itself.
