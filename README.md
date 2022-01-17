# RotaryEncoder
***STM32*** Low Layer(LL) library. A rotary encoder is a type of position sensor that converts the angular position 
(rotation) of a knob into an output signal that is used to determine what direction the knob is being rotated.

<img src="https://github.com/ximtech/RotaryEncoder/blob/main/example/encoder.PNG" alt="image" width="300"/>

### Features
- Good precision
- Button handling

### Add as CPM project dependency

How to add CPM to the project, check the [link](https://github.com/cpm-cmake/CPM.cmake)
```cmake
CPMAddPackage(
        NAME RotaryEncoder
        GITHUB_REPOSITORY ximtech/RotaryEncoder
        GIT_TAG origin/main)
```

### Project configuration

1. Start project with STM32CubeMX:
    * [Timer configuration](https://github.com/ximtech/RotaryEncoder/blob/main/example/config_1.PNG)
    * [GPIO configuration](https://github.com/ximtech/RotaryEncoder/blob/main/example/config_2.PNG)
    * [GPIO configuration 2](https://github.com/ximtech/RotaryEncoder/blob/main/example/config_3.PNG)
2. Select: Project Manager -> Advanced Settings -> GPIO -> LL
3. Generate Code
4. Add sources to project:

```cmake
include_directories(${includes} 
        ${ROTARY_ENCODER_DIRECTORY})   # source directories

file(GLOB_RECURSE SOURCES ${sources} 
        ${ROTARY_ENCODER_SOURCES})    # source files
```

3. Then Build -> Clean -> Rebuild Project

## Wiring

- <img src="https://github.com/ximtech/RotaryEncoder/blob/main/example/Rotary-Encoder-Pinout.jpg" alt="image" width="300"/>

## Usage

The `RotaryEncoder.h` has default configuration defines. Override them in `main.h` if needed

```c
#define ENCODER_POLLING_TICK_DELAY 1000
#define BUTTON_HOLD_TIMEOUT_MS 1000
#define BUTTON_CLICK_TIMEOUT_MS 1000
```

- Usage example: [link](https://github.com/ximtech/RotaryEncoder/blob/main/example/example.c)
