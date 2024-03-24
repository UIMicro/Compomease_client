# Compomease
This is the source code of my AI2618 Term Project, which aims to build a simple circuit to decide the type of components and to measure its parameters.

## Academic Integrity

You can refer to the code in this repo, but with following limitations:
- If you refer to my code, by default you will abide by the license and the following rules：
- DO NOT directly copy and paste. Understand the code and write your own version.
- Indicate in your report that you referred the code here.

## Basic Information

- MCU: Arduino nano ESP32 (with multiple channels of 12bit ADC, high sample rate, high)
- Framework: Arduino (easier to get started than ESP-IDF)
- Layout: breadboard (DO NOT CONNECT WITH THE PCB MODULE!)

  
## MCU Program Framework

### void initializePins();

- Launched: on startup
- Function: initialize pin configuration, initialize ADC resolution

### void setHigh(low)Impedance();

- Launched: on demand
- Function: enable strong (weak) pullup and pulldown resistor

### void generateCalibration();

- Launched: on startup
- Function: generate calibration data of the ADC using linear interpolation
- Input: previously measured voltage data
- Output: an array mapping binary data to voltage level, float[4096]

### double repeatSample(uint8_t pin, size_t sampleTimes, unsigned long long delay_us);

- Launched: on demand
- Function: repeatly measure voltage level to reduce noise

### void decideComponent();

- Launched: called by main()
- Function: decide the type of the measured component, send to host

### void measureResistor();

- Launched: called by main()
- Functions:
  - Decide the resistor value range and switch measuring range
  - sample many times to reduce noise
  - send to host

### void measureCapacitor();

- Launched: called by main()
- Functions:
  - Decide the capacitor value range and switch measuring range
  - Charge from 1/3u to 2/3u, and discharge from 2/3u to 1/3u to acquire capacitor value
  - send to host

### void measureBJT();

- Launched: called by main()
- Functions:
  - Decide BJT type (PNP/NPN) using Ub
  - Modulate Ib with sigma-delta modulation
  - sample Uc with different Rc and Ib
  - send to host

## Host Program Framework

- Language: Python
- Communication: serial
- Visualization: Matplotlib
- GUI Framework: tkinter, PyQt, etc.

## Communication Protocol

#TODO

## Involution Points

- Bluetooth Communication
- LCD display
- High-Precision Acquisition (with OPAMP, external ADCs, balanced bridges, etc.)
- PCB

## References

### ADC/Analog: https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html
