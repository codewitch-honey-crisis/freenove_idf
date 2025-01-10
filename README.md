# Freenove ESP32S3 Devkit peripheral test

This code supports all the peripherals of this board behind a C ABI

Note:

I2C is limited to the legacy drivers due to the camera component using legacy I2C
See https://github.com/espressif/esp32-camera/issues/713


The ESP LCD Panel API is not useable with this display

See https://github.com/espressif/esp-idf/issues/15160

SPI master ISR functions are in SRAM

Neopixel uses I2S bus index 1 (the second bus) instead of RMT so it won't interfere with the radio


