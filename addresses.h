

#ifndef FIRMWARE_ADDRESSES_H
#define FIRMWARE_ADDRESSES_H

#define sensorAddr 0x39  // Sensor physical address on the power board - 0x39
#define ch0        0x43  // Read ADC for channel 0 - 0x43
#define ch1        0x83  // Read ADC for channel 1 - 0x83

#define grabber_i2c_addr 1
#define encoder_i2c_addr 5
#define kicker_i2c_addr 6

#endif //FIRMWARE_ADDRESSES_H
