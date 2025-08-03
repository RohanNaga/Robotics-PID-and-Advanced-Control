# Hardware Setup Guide

## System Overview

The control system interfaces with the CRS Catalyst-5 robot through a custom control board based on the TI F28335 DSP.

## Hardware Components

### 1. TI F28335 DSP Board
- **Processor:** TMS320F28335
- **Clock:** 150 MHz
- **Memory:** 68KB RAM, 512KB Flash
- **Features:** 32-bit floating-point unit, 12-bit ADC, ePWM modules

### 2. Robot Specifications
- **Model:** CRS Catalyst-5
- **DOF:** 3 rotational joints
- **Payload:** 5 kg maximum
- **Repeatability:** ±0.05 mm
- **Reach:** 710 mm

### 3. Motor System
- **Motors:** DC brushed motors with integrated encoders
- **Gearing:** 100:1 harmonic drives
- **Power:** 24V nominal, 3A continuous per motor

### 4. Sensors
- **Encoders:** Incremental, 4096 counts/revolution
- **Current Sensing:** Hall-effect sensors, ±10A range
- **Limit Switches:** Mechanical switches at joint limits

## Electrical Connections

### Power Supply
```
Main Power: 24V DC, 15A minimum
Logic Power: 5V DC (derived from main)
DSP Power: 3.3V, 1.8V (on-board regulators)
```

### Motor Connections
```
Joint 1: PWM1A/B (GPIO0/1), QEP1 (GPIO20/21)
Joint 2: PWM2A/B (GPIO2/3), QEP2 (GPIO24/25)
Joint 3: PWM3A/B (GPIO4/5), EQEP1A/B (GPIO50/51)
```

### Communication
```
UART: GPIO28/29 (SCI-A) at 115200 baud
SPI: GPIO16-19 (for future expansion)
```

## DSP Configuration

### Clock Setup
```c
// PLL configuration for 150MHz
// External crystal: 30MHz
// PLLCR = 10 (multiply by 5)
// DIVSEL = 2 (divide by 2)
// System clock = 30MHz * 5 / 2 = 150MHz
```

### PWM Configuration
- Frequency: 20 kHz
- Dead-band: 2 μs
- Mode: Complementary PWM with dead-band

### ADC Setup
- Sampling rate: 10 kHz
- Channels: Motor currents, battery voltage
- Trigger: ePWM synchronized

## Safety Circuits

### Emergency Stop
- Hardware E-stop bypasses DSP
- Cuts power to motor drivers
- Maintains encoder power for position tracking

### Overcurrent Protection
- Hardware comparators on each motor
- Threshold: 5A peak
- Auto-reset after 100ms

### Watchdog Timer
- Timeout: 100ms
- Resets DSP if control loop fails
- Logs fault before reset

## Calibration Procedures

### Encoder Calibration
1. Move joint to mechanical zero
2. Reset encoder counter
3. Move through full range
4. Verify count matches expected value

### Current Sensor Calibration
1. Apply known current through sensor
2. Read ADC value
3. Calculate scale factor and offset
4. Store in EEPROM

### Joint Limit Calibration
1. Slowly move to positive limit
2. Record encoder position
3. Repeat for negative limit
4. Set soft limits 5° inside hard limits

## Communication Protocol

### Packet Structure
```
[START][LENGTH][CMD][DATA...][CRC16][END]
0xAA    1 byte  1 byte  N bytes  2 bytes  0x55
```

### Command Set
- 0x01: Read joint positions
- 0x02: Set joint targets
- 0x03: Read joint velocities
- 0x04: Set control gains
- 0x05: Emergency stop
- 0x06: Read system status

### Error Handling
- Timeout: 100ms
- Retry count: 3
- CRC validation on all packets

## Startup Sequence

1. **Power On**
   - Check supply voltages
   - Initialize DSP peripherals
   - Load configuration from flash

2. **System Check**
   - Verify encoder connections
   - Test limit switches
   - Check motor driver status

3. **Homing**
   - Move joints to known position
   - Reset encoder counts
   - Enable closed-loop control

4. **Ready State**
   - Wait for commands
   - Monitor safety systems
   - Update status LEDs

## Troubleshooting

### Common Issues

1. **Joint doesn't move**
   - Check motor driver enable
   - Verify PWM output
   - Check current limits

2. **Position drift**
   - Verify encoder connections
   - Check for missed counts
   - Calibrate encoder scaling

3. **Oscillation**
   - Reduce derivative gain
   - Check for mechanical backlash
   - Verify sample rate

### Debug Tools

- **Serial Monitor:** Real-time variable monitoring
- **Scope Channels:** PWM and encoder signals
- **LED Indicators:**
  - Green: Normal operation
  - Yellow: Warning (near limits)
  - Red: Fault condition

## Maintenance

### Regular Checks
- Monthly: Encoder alignment
- Quarterly: Gear backlash
- Annually: Full calibration

### Spare Parts
- Encoders: US Digital E4P
- Motor drivers: L298N modules
- Fuses: 5A fast-blow

## Performance Specifications

### Achieved Performance
- Position accuracy: ±0.5°
- Repeatability: ±0.1°
- Maximum velocity: 180°/s
- Maximum acceleration: 360°/s²

### Power Consumption
- Idle: 2W (DSP only)
- Moving: 15-50W (load dependent)
- Peak: 150W (all joints accelerating)