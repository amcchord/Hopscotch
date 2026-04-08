# RobStride RS00 - 14N.M Quasi-Direct Drive Integrated Motor Module - User Manual

**Manufacturer:** RobStride Dynamics (Beijing Lingfoot Times Technology Co., LTD)
**Website:** https://www.robstride.com/

---

## Precautions

1. Please use according to the working parameters specified in this article, otherwise it may cause serious damage to the product!
2. Do not switch the control mode when the joint is running. If you need to switch, send the command to stop the operation before switching.
3. Check whether the parts are in good condition before use. If the parts are missing or damaged, contact technical support in time.
4. Do not disassemble the motor at will, so as to avoid unrecoverable failure.
5. Ensure that there is no short circuit when the motor is connected, and the interface is correctly connected as required.

---

## Legal Statement

Before using this product, please read this manual carefully and operate the product according to the contents of this manual. Published by Beijing Lingfoot Times Technology Co., LTD. (hereinafter referred to as Lingfoot). Updates available at www.robstride.com.

---

## After-sales Policy

- **Warranty period:** 1 year
- **7-day return policy** for online purchases
- **7 days:** Return for non-human damage
- **7-15 days:** Replace for non-human damage
- **15-365 days:** Free maintenance for quality faults
- **Non-warranty:** exceeding warranty period, wrong use, improper operation, abnormal conditions, natural disasters, exceeding peak torque, etc.

---

# Section 1: Motor Specification

## 1.1 Outline and Mounting Dimensions

- **6-M3 EQS mounting holes** at 30° spacing
- **Output shaft:** Φ24.46±0.1
- **Body diameter:** Φ50±0.1
- **Base diameter:** Φ27±0.1
- **Rear:** 4-M3 V4, Φ38±0.1
- **Rear mounting:** 6-M3*5
- **Height:** 51±1
- **Important:** When fixing, the screw depth should not exceed the depth of the casing thread

## 1.2 Standard Service Condition

1. **Rated voltage:** 48 VDC
2. **Operating voltage range:** 24V-60 VDC
3. **Rated load (CW):** 5 N.m
4. **Operation direction:** CW/CCW from the direction of the exit shaft
5. **Use posture:** the direction of the exit axis is horizontal or vertical
6. **Standard operating temperature:** 25±5°C
7. **Operating temperature range:** -20 ~ 50°C
8. **Standard operating humidity:** 65%
9. **Humidity range:** 5 ~ 85%, no condensation
10. **Storage temperature range:** -30 ~ 70°C
11. **Insulation Class:** Class B

## 1.3 Electrical Characteristics

1. **No load speed:** 315 rpm±10%
2. **No-load current:** 0.5 Arms
3. **Rated load:** 5 N.m
4. **Rated load speed:** 100rpm±10%
5. **Rated load phase current (peak):** 4.7Apk±10%
6. **Peak load:** 14 N.m
7. **Maximum load phase current (peak):** 15.5Apk±10%
8. **Insulation resistance/stator winding:** DC 500VAC, 100M Ohms
9. **High voltage/stator and housing:** 600 VAC, 1s, 2mA
10. **Motor back potential:** 9.5Vrms/kRPM±10%
11. **Torque constant (Valid value):** 1.48N.m/Arms

### T-N Curve (48V)

The T-N curve shows torque (N.m) vs speed (RPM). At 48V, the motor produces approximately:

- 14 N.m at 0 RPM (stall)
- ~13 N.m at 40 RPM
- ~12 N.m at 80 RPM
- ~10 N.m at 140 RPM
- ~8 N.m at 200 RPM
- ~6 N.m at 240 RPM
- ~4 N.m at 280 RPM
- ~2 N.m at 300 RPM
- 0 N.m at ~315 RPM (no-load)

### Maximum Overload Curve

**Test conditions:**
- Ambient temperature: 25°C
- Winding limit temperature: 145°C (constraint, actual is 180°C)
- Speed: 24rpm

#### Rotation Test Data

| Load (N.m) | Operating Time (s) |
|---|---|
| 14 | 5 |
| 13 | 6 |
| 12 | 10 |
| 11 | 12 |
| 10 | 18 |
| 9 | 22 |
| 8 | 55 |
| 7 | 120 |
| 6 | 280 |
| 5 | rated |

#### Locked-Rotor Test Data

(Phase current during locked-rotor is approximately 1.414 times that of rotation)

| Load (N.m) | Operating Time (s) |
|---|---|
| 14 | 1.5 |
| 13 | 1.5 |
| 12 | 2 |
| 11 | 3 |
| 10 | 5 |
| 9 | 9 |
| 8 | 16 |
| 7 | 43 |
| 6 | 74 |
| 5 | rated |

### Thermal Testing

**Test Conditions:**
- Heat sink size: 90mm x 85mm
- Ambient temperature: 25°C
- Winding protection temperature: 145°C
- Test condition 1: Motor rotates at 100rpm, monitor the motor thermistor temperature, and record the time it takes for the motor thermistor temperature to rise from 25°C to 145°C.

### Mechanical Characteristics

1. **Weight:** 310g±3g
2. **Number of poles:** 28
3. **Phase number:** 3 phases
4. **Drive mode:** FOC
5. **Deceleration ratio:** 10:1

---

# Section 2: Driver Product Information

## 2.1 Driver Product Specifications

| Parameter | Value |
|---|---|
| Rated working voltage | 48VDC |
| Maximum allowable voltage | 60VDC |
| Rated working phase current | 4.7Apk |
| Maximum allowable phase current | 15.5Apk |
| Standby power | ≤18mA |
| CAN bus bit rate | 1Mbps |
| Dimensions | Φ57mm |
| Working environment temperature | -20°C to 50°C |
| Maximum allowable temperature of control board | 145°C |
| Encoder resolution | 14bit (absolute turn) |

## 2.2 Driver Interface Definition

The driver has 4 connections:

- **VBAT+:** Power positive
- **GND:** Ground
- **CAN-L:** CAN bus low
- **CAN-H:** CAN bus high

## 2.3 Recommended Driver Interface Brand and Model

| Board End Model | Brand Manufacturer | Line End Model | Brand Manufacturer |
|---|---|---|---|
| XT30PB(2+2)-M.G.B | AMASS (Ams) | XT30(2+2)-F.G.B | AMASS (Ams) |

---

# Section 3: Upper Computer Instructions

Download software from www.robstride.com website download center.

## 3.1 Hardware Disposition

The articulated motor uses CAN communication mode and has two communication cables. It is connected to the debugger through the CAN to USB tool. The debugger needs to be installed with the ch340 driver in advance and works in AT mode by default.

The CAN to USB tool is recommended to use the official USB-CAN module of Lingzu Times. The frame header of the corresponding serial port protocol is `41 54`, and the frame tail is `0D 0A`.

### DIP Switch Settings

When using the CAN-to-USB module, pay attention to the settings of the DIP switches:

- **DIP switch 1 ON:** Boot mode (cannot communicate with host)
- **DIP switch 2 ON:** 120Ω terminal resistor connected to module port

## 3.2 Upper Computer Interface and Description

**Software:** motorstudio 0.0.11

### Interface Modules

- **Motor Connection Module:** Refresh Serial Port, Open Serial Port, Testing the Device
- **Motor Configuration Module:** Starting the Upgrade, Opening a File, Starting the Upgrade, Modifying the Motor CAN ID, Setting the Motor's Mechanical Zero Position
- **Motor Upgrade Module:** Magnetic Encoder Calibration, Motor Active Reporting Switch, Setting the Motor Active Reporting Time, Modifying the Motor CAN ID, Setting the Motor's Mechanical Zero Position
- **Motor Main Interface:** Parameter Settings, Motor Oscilloscope
- **Run and Debug Area:** Parameter Debugging Buttons, Motor Mode Configuration and Parameter Modification, Sine Signal Testing

## 3.3 Motor Settings

### 3.3.1 Motor Connection Settings

Connect the CAN to USB tool (install the ch340 driver, which works in AT mode by default), click Refresh Serial Port, open the serial port, and click Detect Device to detect the corresponding motor. The green text below is the motor type.

### 3.3.2 Motor Configuration Module

1. **Recalibrate the motor magnetic encoder.** Reinstalling the motor board and motor, or reconnecting the motor's three-phase wiring requires recalibrating the magnetic encoder.
2. **Enable active motor reporting.** Click Start Reporting to enable active motor reporting in communication type 2. You can set the interval below, with a minimum of 10ms.
3. **Set ID:** Set the motor's CAN ID.
4. **Set Zero Position:** Set the current position to 0.

### 3.3.3 Motor Upgrade Module (OAT)

1. Click to open the file and select the firmware to upgrade. The rs-0x in the firmware name is the selected motor type.
2. Click Start Upgrade, and the motor will enter the upgrade preparation stage.
3. When the green text "Device has entered upgrade mode" pops up, click to start the upgrade.
4. When the green text "Upgrade Successfully" pops up, the upgrade is complete.

**Troubleshooting:**
- If the green progress bar gets stuck halfway through the upgrade, you can click to stop the upgrade, or re-power on and re-enter the upgrade process.
- The internal program of the motor will not be lost after the upgrade fails.

### 3.3.4 Parameter Settings

**IMPORTANT NOTE:** Please do not change the torque limit, protection temperature and overtemperature time of the motor. Our company will not bear any legal responsibility for any damage to human body or irreversible damage to joints caused by illegal operation of this product.

#### Parameter Management Operations

After successfully connecting to the motor:

1. Click **Refresh Parameter Table** to read motor parameters (must be in standby mode).
2. Click **Read Parameters** to upload motor parameters to the debugger (light blue = observed real-time).
3. Click **Write Parameters** to download debugger parameters to the motor.
4. Click **Restore Factory** to restore default parameters for the latest firmware.
5. Click **Export** to export the current motor parameters.
6. Click **Open Multi-Device Connection** to connect the host computer to multiple motors.

#### Full Parameter Table

| Function Code | Name | Parameter Type | Attribute | Max Value | Min Value | Current Value | Notes |
|---|---|---|---|---|---|---|---|
| 0X0000 | Name | String | Read/Write | | | yyyyyyyyyyyyy | |
| 0X0001 | BarCode | String | Read/Write | | | yyyyyyyyyyyyy | |
| 0X1000 | BootCodeVersion | String | Read only | | | 0.1.5 | |
| 0X1001 | BootBuildDate | String | Read only | | | Mar 16 2022 | |
| 0X1002 | BootBuildTime | String | Read only | | | 20:22:09 | |
| 0X1003 | AppCodeVersion | String | Read only | | | 0.0.0.1 | Motor program version number |
| 0X1004 | AppGitVersion | String | Read only | | | 7b844b0fM | |
| 0X1005 | AppBuildDate | String | Read only | | | Apr 14 2022 | |
| 0X1006 | AppBuildTime | String | Read only | | | 20:30:22 | |
| 0X1007 | AppCodeName | String | Read only | | | Lingzu_motor | |
| 0X2000 | echoPara1 | uint16 | disposition | 74 | 5 | 5 | |
| 0X2001 | echoPara2 | uint16 | disposition | 74 | 5 | 5 | |
| 0X2002 | echoPara3 | uint16 | disposition | 74 | 5 | 5 | |
| 0X2003 | echoPara4 | uint16 | disposition | 74 | 5 | 5 | |
| 0X2004 | echoFreHz | uint32 | Read/Write | 10000 | 1 | 500 | |
| 0X2005 | MechOffset | float | Settings | 7 | -7 | 4.619583 | Motor magnetic encoder angle offset |
| 0X2006 | MechPos_init | float | Read/Write | 50 | -50 | 4.52 | Reserved parameter |
| 0X2007 | limit_torque | float | Read/Write | 17 | 0 | 17 | Torque limitation |
| 0X2008 | I_FW_MAX | float | Read/Write | 33 | 0 | 0 | Weak magnetic current value, default 0 |
| 0X2009 | motor_baud | uint8 | Settings | 20 | 0 | 1 | Baud rate flag bit |
| 0X200a | CAN_ID | uint8 | Settings | 127 | 0 | 1 | ID of this object |
| 0X200b | CAN_MASTER | uint8 | Settings | 127 | 0 | 0 | CAN host ID |
| 0X200c | CAN_TIMEOUT | uint32 | Read/Write | 100000 | 0 | 0 | CAN timeout threshold. Default value is 0 (disabled) |
| 0X200d | status2 | int16 | Read/Write | 1500 | 0 | 800 | Reserved parameter |
| 0X200e | status3 | uint32 | Read/Write | 1000000 | 1000 | 20000 | Reserved parameter |
| 0X200f | status1 | float | Read/Write | 64 | 1 | 7.75 | Reserved parameter |
| 0X2010 | Status6 | uint8 | Read/Write | 1 | 0 | 1 | Reserved parameter |
| 0X2011 | cur_filt_gain | float | Read/Write | 1 | 0 | 0.9 | Current filtering parameter |
| 0X2012 | cur_kp | float | Read/Write | 200 | 0 | 0.025 | Current kp |
| 0X2013 | cur_ki | float | Read/Write | 200 | 0 | 0.0258 | Current ki |
| 0X2014 | spd_kp | float | Read/Write | 200 | 0 | 2 | Velocity kp |
| 0X2015 | spd_ki | float | Read/Write | 200 | 0 | 0.021 | Speed ki |
| 0X2016 | loc_kp | float | Read/Write | 200 | 0 | 30 | Position kp |
| 0X2017 | spd_filt_gain | float | Read/Write | 1 | 0 | 0.1 | Velocity filter parameter |
| 0X2018 | limit_spd | float | Read/Write | 200 | 0 | 2 | Location mode speed limit |
| 0X2019 | limit_cur | float | Read/Write | 23 | 0 | 23 | Position, Velocity mode current limit |
| 0X201a | loc_ref_filt_gain | float | Read/Write | 100 | 0 | 0 | Reserved parameter |
| 0X201b | limit_loc | float | Read/Write | 100 | 0 | 0 | Reserved parameter |
| 0X201c | position_offset | float | Read/Write | 27 | 0 | 0 | High speed segment offset |
| 0X201d | chasu_angle_offset | float | Read/Write | 27 | 0 | 0 | The low end is offset |
| 0X201e | spd_step_value | float | Read/Write | 150 | 0 | 0 | Velocity-mode acceleration |
| 0X201f | vel_max | float | Read/Write | 20 | 0 | 0 | PP mode speed |
| 0X2020 | acc_set | float | Read/Write | 1000 | 0 | 0 | PP mode acceleration |
| 0X2021 | zero_sta | float | Read/Write | 100 | 0 | 0 | Zero marker |
| 0x2022 | protocol_1 | uint8 | Read/Write | | | 0 | Protocol flag |
| 0x2023 | damper | uint8 | Read/Write | 0 | 20 | 0 | Damping switch |
| 0x2024 | add_offset | float | Read/Write | -7 | 7 | 0 | Position offset parameter |
| 0X3000 | timeUse0 | uint16 | Read only | | | 5 | |
| 0X3001 | timeUse1 | uint16 | Read only | | | 0 | |
| 0X3002 | timeUse2 | uint16 | Read only | | | 10 | |
| 0X3003 | timeUse3 | uint16 | Read only | | | 0 | |
| 0X3004 | encoderRaw | int16 | Read only | | | 11396 | Magnetic encoder sampling value |
| 0X3005 | mcuTemp | int16 | Read only | | | 337 | MCU internal temperature, *10 |
| 0X3006 | motorTemp | int16 | Read only | | | 333 | Motor NTC temperature, *10 |
| 0X3007 | vBus(mv) | uint16 | Read only | | | 24195 | Bus voltage |
| 0X3008 | adc1Offset | int32 | Read only | | | 2084 | ADC sampling channel 1 zero current bias |
| 0X3009 | adc2Offset | int32 | Read only | | | 2084 | ADC sampling channel 2 zero current bias |
| 0X300a | adc1Raw | uint16 | Read only | | | 1232 | ADC sampling value 1 |
| 0X300b | adc2Raw | uint16 | Read only | | | 1212 | ADC sampling value 2 |
| 0X300c | VBUS | float | Read only | | | 36 | Bus voltage V |
| 0X300d | cmdId | float | Read only | | | 0 | Id ring instruction, A |
| 0X300e | cmdIq | float | Read only | | | 0 | Iq ring command, A |
| 0X300f | cmdlocref | float | Read only | | | 0 | Position loop command, rad |
| 0X3010 | cmdspdref | float | Read only | | | 0 | Speed loop command, rad/s |
| 0X3011 | cmdTorque | float | Read only | | | 0 | Torque instruction, nm |
| 0X3012 | cmdPos | float | Read only | | | 0 | MIT Protocol Angle instruction |
| 0X3013 | cmdVel | float | Read only | | | 0 | MIT Protocol Speed instruction |
| 0X3014 | rotation | int16 | Read only | | | 1 | Number of turns |
| 0X3015 | modPos | float | Read only | | | 4.363409 | Motor uncounted coil mechanical angle, rad |
| 0X3016 | mechPos | float | Read only | | | 0.777679 | Load end loop mechanical angle, rad |
| 0X3017 | mechVel | float | Read only | | | 0.036618 | Load speed: rad/s |
| 0X3018 | elecPos | float | Read only | | | 4.714761 | Electrical Angle |
| 0X3019 | ia | float | Read only | | | 0 | U-wire current, A |
| 0X301a | ib | float | Read only | | | 0 | V-wire current, A |
| 0X301b | ic | float | Read only | | | 0 | W-wire current, A |
| 0X301c | timeout | uint32 | Read only | | | 31600 | Timeout counter value |
| 0X301d | phaseOrder | uint8 | Read only | | | 0 | Directional marking |
| 0X301e | iqf | float | Read only | | | 0 | Iq filter value, A |
| 0X301f | boardTemp | int16 | Read only | | | 359 | Plate temperature, *10 |
| 0X3020 | iq | float | Read only | | | 0 | Iq Original value, A |
| 0X3021 | id | float | Read only | | | 0 | Id Original value, A |
| 0X3022 | faultSta | uint32 | Read only | | | 0 | Fault status value |
| 0X3023 | warnSta | uint32 | Read only | | | 0 | Warning status value |
| 0X3024 | drv_fault | uint16 | Read only | | | 0 | Driver chip fault value 1 |
| 0X3025 | drv_temp | int16 | Read only | | | 48 | Driver chip fault value 2 |
| 0X3026 | Uq | float | Read only | | | 0 | Q-axis voltage |
| 0X3027 | Ud | float | Read only | | | 0 | D-axis voltage |
| 0X3028 | dtc_u | float | Read only | | | 0 | Duty cycle of U-phase output |
| 0X3029 | dtc_v | float | Read only | | | 0 | Duty cycle of V-phase output |
| 0X302a | dtc_w | float | Read only | | | 0 | Duty cycle of W-phase output |
| 0X302b | v_bus | float | Read only | | | 24.195 | Vbus in the closed loop |
| 0X302c | torque_fdb | float | Read only | | | 0 | Torque feedback value, nm |
| 0X302d | rated_i | float | Read only | | | 8 | Rated current of motor |
| 0X302e | limit_i | float | Read only | | | 27 | The motor limits the maximum current |
| 0X302f | spd_ref | float | Read only | | | 0 | Motor speed expectation |
| 0X3030 | spd_reff | float | Read only | | | 0 | Motor speed expectation 2 |
| 0X3031 | zero_fault | float | Read only | | | 0 | Motor position determination parameters |
| 0X3032 | chasu_coder_raw | float | Read only | | | 0 | Motor position determination parameters |
| 0X3033 | chasu_angle | float | Read only | | | 0 | Motor position determination parameters |
| 0X3034 | as_angle | float | Read only | | | 0 | Motor position determination parameters |
| 0X3035 | vel_max | float | Read only | | | 0 | Motor position determination parameters |
| 0X3036 | judge | float | Read only | | | 0 | Motor position determination parameters |
| 0X3037 | position | float | Read only | | | 0 | Position value |
| 0X3038 | chasu_angle_init | float | Read only | | | -0.002301 | Angle initialization |
| 0X3039 | chasu_angle_out | float | Read only | | | -0.23012 | Motor position determination parameters |
| 0X303a | motormechinit | float | Read only | | | -0.000383 | Motor position determination parameters |
| 0X303b | mech_angle_init2 | float | Read only | | | -0.000115 | Motor position determination parameters |
| 0X303c | mech_angle_rotat | float | Read only | | | 0 | Motor position determination parameters |
| 0X303d | fault1 | uint32 | Read only | | | 0 | Log failure |
| 0X303e | fault2 | uint32 | Read only | | | 0 | Log failure |
| 0X303f | fault3 | uint32 | Read only | | | 0 | Log failure |
| 0X3040 | fault4 | uint32 | Read only | | | 0 | Log failure |
| 0X3041 | fault5 | uint32 | Read only | | | 0 | Log failure |
| 0X3042 | fault6 | uint32 | Read only | | | 0 | Log failure |
| 0X3043 | fault7 | uint32 | Read only | | | 0 | Log failure |
| 0X3044 | fault8 | uint32 | Read only | | | 0 | Log failure |
| 0X3045 | ElecOffset | float | Read only | | | 0 | Electrical Angle offset |
| 0X3046 | mcOverTemp | int16 | Read only | | | 0 | Overtemperature threshold |
| 0X3047 | Kt_Nm/Amp | float | Read only | | | 0 | Moment coefficient |
| 0X3048 | Tqcali_Type | uint8 | Read only | | | 0 | Motor type |
| 0X3049 | low_position | float | Read only | | | 0 | Motor position determination parameters |
| 0X304a | theta_mech_1 | float | Read only | | | 0 | Type 2 Low speed Angle |
| 0X304b | instep | float | Read only | | | 0 | Motor protection decision parameters |
| 0X304c | adc0ffset_1 | int32 | Read only | | | 2034 | ADC sampling channel 1 zero current bias |
| 0X304d | adc0ffset_2 | int32 | Read only | | | 2040 | ADC sampling channel 2 zero current bias |
| 0X304e | pos_cnt1 | uint16 | Read only | | | 0 | System parameters |
| 0X304f | H' | uint8 | Read only | | | 72 | System parameters |

### 3.3.5 Oscilloscope

The interface supports viewing and observing the graph generated by real-time data, including motor Id/Iq current, temperature, real-time speed at the output end, rotor (encoder) position, output end position, etc.

### 3.3.6 CAN Communication Failure Protection

- When CAN_TIMEOUT is 0, this function is disabled.
- When CAN_TIMEOUT is non-0, when the motor does not receive the CAN command within a certain period of time, the motor enters the reset mode. Unit: 1 = 1s, and 20000 = 1s.

### 3.3.7 Motor Fault Instructions

**Function code 0x3022** indicates the fault code (faultSta):

- **Bit 16:** Motor current fault: A-phase current sampling overcurrent
- **Bit 14:** Motor stall overload algorithm protection
- **Bit 9:** Position initialization fault
- **Bit 8:** Hardware identification fault
- **Bit 7:** Encoder uncalibrated: Motor encoder not calibrated
- **Bit 5:** Motor current fault: C-phase current sampling overcurrent
- **Bit 4:** Motor current fault: B-phase current sampling overcurrent
- **Bit 3:** Overvoltage fault: motor voltage exceeds 60V protection
- **Bit 2:** Undervoltage fault: motor voltage lower than 12V protection
- **Bit 1:** Driver chip failure: Motor driver chip failure reported
- **Bit 0:** Motor overtemperature fault: motor thermistor temperature exceeds 145°C

#### DRV8353xS Fault Status Register 1 (Function code 0x3024)

| Bit | Field | Type | Default | Description |
|---|---|---|---|---|
| 10 | FAULT | R | 0b | Logic OR of FAULT status registers. Mirrors nFAULT pin. |
| 9 | VDS_OCP | R | 0b | Indicates VDS monitor overcurrent fault condition |
| 8 | GDF | R | 0b | Indicates gate drive fault condition |
| 7 | UVLO | R | 0b | Indicates undervoltage lockout fault condition |
| 6 | OTSD | R | 0b | Indicates overtemperature shutdown |
| 5 | VDS_HA | R | 0b | Indicates VDS overcurrent fault on the A high-side MOSFET |
| 4 | VDS_LA | R | 0b | Indicates VDS overcurrent fault on the A low-side MOSFET |
| 3 | VDS_HB | R | 0b | Indicates VDS overcurrent fault on the B high-side MOSFET |
| 2 | VDS_LB | R | 0b | Indicates VDS overcurrent fault on the B low-side MOSFET |
| 1 | VDS_HC | R | 0b | Indicates VDS overcurrent fault on the C high-side MOSFET |
| 0 | VDS_LC | R | 0b | Indicates VDS overcurrent fault on the C low-side MOSFET |

#### DRV8353xS Fault Status Register 2 (Function code 0x3025)

| Bit | Field | Type | Default | Description |
|---|---|---|---|---|
| 10 | SA_OC | R | 0b | Indicates overcurrent on phase A sense amplifier (DRV8353xS) |
| 9 | SB_OC | R | 0b | Indicates overcurrent on phase B sense amplifier (DRV8353xS) |
| 8 | SC_OC | R | 0b | Indicates overcurrent on phase C sense amplifier (DRV8353xS) |
| 7 | OTW | R | 0b | Indicates overtemperature warning |
| 6 | GDUV | R | 0b | Indicates VCP charge pump and/or VGLS undervoltage fault condition |
| 5 | VGS_HA | R | 0b | Indicates gate drive fault on the A high-side MOSFET |
| 4 | VGS_LA | R | 0b | Indicates gate drive fault on the A low-side MOSFET |
| 3 | VGS_HB | R | 0b | Indicates gate drive fault on the B high-side MOSFET |
| 2 | VGS_LB | R | 0b | Indicates gate drive fault on the B low-side MOSFET |
| 1 | VGS_HC | R | 0b | Indicates gate drive fault on the C high-side MOSFET |
| 0 | VGS_LC | R | 0b | Indicates gate drive fault on the C low-side MOSFET |

## 3.4 Control Demo

### 3.4.1 Operation and Control Mode

1. Switch the control mode to operation mode.
2. The motor starts running and enters motor_mode.
3. Set five parameter values (Torque, Angle, Velocity, KP, KD) and click Start or Send continuously. The motor will return feedback frames and run according to the target command.
4. Click Stop to stop the motor and terminate the continuous sending of commands.

### 3.4.2 Current Mode

1. Switch the control mode to current mode.
2. The motor starts running and enters motor_mode.
3. Set the current command value for Iq Command 1 (A). Click the >> button on the right. The motor will follow the current command.
4. Click Stop to stop the motor.

**Sine Wave Test:** Set amplitude and frequency, click OK, then click Start. The corresponding mode target command will be planned according to the sine law.

### 3.4.3 Speed Mode

1. Switch the control mode to speed mode.
2. The motor starts running and enters motor_mode.
3. First, set the current limit (maximum phase current) and speed step value (motor acceleration). If no settings are made, the motor will operate at the default values. Finally, set the speed command (target speed). The motor will follow the command.
4. Click Stop to stop the motor.

### 3.4.4 Position Mode (PP)

1. Switch the control mode to interpolation position mode.
2. Start the motor and enter motor_mode.
3. First, set the speed and acceleration. If not set, the motor will operate at the default values. Finally, set the position command (target position). The motor will follow the command.
4. Set the speed to 0 to stop the motor at the current position. To continue operation, re-issue the speed and position.
5. Click Stop to stop the motor.

### 3.4.5 Location Mode (CSP)

1. Switch the control mode to position mode.
2. The motor starts running and enters motor_mode.
3. Set the speed first. If no speed setting is made, the motor will run at the default value. Finally, set the position command (target position). The motor will follow the command.
4. Click Stop to stop the motor.

---

# Section 4: Driver Protocol and Instructions

The motor communication is the CAN 2.0 communication interface, the baud rate is 1Mbps, and the extended frame format is adopted.

## CAN 2.0 Extended Frame Format (29-bit ID)

| Data Field | 29-bit ID | | | 8-Byte Data Field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | Communication type | data area 2 | Destination address | data area 1 |

### Supported Control Modes

The control modes supported by the motor include:

- **Operation control mode:** set 5 parameters of motor operation control
- **Current mode:** the specified Iq current of the given motor
- **Velocity mode:** the specified running speed of the given motor
- **Position mode:** Given the specified position, the motor will run to the specified position

## 4.1 Description of the Communication Protocol Type

### 4.1.1 Communication Type 0: Get Device ID

Gets the device's ID and 64-bit MCU unique identifier.

#### Send Frame

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x0 | bit15~8: identifies host CAN_ID | target motor CAN_ID | 0 |

#### Reply Frame

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x0 | target motor CAN_ID | 0XFE | 64-bit MCU unique identifier |

### 4.1.2 Communication Type 1: Operation Control Mode

#### Send Frame

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x1 | Byte2: Torque (0~65535) corresponds to (-14Nm~14Nm) | target motor CAN_ID | Byte0~1: target Angle [0~65535] corresponds to (-4π~4π), Byte2~3: Target angular velocity [0~65535] corresponds to (-33rad/s~33rad/s), Byte4~5: Kp [0~65535] corresponds to (0.0~500.0), Byte6~7: Kd [0 to 65535] corresponds to (0.0 to 5.0). High byte first, low byte last. |

**Response Frame:** Communication Type 2 (motor feedback data)

### 4.1.3 Communication Type 2: Motor Feedback Data

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x2 | Bit8~Bit15: CAN ID of motor, bit21~16: fault information (0=none, 1=has), bit21: uncalibrated, bit20: Uncalibrated/Gridlock overload fault, bit19: magnetic coding fault, bit18: overtemperature, bit17: Three-phase overcurrent fault, bit16: undervoltage fault, bit22~23: Mode status 0: Reset [reset], 1: Cali [calibration], 2: Motor [Run] | host CAN_ID | Byte0~1: Current Angle [0~65535] corresponds to (-4π~4π), Byte2~3: Current angular velocity [0~65535] corresponds to (-33rad/s~33rad/s), Byte4~5: Current torque [0~65535] corresponds to (-14Nm~14Nm), Byte6~7: Current temperature: Temp(Celsius)*10. If value > 10, high byte first, low byte last. |

### 4.1.4 Communication Type 3: Motor Enabled to Run

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x3 | bit15~8: identifies the main CAN_ID | and target motor CAN_ID | Byte0~Byte7 |

Response: Communication Type 2 feedback frame.

### 4.1.5 Communication Type 4: Motor Stops Running

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x4 | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | When motor is running normally, 0 must be cleared in the data field. Byte[0]=1: The fault is cleared. |

Response: Communication Type 2 feedback frame.

### 4.1.6 Communication Type 6: Set Motor Mechanical Zero

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x6 | bit15~8: Identifies the main CAN_ID | and target motor CAN_ID | Byte[0]=1 |

Response: Communication Type 2 feedback frame.

### 4.1.7 Communication Type 7: Set Motor CAN_ID

Change the current motor CAN_ID, effective immediately.

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x7 | bit15~8: used to identify main CAN_ID. Bit16~23: preset CAN_ID | Target motor CAN_ID | Byte0~Byte7 |

Answer frame: Communication type 0 broadcast frame.

### 4.1.8 Communication Type 17: Single Parameter Read

#### Send Frame

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x11 | bit15~8: Used to identify the main CAN_ID | target motor CAN_ID | Byte0~1: index. Byte2~3:00 Byte4~7: In data above 00, the low byte is first and the high byte is second |

#### Reply Frame

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x11 | bit15~8: indicates that the master CAN_ID. Bit23~16:00 indicates that the master CAN_ID is successfully read. 01 indicates that the master can_ID | Byte0~1: Byte2~3:00, Byte4~7: Parameter data. 1 byte of data above. Byte4 is preceded by low bytes and followed by high bytes at |

### 4.1.9 Communication Type 18: Single Parameter Write (lost in power failure)

With type 22, the parameter starting with function code 0x20 of the parameter table in the upper computer module can be saved.

#### Send Frame

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x12 | bit15~8: Used to identify the main CAN_ID | target motor CAN_ID | Byte0~1: index. Byte2~3:00 Byte4~7: Parameter data. Low byte first, high byte last. |

Response: Communication Type 2 feedback frame.

### 4.1.10 Communication Type 21: Fault Feedback Frame

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x15 | bit15~8: motor CAN_ID | identifies the main CAN_ID | Byte0~3: fault value (non-0: faulty; 0: Normal). Bit 16: A-phase current sampling overcurrent, Bit 14: Motor stall overload algorithm protection, Bit 9: Position initialization fault, Bit 8: Hardware identification fault, Bit 7: Encoder uncalibrated, Bit 5: C-phase current sampling overcurrent, Bit 4: B-phase current sampling overcurrent, bit3: overvoltage fault, bit2: undervoltage fault, bit1: driver chip fault, bit0: motor overtemperature fault (Default 145°C). Byte4~7: warning Value: bit0: motor overtemperature warning (default is 135°C) |

### 4.1.11 Communication Type 22: Motor Data Save Frame

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x16 | bit15~8: identifies and target | motor CAN_ID | 01 02 03 04 05 06 07 08 |

Response: Communication Type 2 feedback frame.

### 4.1.12 Communication Type 23: Motor Baud Rate Modification Frame (re-power-on effect)

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x17 | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | 01 02 03 04 05 06 F_CMD. F_CMD values: 01=1M, 02=500K, 03=250K, 04=125K |

Response: Communication Type 0 feedback frame.

### 4.1.13 Communication Type 24: The Motor Actively Reports Frames

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x18 | bit15~8: identifies the main CAN_ID | target motor CAN_ID | 01 02 03 04 05 06 F_CMD. F_CMD: 00=disable active reporting (default), 01=enable active reporting (default interval 10ms) |

Response: Same as Type 2 active reporting frame with fault/status information in ID bits.

### 4.1.14 Communication Type 25: Motor Protocol Modification Frame (re-power-on effect)

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x19 | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | 01 02 03 04 05 06 F_CMD. F_CMD: 0=private protocol (default), 1=CANopen protocol, 2=MIT protocol |

Response: Communication Type 0 feedback frame.

### 4.1.15 Communication Type 26: Version Number Read Frame

#### Send Frame

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x4 | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | Byte[0]=0x00, Byte[1]=0xC4 |

#### Response Frame (Type 2)

| Data Field | Bit28~bit24 | bit23~8 | bit7~0 | 8Byte Data |
|---|---|---|---|---|
| Description | 0x2 | Bit8~Bit15: CAN ID of the current motor, bit21~16: fault information, etc. | target motor CAN_ID | Byte0=0x00, Byte1=0xC4, Byte2=0x56, Byte3~6: Motor version number. Order from high to low |

### 4.1.16 Read Example (loc_kp)

#### Read Instruction

| Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
|---|---|---|---|
| 0x11 (Type 17) | Host id 0x00FD | Target motor CAN_ID 0x7F | 1E 70 00 00 00 00 00 00. Byte0~1: index corresponding to loc_kp |

#### Feedback Instruction

| Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
|---|---|---|---|
| 0x11 (Type 17) | bit15~8: Target motor CAN_ID 0x007F | Host id 0xFD | 1E 70 00 00 00 F0 41. Byte0~1: index, corresponding to loc_kp. Byte4~7: loc_kp value 30, high right byte, (32-bit single precision) hexadecimal IEEE-754 standard floating point number |

## 4.2 Read and Write Parameter Index Table

| Index | Name | Description | Type | Number of Bytes | Range/Default | R/W |
|---|---|---|---|---|---|---|
| 0x7005 | run_mode | 0: operation mode, 1: position mode (PP), 2: Velocity mode, 3: Operation mode Current mode, 5: Position mode (CSP) | uint8 | 1 | | W/R |
| 0x7006 | iq_ref | Current mode Iq command | float | 4 | -16 to 16A | W/R |
| 0x700A | spd_ref | Rotational Velocity mode Rotational speed command | float | 4 | -33 to 33rad/s | W/R |
| 0x700B | limit_torque | torque limit | float | 4 | 0 to 14Nm | W/R |
| 0x7010 | cur_kp | Kp | float | 4 | default 0.17 | W/R |
| 0x7011 | cur_ki | Ki | float | 4 | default 0.012 | W/R |
| 0x7014 | cur_filt_gain | filt_gain | float | 4 | 0 to 1.0, default 0.1 | W/R |
| 0x7016 | loc_ref | Position Mode Angle instruction | float | 4 | rad | W/R |
| 0x7017 | limit_spd | Location mode (CSP) speed limit | float | 4 | 0 to 33rad/s | W/R |
| 0x7018 | limit_cur | Velocity position mode Current limitation | float | 4 | 0 to 16A | W/R |
| 0x7019 | mechPos | Mechanical Angle of the loading coil | float | 4 | rad | R |
| 0x701A | iqf | iq Filter | float | 4 | -16 to 16A | R |
| 0x701B | mechVel | Speed of the load | float | 4 | -33 to 33rad/s | R |
| 0x701C | VBUS | Bus voltage | float | 4 | V | R |
| 0x701E | loc_kp | kp at | float | 4 | default 40 | W/R |
| 0x701F | spd_kp | speed kp | float | 4 | default 6 | W/R |
| 0x7020 | spd_ki | ki | float | 4 | default 0.02 | W/R |
| 0x7021 | spd_filt_gain | Speed filter value | float | 4 | default 0.1 | W/R |
| 0x7022 | acc_rad | velocity mode acceleration | float | 4 | default 20rad/s^2 | W/R |
| 0x7024 | vel_max | Location mode (PP) speed | float | 4 | default 10rad/s | W/R |
| 0x7025 | acc_set | Location mode (PP) acceleration | float | 4 | default 10rad/s^2 | W/R |
| 0x7026 | EPScan_time | Report time. 1=10ms. Plus 1 increments by 5ms | uint16 | 2 | default 1 | W/R |
| 0x7028 | canTimeout | CAN timeout threshold, 20000=1s | uint32 | 4 | default 0 | W/R |
| 0x7029 | zero_sta | zero flag bit, 0 means 0-2π, 1 means -π-π | uint8 | 1 | default 0 | W/R |
| 0x702A | damper | Damping switch | uint8 | 1 | default 0 | W/R |
| 0x702B | add_offset | Zero offset | float | 4 | default 0 | W/R |

## 4.3 Motor Function Description

### 4.3.1 Active Reporting

- Disabled by default. Enable via Type 24.
- Report type: Type 2 (default interval: 10ms). Adjust interval by modifying EPScan_time via Type 18.

### 4.3.2 Zero-Point Flag (zero_sta)

- Modify via Host computer or Type 18 (requires saving via Type 22 for communication).
- Default flag: 0 → Power-on position range: 0–2π.
- If set to 1: Power-on position range: -π–π.

### 4.3.3 Type 2 Update

- Updated to periodic looping within -4π–4π (enables cycle counting).
- Note: Position interface parameters must be adjusted: P_MIN: 12.57f, P_MAX: 12.57f

### 4.3.4 Protocol Switching (Requires CAN adapter)

- Methods: Modify protocol_1 via host computer, or Send Type 25 command.
- Reboot required after switching.
- Post-switch CAN commands: CANopen: Send extended frame (protocol switch frame). MIT Protocol: Send standard frame (Command 8).

### 4.3.5 Post-Power-Off Anti-Backdrive Protection

- Default: Motor imposes damping if rotated rapidly while powered off (prevents surge).
- Disable: Set damper = 1.

### 4.3.6 Zero Calibration Rules

- Supported modes: CSP and Motion Control.
- PP Mode: Zero calibration is blocked.
- Old vs. New Versions:
  - Old: Zero calibration causes large deviation → motor immediately moves to target.
  - New (CSP/Motion Control): Target updates to 0 instantly → motor remains stationary.

### 4.3.7 Position Offset (add_offset)

- Example: If offset = 1, the current zero shifts to (current position + 1 rad).
- Use case: Bypass mechanical limits (e.g., set zero at 1 rad → power-on treats 1 rad as new zero).

### 4.3.8 CANopen ID

- Old version: Fixed to 1.
- New version: Matches the private protocol CAN ID.

### Implementation Notes

- Always save settings (e.g., Type 22 for zero_sta).
- Verify CAN adapter compatibility for protocol switching.
- For zero offsets, ensure mechanical safety limits are respected.

## 4.4 Control Mode Instructions

### 4.4.1 Operation Control Mode

**Control formula:** `t_ref = Kd * (v_vset - v_actual) + Kp * (p_set - p_actual) + t_ff`

Tref is converted to the expected iq current through an internal formula and output through the current loop.

#### Simple Control Demonstrations

- **Velocity control:** Set t_ff=0, v_vset=1, Kd=1, p_set=0, Kp=0 → motor runs at 1rad/s (no external load). Increase kd if there's external load.
- **Damping mode:** Set t_ff=0, v_vset=0, Kd=1, p_set=0, Kp=0 → motor generates electricity when externally rotated; requires power supply to prevent overvoltage.
- **Position control:** Set t_ff=0, v_vset=0, Kd=1, p_set=5, Kp=1 → motor runs to position 5. Increasing kp increases holding force; kd provides damping.

#### Sequence

Enable (Type 3) → Send operation control (Type 1) → Receive feedback (Type 2)

### 4.4.2 Current Mode

Sequence: Set runmode to 3 (Type 18) → Enable (Type 3) → Set iq_ref (Type 18) as default current instruction

### 4.4.3 Velocity Mode

Sequence: Set runmode to 2 (Type 18) → Enable (Type 3) → Set limit_cur (Type 18) → Set acc_rad (Type 18) → Set spd_ref (Type 18)

### 4.4.4 Location Mode (CSP)

Sequence: Set runmode to 5 (Type 18) → Enable (Type 3) → Set limit_spd (Type 18) → Set loc_ref (Type 18)

### 4.4.5 Location Mode (PP)

Sequence: Set runmode to 1 (Type 18) → Enable (Type 3) → Set vel_max (Type 18) → Set acc_set (Type 18) → Set loc_ref (Type 18)

**Note:** This mode does not support changing the speed and acceleration during operation. For emergency stop, change vel_max to 0.

### 4.4.6 Stop Running

Sending motor stop frame (communication type 4)

## 4.5 Program Sample

Examples for gd32f303 microcontroller:

```c
#define P_MIN -12.57f  //0.0.2.6 is 12.5 before and 12.57 after
#define P_MAX 12.57f
#define V_MIN 33.0f
#define V_MAX 33.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -14.0f
#define T_MAX 14.0f

struct exCanIdInfo{
  uint32_t id:8;
  uint32_t data:16;
  uint32_t mode:5;
  uint32_t res:3;
};

can_receive_message_struct rxMsg;
can_trasnmit_message_struct txMsg={
  .tx_sfid = 0,
  .tx_efid = 0xff,
  .tx_ft = CAN_FT_DATA,
  .tx_ff = CAN_FF_EXTENDED,
  .tx_dlen = 8,
};

#define txCanIdEx (*((struct exCanIdInfo*)&(txMsg.tx_efid)))
#define rxCanIdEx (*((struct exCanIdInfo*)&(rxMsg.rx_efid)))

int float_to_uint(float x, float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  if(x > x_max) x=x_max;
  else if(x < x_min) x= x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

#define can_txd() can_message_transmit(CAN0, &txMsg)
#define can_rxd() can_message_receive(CAN0, CAN_FIFO1, &rxMsg)
```

### 4.5.1 Motor Enabled Run Frame (Communication Type 3)

```c
void motor_enable(uint8_t id, uint16_t master_id)
{
  txCanIdEx.mode = 3;
  txCanIdEx.id = id;
  txCanIdEx.res = 0;
  txCanIdEx.data = master_id;
  txMsg.tx_dlen = 8;
  txCanIdEx.data = 0;
  can_txd();
}
```

### 4.5.2 Operation Control Mode Motor Control Instruction (Communication Type 1)

```c
void motor_controlmode(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd)
{
  txCanIdEx.mode = 1;
  txCanIdEx.id = id;
  txCanIdEx.res = 0;
  txCanIdEx.data = float_to_uint(torque,T_MIN,T_MAX,16);
  txMsg.tx_dlen = 8;
  txMsg.tx_data[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8;
  txMsg.tx_data[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16);
  txMsg.tx_data[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;
  txMsg.tx_data[3]=float_to_uint(speed,V_MIN,V_MAX,16);
  txMsg.tx_data[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;
  txMsg.tx_data[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);
  txMsg.tx_data[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;
  txMsg.tx_data[7]=float_to_uint(kd,KD_MIN,KD_MAX,16);
  can_txd();
}
```

### 4.5.3 Motor Stop Frame (Communication Type 4)

```c
void motor_reset(uint8_t id, uint16_t master_id)
{
  txCanIdEx.mode = 4;
  txCanIdEx.id = id;
  txCanIdEx.res = 0;
  txCanIdEx.data = master_id;
  txMsg.tx_dlen = 8;
  for(uint8_t i=0;i<8;i++)
  {
    txMsg.tx_data[i]=0;
  }
  can_txd();
}
```

### 4.5.4 Motor Mode Parameter Write Command (Communication Type 18, Running Mode Switch)

```c
uint8_t runmode;
uint16_t index;
void motor_modechange(uint8_t id, uint16_t master_id)
{
  txCanIdEx.mode = 0x12;
  txCanIdEx.id = id;
  txCanIdEx.res = 0;
  txCanIdEx.data = master_id;
  txMsg.tx_dlen = 8;
  for(uint8_t i=0;i<8;i++)
  {
    txMsg.tx_data[i]=0;
  }
  memcpy(&txMsg.tx_data[0],&index,2);
  memcpy(&txMsg.tx_data[4],&runmode, 1);
  can_txd();
}
```

### 4.5.5 Motor Mode Parameter Write Command (Communication Type 18, Control Parameter Write)

```c
uint16_t index;
float ref;
void motor_write(uint8_t id, uint16_t master_id)
{
  txCanIdEx.mode = 0x12;
  txCanIdEx.id = id;
  txCanIdEx.res = 0;
  txCanIdEx.data = master_id;
  txMsg.tx_dlen = 8;
  for(uint8_t i=0;i<8;i++)
  {
    txMsg.tx_data[i]=0;
  }
  memcpy(&txMsg.tx_data[0],&index,2);
  memcpy(&txMsg.tx_data[4],&ref,4);
  can_txd();
}
```

---

# Section 5: Explanation of CANopen Communication Protocol Types

## 5.1 Introduction to CANopen Communication

CANopen is a higher-level protocol based on the CAN bus. CAN bus (ISO 11898) serves as the transportation vehicle for CANopen information. CAN implements the transmission of frames with an 11-bit CAN ID, a Remote Transmission (RTR) bit, and 64 data bits. CANopen implements Layer 7 of the OSI model and is compatible with other data link layer protocols besides CAN. Download EDS files from www.robstride.com.

## 5.2 CANopen Protocol Message Classification

| Classification | Function |
|---|---|
| NMT (Network Management Messages) | Manage the network and switch node states. Typically sent by the master station. |
| SDO (Service Data Object Message) | Used for setting device parameters or transmitting critical data. Master initiates, slave responds. |
| PDO (Process Data Object) messages | Transmit process data from various devices (temperature, speed). Both master and slave send. |
| EMCY (Emergency Message) | Transmits fault information. Both master and slave stations send. |
| SYNC (Synchronization Message) | Synchronization data, used to synchronize TPDO data. Generally sent by master station. |
| NODE GUARDING Message | Master requests slave's status; slave responds. |
| Heartbeat Message | A device actively sends a heartbeat to indicate it is online. Both can send. |

## 5.3 State Machine Description

### State Machine Flow

```
POWER DISABLED
    |
    v
START
    |
    v
NOT READY TO SWITCH ON
    |
    v
SWITCH ON DISABLED
    |
    | (Controlword 6)
    v
READY TO SWITCH ON
    |
    | (Controlword 7)
    v
SWITCHED ON
    |
    | (Controlword 15)
    v
OPERATION ENABLE
    |
    | (Controlword 11)
    v
QUICK STOP ACTIVE
    |
    | (Controlword 15)
    v
back to OPERATION ENABLE

FAULT
    |
    v
FAULT REACTION ACTIVE
    |
    | (Controlword 130)
    v
SWITCH ON DISABLED
```

### Motor Operations

**Motor Enable:**
When initially powered on, the motor defaults to the SWITCH_ON_DISABLED state. To transition to OPERATION_ENABLE, modify the Controlword (6040H) to 6, 7, or 15 (step-by-step transition), or directly set it to 15 for immediate enablement.

**Stopping the Motor:**
If the motor is in OPERATION_ENABLE state, modify the Controlword (6040H) to 1. The motor will return to SWITCH_ON_DISABLED.

**Emergency Stop (Use with Caution—Risk of Voltage Surge):**
Set Controlword (6040H) to 11.

**Fault Clearance:**
If the motor enters a FAULT state, modifying the Controlword (6040H) to 130 can clear standard errors.

**Important Note:** Mode changes must be performed in the disabled state (SWITCH_ON_DISABLED). Ensure the desired mode is configured before enabling OPERATION_ENABLE.

## 5.4 Status Feedback Parameters

| Index | Name | Attribute | Type | Unit |
|---|---|---|---|---|
| 603F | Error_code | Read-only | UINTEGER16 | / |
| 6041 | Statusword | Read-only | UINTEGER16 | / |
| 6061 | Modes_of_operation_display | Read-only | INTEGER8 | / |
| 6062 | Position_demand_value | Read-only | INTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 6064 | Position_actual_value | Read-only | INTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 606B | Velocity_demand_value | Read-only | INTEGER32 | 0.1 rpm |
| 606C | Velocity_actual_value | Read-only | INTEGER32 | 0.1 rpm |
| 6077 | Torque_actual_value | Read-only | INTEGER16 | 0.1% load ratio (1000 = 5 N·m) |
| 6078 | Current_actual_value | Read-only | INTEGER16 | mA |
| 6079 | DC_link_circuit_voltage | Read-only | INTEGER32 | mV |

## 5.5 Homing Mode (Zero Position Setting)

| Index | Name | Attribute | Type | Unit |
|---|---|---|---|---|
| 6040 | Controlword | Read-write | UINTEGER16 | / |
| 6060 | Modes_of_operation | Read-write | INTEGER8 | / |

### Homing Method

- Set Modes_of_operation to 6 while the motor is in the disabled state (SWITCH_ON_DISABLED). The motor will define the current position as the zero point.
- To hold the zero position, modify the Controlword to 15.

## 5.6 Position Mode (PP - Profile Position)

| Index | Name | Attribute | Type | Unit |
|---|---|---|---|---|
| 6040 | Controlword | Read-write | UINTEGER16 | / |
| 6060 | Modes_of_operation | Read-write | INTEGER8 | / |
| 6067 | Position_window | Read-write | UINTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 6068 | Position_window_time | Read-write | UINTEGER16 | ms |
| 6071 | Target_torque | Read-write | INTEGER16 | 0.1% load ratio (1000 = 5 N·m) |
| 607A | Target_position | Read-write | INTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 6081 | Profile_velocity | Read-write | UINTEGER32 | 0.1 rpm |
| 6083 | Profile_acceleration | Read-write | UINTEGER32 | 0.1 rpm/s |

### Steps to Configure Position Mode (PP)

1. While motor is in SWITCH_ON_DISABLED, set Modes_of_operation to 1.
   - Mandatory: Target_torque, Profile_velocity, Profile_acceleration
   - Optional: Position_window, Position_window_time
2. Set Controlword (6040) to 15 to enable operation.
3. Set Target_position to move the motor.

## 5.7 Position Mode (CSP - Cyclic Synchronous Position)

| Index | Name | Attribute | Type | Unit |
|---|---|---|---|---|
| 6040 | Controlword | Read-write | UINTEGER16 | / |
| 6060 | Modes_of_operation | Read-write | INTEGER8 | / |
| 6067 | Position_window | Read-write | UINTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 6068 | Position_window_time | Read-write | UINTEGER16 | ms |
| 6071 | Target_torque | Read-write | INTEGER16 | 0.1% load ratio (1000 = 5 N·m) |
| 607A | Target_position | Read-write | INTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 6081 | Profile_velocity | Read-write | UINTEGER32 | 0.1 rpm |

### Steps to Configure Position Mode (CSP)

1. While motor is in SWITCH_ON_DISABLED, set Modes_of_operation to 5.
   - Mandatory: Target_torque, Profile_velocity
   - Optional: Position_window (0 = disabled), Position_window_time (0 = disabled)
2. Set Controlword (6040) to 15 to enable operation.
3. Set Target_position to move the motor.

## 5.8 Velocity Mode

| Index | Name | Attribute | Type | Unit |
|---|---|---|---|---|
| 6040 | Controlword | Read-write | UINTEGER16 | / |
| 6060 | Modes_of_operation | Read-write | INTEGER8 | / |
| 6071 | Target_torque | Read-write | INTEGER16 | 0.1% load ratio (1000 = 5 N·m) |
| 60FF | Target_velocity | Read-write | INTEGER32 | 0.1 rpm |

### Steps to Configure Velocity Mode

1. While motor is in SWITCH_ON_DISABLED, set Modes_of_operation to 3.
   - Mandatory: Target_torque
2. Set Controlword (6040) to 15 to enable operation.
3. Set Target_velocity to reach the desired speed.

## 5.9 Torque Mode

### Steps to Configure Torque Mode

1. While motor is in SWITCH_ON_DISABLED, set Modes_of_operation to 4.
2. Set Controlword (6040) to 15 to enable operation.
3. Set Target_torque to output the desired torque.

## 5.10 Protocol Switching (Extended Frame)

| Data Field | 29-bit ID | 8-Byte Data Area |
|---|---|---|
| Size | Bit 28~0 | Byte 0~6 |
| Description | 0xFFF | 01 02 03 04 05 06 F_CMD |

**F_CMD (Byte 6) defines the motor protocol:**
- 0: Private protocol (default)
- 1: CANopen protocol
- 2: MIT protocol

### Response Frame

| Data Field | 11-bit ID | 8-Byte Data Area |
|---|---|---|
| Description | Motor ID | 64-bit MCU unique identifier |

---

# Section 6: MIT Communication Protocol Description

The motor communication adopts the CAN 2.0 interface with a default baud rate of 1 Mbps. The baud rate can be modified by switching to the private protocol.

## Standard Frame Format

| Data Field | 11-bit ID | | 8-byte Data Area |
|---|---|---|---|
| Size | Bit 10~8 | Bit 7~0 | Byte 0~7 |
| Description | Mode type | ID | |

### Supported Control Modes

- **MIT Mode:** Provides five motion control parameters to the motor.
- **Velocity Mode:** Specifies the target speed for the motor.
- **Position Mode:** Specifies the target position and speed.

## 6.1 Response Command 1: Data Feedback (Motor Status)

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | Byte 0~7 |
| Description | Host ID | Byte 0: Motor CAN ID. Byte 1~2: Current angle [0~65535], corresponds to (-12.57 rad ~ 12.57 rad). Byte 3 (high 8 bits), Byte 4[7-4] (low 4 bits): Current speed [0~4096], corresponds to (-33 rad/s ~ 33 rad/s). Byte 4[3-0] (high 4 bits), Byte 5 (low 8 bits): Current torque [0~4096], corresponds to (-14 N·m ~ 14 N·m). Byte 6~7: Winding temperature (Celsius) * 10 |

## 6.2 Response Command 2: MCU Identification

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Description | Motor ID | 64-bit MCU unique identifier |

## 6.3 Command 1: Enable Motor Operation

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | Byte 0~7 |
| Description | Target motor CAN ID | FF FF FF FF FF FF FF FC |

Response: Response Command 1

## 6.4 Command 2: Stop Motor Operation

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | Byte 0~7 |
| Description | Target motor CAN ID | FF FF FF FF FF FF FF FD |

Response: Response Command 1

## 6.5 Command 3: MIT Dynamic Parameters

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | Byte 0~7 |
| Description | Target motor CAN ID | Byte 0~1: Target angle [0~65535], (-12.57 rad ~ 12.57 rad). Byte 2 (high 8 bits), Byte 3[7-4] (low 4 bits): Target speed [0~4096], (-33 rad/s ~ 33 rad/s). Byte 3[3-0] (high 4 bits), Byte 4 (low 8 bits): Kp [0~4096], (0~500). Byte 5 (high 8 bits), Byte 6[7-4] (low 4 bits): Kd [0~4096], (0~5). Byte 6[3-0] (high 4 bits), Byte 7 (low 8 bits): Target torque [0~4096], (-14 N·m ~ 14 N·m) |

Response: Response Command 1

## 6.6 Command 4: Set Zero Position (Non-Position Mode)

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Description | Target motor CAN ID | FF FF FF FF FF FF FF FE |

Response: Response Command 1

## 6.7 Command 5: Clear Errors & Read Fault Status

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | FF FF FF FF FF FF F_CMD FBF_CMD |

- F_CMD = 0xFF → Clear current fault
- Any other value → Returns fault value in Byte 1 of the response

Response (Fault Clear): Response Command 1

## 6.8 Fault Status Response

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | Byte 0: Motor CAN ID. Byte 1~4: Fault value (Non-zero = Fault present; 0 = Normal). Bit 14: Stall/I²t overload fault. Bit 7: Encoder not calibrated. Bit 3: Overvoltage fault. Bit 2: Undervoltage fault. Bit 1: Driver IC fault. Bit 0: Motor overtemperature fault (Default threshold: 145°C) |

## 6.9 Command 6: Set Operation Mode

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | FF FF FF FF FF FF F_CMD FCF_CMD |

**F_CMD: Mode type**
- 0: MIT mode (default)
- 1: Position mode
- 2: Velocity mode

Response: Response Command 1

## 6.10 Command 7: Modify Motor CAN ID

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | FF FF FF FF FF FF F_CMD FAF_CMD: Target motor CAN ID |

Response: Response Command 2

## 6.11 Command 8: Change Communication Protocol (Takes Effect After Power Cycle)

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | FF FF FF FF FF FF F_CMD FDF_CMD: Protocol type |

**F_CMD: Protocol type**
- 0: Private protocol (default)
- 1: CANopen
- 2: MIT protocol

Response: Response Command 2

## 6.12 Command 9: Modify Host CAN ID

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | FF FF FF FF FF FF F_CMD 01F_CMD: Host CAN ID |

Response: Response Command 2

## 6.13 Command 10: Position Mode Control Command

| Data Field | 11-bit ID | | 8-byte Data Area |
|---|---|---|---|
| Size | Bit 10~8 | Bit 7~0 | Byte 0~7 |
| Description | 1 | Target motor CAN ID | Byte 0~3: Target position (rad, 32-bit float). Byte 4~7: Target speed (rad/s, 32-bit float) |

Response: Response Command 1

## 6.14 Command 11: Velocity Mode Control Command

| Data Field | 11-bit ID | | 8-byte Data Area |
|---|---|---|---|
| Size | Bit 10~8 | Bit 7~0 | Byte 0~7 |
| Description | 2 | Target motor CAN ID | Byte 0~3: Target speed (rad/s, 32-bit float). Byte 4~7: Current limit in speed/position mode (A, 32-bit float) |

Response: Response Command 1

## 6.15 Motion Control Mode (MIT Protocol)

The motor defaults to Motion Control Mode upon power-up.

1. Send the Motor Enable Command (Command 1).
2. Send the Motion Control Command (Command 3) to activate dynamic parameter control.
3. Send the Motor Stop Command (Command 2) to halt operation when needed.

## 6.16 Velocity Mode (MIT Protocol)

1. Configure operation mode by sending Set Operation Mode Command (Command 6) with Mode = 2 (Velocity Mode).
2. Send Motor Enable Command (Command 1) to activate the motor.
3. Send the Velocity Mode Control Command (Command 11) to set the maximum current (absolute value) and target speed.
4. To stop, send the Motor Stop Command (Command 2).

## 6.17 Position Mode (CSP - Cyclic Synchronous Position) (MIT Protocol)

1. Configure operation mode by sending Set Operation Mode Command (Command 6) with Mode = 1 (Position Mode).
2. Send Motor Enable Command (Command 1) to activate the motor.
3. Send the Position Mode Control Command (Command 10) to set the maximum speed (absolute value) and target position.
4. To stop, send the Motor Stop Command (Command 2).

---

# Section 7: Version History

| Version Number | Description | Date |
|---|---|---|
| 1.0 | Initial Release | November 25, 2025 |

---

## Document Information

This manual covers the RobStride RS00 - 14N.M Quasi-Direct Drive Integrated Motor Module. For the most up-to-date information and firmware updates, visit https://www.robstride.com/.
