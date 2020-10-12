## PhotoStamp
A system which can control cameras synchronously, and record messages includes GPS msg and feedback of cameras into SD cards. 

### How to use?

#### Preparing
1. Open this code using Keil Software, compile it. When there is no error, `PhotoStamp.axf` will be created. 
2. Connect MCU Board to computer using USB writer. Be careful to the wire mapping. Download code to Hardware Board.
3. Insert TF card to MCU Board. Connect MCU Board to Hardware module and LED module.

#### Working
1. When MCU board is power on, system will first execute initial process. System will power on each camera in turn. When system received available feedback, it will turn on corresponding led for seconds.
2. After initialization, system will waiting for GPS time message. Status LED will blink slowly. If system received GPS time message, message file will be created, Status LED will blink fastly. If system received GPS location message, Status LED will always on. If system lost GPS message, Status LED will back to blink.
3. If system received trigger singal, it will active all cameras at the same time and waiting for feedback. If message file is created, system will record GPS time and location message and feedback message of cameras. If feedback is available, it will turn on corresponding led for seconds.


### Message

#### Filename

Message file will be created when system is available to get GPS time message(in GNRMC).

Format:
```
[gps_month][gps_day][gps_hour][gps_minute].txt // MMddhhmm.txt
```
Example: ``` 10121558.txt ```

#### Data

Message file should be created first otherwise data record process will be ignored.

Message data will be recorded when system is available to get all cameras' feedback (whether normal or timeout).

Format:
```
seqid, [gps_year][gps_month][gps_day], [gps_hour][gps_minute][gps_second], gps_latitude, gps_longitude, gps_height, cam0_fb, cam1_fb, cam2_fb, cam3_fb, cam4_fb
```

Example: ```8,20200701,191852,31.0448462,121.4394347,28.6,1,1,1,1,1```

### Hardware
#### Camera
Should uncommented `FUNC_CAM` macro in `utils.h`
 - CAMx_OUT: Low level trigger
 - CAMx_IN: Low level feedback
 - CAMx_LED: Low level active

|CAM No.|CAMx_OUT   |CAMx_IN    |CAMx_LED   |
|-      |-          |-          |-          |
|CAM1   |PD8        |PB13       |PB7        |
|CAM2   |PD10       |PE7*       |PB6        |
|CAM3   |PD12       |PE15       |PB9        |
|CAM4   |PD14       |PE13       |PB8        |
|CAM5   |PC6        |PE11       |PE1        |
*: A little bit changed because PCB layout issues

#### Trigger
 - PWM_IN:  PE9
   - should uncommented `TRIG_MODE_PWM` macro in `utils.h`
   - pulse width should between `TRIG_PWM_MIN` and `TRIG_PWM_MAX` 
 - TRIG_IN: PC3(EXTI3)
   - should uncommented `TRIG_MODE_FALLING` macro in `utils.h`
   - low level trigger

#### USART
 - GPS: PA3(Rx)
 - Debug: PA9(Tx) / PA10(Rx)

#### Others
 - KEY: PA0(Onboard)
 - LED: PA1(Onboard) / PE0(Ext)

### Software

#### `gpio.h`
Pin mapping can be modified in this file. Just modify corresponding macro define.

#### `utils.h`
Changeable config for system. Detailed comment is in the code file.

#### Other file ( `main.c` / `gpio.c` / ... )

**DO NOT MODIFY THESE FILES** until you know the actual meaning of the code.


