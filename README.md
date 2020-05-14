# M031BSP_PWM_1Hz
 M031BSP_PWM_1Hz

update @ 2020/05/14

- Add define ENABLE_PWM_CH0 , ENABLE_PWM_CH4 to switch to PB.5 (PWM_CH0) or PB.1 (PWM0_CH4) 

- Enable HXT configuration


update @ 2020/04/30

- Set PA.5 (PWM0_CH0) Freq as 1Hz , change duty with 0.1% (resolution 1000) per 500 ms

- reverse duty increase or decrease when reach 0% or 100%

- Toggle PB.14 (M032 EVM LED) to monitor as channel 2 in below scope waveform

![image](https://github.com/released/M031BSP_PWM_1Hz/blob/master/FREQ_1Hz_RESOLUTION_1K.jpg)

