# nucleo-h743zi2_playground
[WIP] My NUCLEO-H743ZI2 playground for tflite-micro micro_speech and etc.

## About  
* See h743_inmp441_v2_run_tflm_no_mic_success.rar  
* Original from https://github.com/KeilChris/TensorFlow_MIMXRT1064-EVK_Microspeech   
* Also see https://github.com/weimingtom/rt1020-evk-playground  

## (Not implemented and used yet, but tested) How to link to inmp441 mems microphone with SAI       
```
串口不用接，默认被连接到st-link的虚拟串口上
usart3, TX, PD8(not need), USART3_TX
usart3, RX, PD9(not need), USART3_RX

INMP441
SCK,WS,LR
[===]xxxxxxRRRRR
SD,VDD,GND

INMP441<->STM32F446
SCK(left top 1)<->SAI1_SCK_B, PF8 (left 4 bottom 4)
SD(right top 1)<->SAI1_SD_B, PE3 (left 4 bottom 5)
WS(left top 2)<->SAI1_FS_B, PF9 (left 4 bottom 2)
VDD(right top 2)<->3.3(left 3 top 4)
L/R(left top 3)<->GND
GND(right top 3)<->GND(left 3 top 7)
```

