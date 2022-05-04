This project is a result of my reverse engineering of the Hubsan protocol.

I decided to build a receiver for the Hubsan transmitter that I had after H501S Spyhawk model.


## Hardware
 - Arduino model: Mini pro 328p 8Mhz 3.3v
 - Transmitter model: A7105 connected via SPI

## FC commuication 
 In the config.h choose one of commuication protocls (S.Bus, Ibus, PPM) by uncommenting respective line.
 #Note: S.BUS is inverted, as arduino has no options for the inverted Serial.
S.Bus and Ibus are on the TX pin. The PPM is on pin 6
 
## Telemetry
 As of telementry protocol with the FC I chose LTM as the lightest protocol for the arduino.
 So in the INAV/Betaflight, please, choose LTM speed 9600 and connect UART to the pin 3 on Arduino mini (you can define it in LTM.ino file.)
 
 In case you need Mavlink protocol, there is a folder *feature* where I did such integration, but the code itself is outdated.

 This data is displayed natively on the transmitter's screen: GPS position, speed, Angles and the distance to home.
 
## Binding with the transmitter
  - Press and hold "enter" button while powering on the transmitter.
  - Wait when the text "Bind to plane" appears on the screen.
  - Power on the receiver.
  - Wait when transmitter beeps and turns into operation mode.
  - This process is buggy, so probably you need to repeat this step several times.

  Unfortunatelly, Hubsan doesn't support several models, so in case you have several custom receivers you need to repeat binding for each of them.
  
## Control channels

The transmitter has 2 sticks, 4 switches and 2 knobs. Also the ESC and Enter buttons are working as toggle switches.
So they could be used as Acro/Angle mode toggles or as ARM.

## VTX control (not implemented)
Packets that starts with 0x40 byte a used to change the VTX frequency. It is possible to make one more SoftSerial with SmartAudio protocol to adjust VTX setting from the transmitter's menu.
