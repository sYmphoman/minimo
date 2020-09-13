# SmartLCD Minimotors
Warning : this is work in progress. I decline all responsability about using informations from this project.

# What is SmartLCD

It's a combo of electronic and smartphone application to extend all features from original Minimorots EYE LCD and controller.

## What can be done ?

I suggest you to have a look at all planed features below.
There is almost no limit in custom features.
The controller power cannot exceed the nominal power, but with a shunt, you can go upper and still have a current control loop to limit the current by software (and avoid burning the controller).

Most settings will be configurable by the smartphone, and additionnal buttons will allow you to control specific features.

## Main features

![Idea](/SmartLCD.png)

## Electronic
### Software
The software is designed for ESP32 Dev Kit v4.

### Schematics
https://easyeda.com/Koxx3/bettercontroller

## Inspiration to understand controler electronic
http://avdweb.nl/Article_files/Solarbike/Motor-controller/China-BLDC-motor-controller-36V-250W.pdf

## TODO / Done
- [X] Serial Minimotors
    - [X] Read/write serial link LCD_TO_CNTRL
    - [X] Read/write serial link CNTRL_TO_LCD
    - [X] Decode speed/mode/brake/regulator from serial link
- [ ] Serial Kaabo / Zero / Appolo
- [ ] Bluetooth 
    - [X] communication with Android
    - [X] Bluetooth anti-theeth with smartphone
    - [X] Bluetooth anti-theeth with beacon (scan & rssi detection)
    - [ ] Bluetooth devices parameters and security
- [ ] Other inputs/outputs
    - [X] Read current with WCS1700
    - [X] Read battery voltage
    - [ ] Read break handle position
    - [ ] Read tempertature/humidity with DHT11
- [ ] Features    
    - [ ] Progressive electric braking (in progress)
    - [ ] Customize modes with different power / max speed
    - [ ] Remote physical button to change mode (or other features)
    - [ ] Current loop regulation
    - [ ] Save settings in non volatile memory
    - [ ] Custom acceleration curve
    - [ ] Data logging
- [ ] Android app
    - [X] display : speed, mode, voltage, current, power, brake mode (PA)
    - [ ] parameters override (P5/P6/P7/)
    - [ ] parameters custom settings
    - [ ] display : time, moving time, 
    - [ ] auto-launch app with NFC tag
    - [ ] history graphics

- [ ] iPhone app => for someone else

## Serial links data decoding
[Excel](http://github.com/Koxx3/minimo/edit/master/MINIMO.xlsx

## Donate to support
https://www.paypal.com/donate/?cmd=_s-xclick&hosted_button_id=W3KHBZCNL9N2C&source=url
