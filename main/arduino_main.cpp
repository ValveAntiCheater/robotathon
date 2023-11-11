/****************************************************************************
Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
GIT PUSH (Uploading your changes on the repository) Instruction:
    Make changes
    Open Git bash
    Open cd robotathon
    git status
    git add main/arduino_main.cpp
    git commit -m "(Add your message about change in here)"
    git push
****************************************************************************/

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <Bluepad32.h>

#include <ESP32Servo.h>
#include <ESP32SharpIR.h>
#include <QTRSensors.h>

//Color Sensor headers
#include <Wire.h>
#include <Arduino_APDS9960.h>
#include <bits/stdc++.h>

//Color sensor definitions
#define APDS9960_INT 2
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 100000

//Color sensor unit & I2C unit
TwoWire I2C_0 = TwoWire(0);
APDS9960 apds = APDS9960(I2C_0, APDS9960_INT);

//IR sensor (Distance) header
#include <ESP32SharpIR.h>

// Distance Sensor unit
ESP32SharpIR leftSensor(ESP32SharpIR::GP2Y0A21YK0F, 18);
ESP32SharpIR rightSensor(ESP32SharpIR::GP2Y0A21YK0F, 19);
ESP32SharpIR centerSensor(ESP32SharpIR::GP2Y0A21YK0F, 17);


//
// README FIRST, README FIRST, README FIRST
//
// Bluepad32 has a built-in interactive console.
// By default it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

// Setting LED Variable to 2
int LED = 2;

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            // Console.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            // GamepadProperties properties = gp->getProperties();
            // Console.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n", gp->getModelName(), properties.vendor_id,
            //                properties.product_id);
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        // Console.println("CALLBACK: Gamepad connected, but could not found empty slot");
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            // Console.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }

    if (!foundGamepad) {
        // Console.println("CALLBACK: Gamepad disconnected, but not found in myGamepads");
    }
}

Servo motor1;
Servo motor2;
Servo arm;
ESP32SharpIR sensor1( ESP32SharpIR::GP2Y0A21YK0F, 27);
QTRSensors qtr;

// Arduino setup function. Runs in CPU 1
void setup() {
    // Console.printf("Firmware: %s\n", BP32.firmwareVersion());
        pinMode(LED, OUTPUT);
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

    //motor
    motor1.setPeriodHertz(50);
    motor1.attach(13, 1000, 2000);
    motor2.setPeriodHertz(50);
    motor2.attach(14, 1000, 2000);
    arm.setPeriodHertz(50);
    arm.attach(, 1000, 2000);
    // motor1.write(1500);
    // motor2.write(1500);

    // Line Sensor
    Serial.begin(115200);
    // sensor1.setFilterRate(0.1f);

    qtr.setTypeAnalog(); // or setTypeAnalog()
    qtr.setSensorPins((const uint8_t[]) {36, 39, 34, 35, 32, 33, 25, 26}, 8);
    for (uint8_t i = 0; i < 50; i++)
    {
        Serial.println("calibrating");
        qtr.calibrate();
        delay(20);
    }

    //Sets up I2C protocol
    I2C_0.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

    // Set up color sensor
    apds.setInterruptPin(APDS9960_INT);
    apds.begin();
    Serial.begin(115200);

    //IR (Distance) Sensor Setup
    leftSensor.setFilterRate(0.1f);
    rightSensor.setFilterRate(0.1f);
    centerSensor.setFilterRate(0.1f);
    
}

// Arduino loop function. Runs in CPU 1
void loop() {
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.

    //Light
    digitalWrite(LED, HIGH);
    // delay(1000);
    // digitalWrite(LED, LOW);
    // delay(1000);

    // Code for Controller
    BP32.update();
    GamepadPtr controller = myGamepads[0];

    // Code for Motor
    
    // motor1.write(1600);
    // motor2.write(1400);
    
    
    if (controller && controller->isConnected()) {
       // Controlling motor
        motor1.write(((((float) controller->axisY()) / 512.0f) * 500) + 1500);
        motor2.write(((((float) controller->axisY()) / 512.0f) * 500 ) + 1500);
    }
    vTaskDelay(1);


    // It is safe to always do this before using the gamepad API.
    // This guarantees that the gamepad is valid and connected.
    // for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    //     GamepadPtr myGamepad = myGamepads[i];

    //     if (myGamepad && myGamepad->isConnected()) {

    //         servo.write( ((((float) myGamepad->axisY()) / 512.0f) * 500) + 1500 );

            // Another way to query the buttons, is by calling buttons(), or
            // miscButtons() which return a bitmask.
            // Some gamepads also have DPAD, axis and more.
            // Console.printf(
            //     "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, "
            //     "%4d, brake: %4d, throttle: %4d, misc: 0x%02x\n",
            //     i,                        // Gamepad Index
            //     myGamepad->dpad(),        // DPAD
            //     myGamepad->buttons(),     // bitmask of pressed buttons
            //     myGamepad->axisX(),       // (-511 - 512) left X Axis
            //     myGamepad->axisY(),       // (-511 - 512) left Y axis
            //     myGamepad->axisRX(),      // (-511 - 512) right X axis
            //     myGamepad->axisRY(),      // (-511 - 512) right Y axis
            //     myGamepad->brake(),       // (0 - 1023): brake button
            //     myGamepad->throttle(),    // (0 - 1023): throttle (AKA gas) button
            //     myGamepad->miscButtons()  // bitmak of pressed "misc" buttons
            // );

    //         // You can query the axis and other properties as well. See Gamepad.h
    //         // For all the available functions.
    //     }
    // }

    // Serial.println(sensor1.getDistanceFloat());
    
    // Line Sensor 
    uint16_t sensors[3];
    int16_t position = qtr.readLineBlack(sensors);
    int16_t error = position - 3500;
    Serial.println(position);

    if (error < 0)
    {
        Serial.println("On the left");
        motor2.write(1500);
        motor1.write(1600);
    }
    if (error > 0)
    {
        Serial.println("On the right");
        motor1.write(1500);
        motor2.write(1400);
    }
    if(error == 0){
        Serial.println("Straight Ahead"); 
        motor1.write(1700);
        motor2.write(1300); 
    }
    vTaskDelay(1);

    //Color Sensor
    /*int r, g, b, a;
    //Wait until color is read from the sensor
    while (!apds.colorAvailable()) {
        delay(5);
    }
    // Read color from sensor
    apds.readColor(r, g, b, a);
    // Print color in decimal
    Serial.print("RED: ");
    Serial.println(r);
    Serial.print("GREEN: ");
    Serial.println(g);
    Serial.print("BLUE: ");
    Serial.println(b);
    Serial.print("AMBIENT: ");
    Serial.println(a);*/

    // //IR (Distance) Sensor
    // Serial.println(leftSensor.getDistanceFloat());
    // delay(500);
    // Serial.println(rightSensor.getDistanceFloat());
    // delay(500);
    // Serial.println(centerSensor.getDistanceFloat());
    // delay(500);


    }
