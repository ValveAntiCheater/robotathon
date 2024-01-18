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
QTRSensors qtr;
// Distance Sensor unit
ESP32SharpIR leftSensor(ESP32SharpIR::GP2Y0A21YK0F, 36);
ESP32SharpIR rightSensor(ESP32SharpIR::GP2Y0A21YK0F, 25);
ESP32SharpIR centerSensor(ESP32SharpIR::GP2Y0A21YK0F, 39);

// Arduino setup function. Runs in CPU 1
void setup() {
    // Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    pinMode(2, OUTPUT);
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
    arm.attach(15, 1000, 2000);
    // motor1.write(1500);
    // motor2.write(1500);

    // Line Sensor
    Serial.begin(115200);
    // sensor1.setFilterRate(0.1f);

    qtr.setTypeAnalog(); // or setTypeAnalog()
    qtr.setSensorPins((const uint8_t[]) {26, 34, 35, 32, 33, 27}, 6);
    for (uint8_t i = 0; i < 110; i++)
    {
        Serial.println("calibrating");
        qtr.calibrate();
        delay(20);
    }
    Serial.println("DONE CALIBRATING");
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
    digitalWrite(2, HIGH);

    // Code for Controller
    BP32.update();
    GamepadPtr controller = myGamepads[0];

    // Code for Motor
    
    // motor1.write(1600);
    // motor2.write(1400);

    

        
    if (controller && controller->isConnected()) {
       // Controlling motor
        if (controller->axisY() < -20 or controller->axisY() > 20)
        {
            // Go straight
            motor1.write(((((float) controller->axisY()) / 512.0f) * 250) + 1500);
            motor2.write(((((float) controller->axisY()) / 512.0f) * -250 ) + 1500);
        }
        else if (controller->axisRY() < -20 or controller->axisRY() > 20)
        {
            // For turning
            motor1.write(((((float) controller->axisRY()) / 512.0f) * 250) + 1500);
            motor2.write(((((float) controller->axisRY()) / 512.0f) * 250 ) + 1500);
        }
        else
        {
            motor1.write(1500);
            motor2.write(1500);
        }
        // Moving Arm
        if (controller->dpad() != 0)
        {
            arm.write(1700);
            delay(800);
            arm.write(1500);
            delay(2000);
            arm.write(1300);
            delay(800);
            arm.write(1500);
        }
        if (controller->x()){
            // Maze subroutine
                Serial.println("Y PRESSED: ENTERING MAZE SUBROUTINE");
                while(1){
                        BP32.update();
                        float left = leftSensor.getDistanceFloat();
                        float right = rightSensor.getDistanceFloat();
                        float center = centerSensor.getDistanceFloat();
                        motor1.write(1400);
                        motor2.write(1600);
                        if (left > right)
                        {
                            motor1.write(1400);
                            motor2.write(1620 + (left - right));
                        }
                        if (right > left)
                        {
                            motor1.write(1380 - (right - left));
                            motor2.write(1600);
                        }
                        if (center < 10)
                        {
                            Serial.println("TURNING TURNING TURNING");
                            Serial.println("RIGHT:");
                            Serial.println(right);
                            Serial.println("LEFT");
                            Serial.println(left);
                            if (right > 15 && left < 15)
                            {
                                motor1.write(1400);
                                motor2.write(1400);
                                delay(1650);
                            }
                            else if (right < 15 && left > 15)
                            {
                                motor1.write(1600);
                                motor2.write(1600);
                                delay(1650);
                            }
                        }
                        // //IR (Distance) Sensor
                        // Serial.println("LEFT");
                        // Serial.println(leftSensor.getDistanceFloat());
                        // delay(5);
                        // Serial.println("RIGHT");
                        // Serial.println(rightSensor.getDistanceFloat());
                        // delay(5);
                        // Serial.println("CENTER");
                        // Serial.println(centerSensor.getDistanceFloat());
                        // delay(5);
                        if (controller->a()){
                            Serial.println("B PRESSED: EXITING MAZE SUBROUTINE");
                            break;
                        }
                }      
        }
        if (controller->y()){
            // Line following subroutine 
            Serial.println("X PRESSED: ENTERING LINE SUBROUTINE");
            while(1){
                BP32.update();
                uint16_t sensors[3];
                int16_t position = qtr.readLineBlack(sensors);
                int16_t error = position - 2500;
                Serial.println(position);
                Serial.println(error);
                delay(500);

                if (error <= -50)
                {
                    Serial.println("On the left");
                    motor1.write(1400);
                    motor2.write(1500);
                }
                if (error >= 50)
                {
                    Serial.println("On the right");
                    motor1.write(1500);
                    motor2.write(1600);
                }
                if(error > -50 && error < 50){
                    Serial.println("Straight Ahead"); 
                    motor1.write(1400);
                    motor2.write(1600); 
                }
                vTaskDelay(1);
                if (controller->a()){
                    Serial.println("B PRESSED: EXITING LINE SUBROUTINE");
                    break;
                }
            }
                
        }
        if (controller->b()){
            // int r,g,b,a;
            // while(1)
            // {
            //     while (!apds.colorAvailable())
            //     {
            //         delay(5);
            //     }
            //     apds.readColor(r,g,b,a);
            //     Serial.println("RED:");
            //     Serial.println(r);
            //     Serial.println("GREEN:");
            //     Serial.println(g);
            //     Serial.println("BLUE:");
            //     Serial.println(b);
            //     Serial.println("AMB");
            //     Serial.println(a);
            //     delay(200);
            //     if(controller->a()){
            //         Serial.println("B PRESSED: EXITING COLOR SUBROUTINE");
            //         digitalWrite(LED, HIGH);
            //         break;
            //     }

            // }
    

            // Color subroutine
            Serial.println("A PRESSED: ENTERING COLOR SUBROUTINE");
            motor1.write(1700);
            motor2.write(1300);
            delay(750);
            motor1.write(1500);
            motor2.write(1500);
            delay(500);
            motor1.write(1700);
            motor2.write(1300);
            delay(2500);
            // int col = 0;
            // int sum = 0;
            // int r,g,b,a;
            // while (!apds.colorAvailable()) {
            //         delay(5);
            //     }
            //     apds.readColor(r, g, b, a);
            //     int thresh = a;
            //     int whitesum = r+g+b+a;
            // int terror = a-thresh;
            // int sumerror = r+g+b+a-whitesum;
            // int botherror = terror+sumerror;
            // while (botherror > 10 && botherror < -10)
            // {
            //     while (!apds.colorAvailable()) {
            //         delay(5);
            //     }
            //     apds.readColor(r, g, b, a);
            //     int terror = a-thresh;
            //     int sumerror = r+g+b+a-whitesum;
            //     botherror = terror+sumerror;
            //     motor1.write(1550);
            //     motor2.write(1450);
                
            // }
            // delay(50);
            // Serial.println("COLOR DETECTED HERE");
            // while (!apds.colorAvailable()) {
            //     delay(5);
            // }
            // apds.readColor(r, g, b, a);
            // if (r > g && r > b)
            //     {
            //         //red
            //         Serial.println("RED");
            //         col = -1;
            //         digitalWrite(LED, LOW);
            //         delay(500);
            //         digitalWrite(LED, HIGH);
            //     }
            //     else if (g > r && g > b)
            //     {
            //         //green
            //         Serial.println("GREEN");
            //         col = 0;
            //         digitalWrite(LED, LOW);
            //         delay(250);
            //         digitalWrite(LED, HIGH);
            //         delay(500);
            //         digitalWrite(LED, LOW);
            //         delay(500);
            //         digitalWrite(LED, HIGH);
            //     }
            //     else if (b > r && b > g)
            //     {
            //         //blue
            //         Serial.println("BLUE");
            //         col = 1;
            //         digitalWrite(LED, LOW);
            //         delay(250);
            //         digitalWrite(LED, HIGH);
            //         delay(500);
            //         digitalWrite(LED, LOW);
            //         delay(250);
            //         digitalWrite(LED, HIGH);
            //         delay(500);
            //         digitalWrite(LED, LOW);
            //         delay(500);
            //         digitalWrite(LED, HIGH);
            //     }
            //     while (1)
            //     {
            //         while (!apds.colorAvailable()) {
            //         delay(5);
            //         }
            //         apds.readColor(r, g, b, a);
            //         if (col < 0)
            //         {
            //             motor1.write(1550);
            //             motor2.write(1450);
            //             if (r * 1.5 > g && r * 1.5 > b)
            //             {
            //                 motor1.write(1500);
            //                 motor2.write(1500);
            //                 break;
            //             }
            //         }
            //         if (col == 0)
            //         {
            //             motor1.write(1550);
            //             motor2.write(1450);
            //             if (g * 1.5 > r && g * 1.5 > b)
            //             {
            //                 motor1.write(1500);
            //                 motor2.write(1500);
            //                 break;
            //             }
            //         }
            //         if (col > 0)
            //         {
            //             motor1.write(1550);
            //             motor2.write(1450);
            //             if (b * 1.5 > r && b * 1.5 > g)
            //             {
            //                 motor1.write(1500);
            //                 motor2.write(1500);
            //                 break;
            //             }
            //         }
            //     }
                // Wait until color is read from the sensor

                // Read color from sensor
                // while(1){
                // while (!apds.colorAvailable()) {
                //     delay(5);
                // }
                // apds.readColor(r, g, b, a);
                // Serial.print("RED: ");
                // Serial.println(r);
                // Serial.print("GREEN: ");
                // Serial.println(g);
                // Serial.print("BLUE: ");
                // Serial.println(b);
                // Serial.print("AMBIENT: ");
                // Serial.println(a);
                // delay(500);
                // }
                
                // Serial.println("THRESHA:");
                // Serial.println(thresh);
                // // Print color in decimal
                
            // while (a < thresh)
            // {
            //     while (!apds.colorAvailable()) {
            //         delay(5);
            //     }
            //     apds.readColor(r, g, b, a);
            //     motor1.write(1550);
            //     motor2.write(1450);
            // }
            // Serial.println("DONE WITH COLOR");
            // while(1){
            //     BP32.update();
            //     while (!apds.colorAvailable()) {
            //         delay(5);
            //     }
            //     // Read color from sensor
            //     apds.readColor(r, g, b, a);
            //     motor1.write(1550);
            //     motor2.write(1450);
            //     int comp = r+g+b+a;
            //     int diff = sum-comp;
            //     Serial.println("diff:");
            //     Serial.println(diff);
            //     if (diff > -10 && diff < 10)
            //     {
            //         motor1.write(1500);
            //         motor2.write(1500);
            //         break;
            //     }
            //     if(controller->a()){
            //         Serial.println("B PRESSED: EXITING COLOR SUBROUTINE");
            //         digitalWrite(LED, HIGH);
            //         break;
            //     }            
                // Serial.print("RED: ");
                // Serial.println(r);
                // Serial.print("GREEN: ");
                // Serial.println(g);
                // Serial.print("BLUE: ");
                // Serial.println(b);
                // Serial.print("AMBIENT: ");
                // Serial.println(a);
           }
     }
    
    
    
    // vTaskDelay(1);
    

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

}

    
    
