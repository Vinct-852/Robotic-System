#include <ThisThread.h>
#include <mbed.h>

#include "Thread.h"
#include "vesc_control.h"
#include "vesc.h"
#include "kinematics.h"
//#include "cchs_can_bus.h"

#include <cstdint>

/**
 * @file       cchs_can_bus.h
 * @author     Ki C
 * @email      polyu.robocon@gmail.com
 * @brief      This stm program is a part of robot Architecture, named CCHS
 * @version    1.0.0Thread
 * @date       27-12-2023
 * @copyright  Copyright (c) 2023
 *
 * @section    LICENSE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *Thread
 * //————————————Swerve Drive Chassis Steering and driving modules CAN ID configurations————————————//
 *
 *
 *       /----- --Front-------\           ---           Modulus Config
 *       | [5]            [6] |            |
 *       |   {1}        {2}   |            |             [1]------[2]
 *       |                    |            |              |        |
 *    Left                    Right      Length           |        |
 *       |                    |            |              |        |
 *       |   {3}        {4}   |            |             [3]------[4]
 *       | [7]            [8] |            |
 *       \--------Aft---------/           ---
 *
 *
 *      |---------Width--------|
 *
 * [Driving module];
 * {Steering Module}
 */


const int canBaudRate = 1000000;   // 1000000
CAN can1(PD_0, PD_1, canBaudRate); // Can bus for transceiving controller commands
CAN can2(PB_5, PB_6, canBaudRate); // Can bus for transceiving chassis vesc commands
CAN can3(PB_3, PB_4, canBaudRate); // Can bus for transceiving sensor data

InterruptIn button(PC_13);

//-------------------------------CAN 1-------------------------------//
typedef enum
{
      CAN_PACKET_CHASSIS_COMMAND_SPEED_X__Y = 0,
      CAN_PACKET_CHASSIS_COMMAND_SPEED_W,
      CAN_PACKET_CCHS_ALIVE_STATE,

      CAN_MOTOR_CONTROL_TESTING,

} CAN_PACKET_CHASSIS_ID;

//-------------------------------CAN 2-------------------------------//
int canDrivingId[4] = {5, 6, 7, 8}; // ref above graph
//int canSteeringId[4] = {1, 2, 3, 4};
int canSteeringId[4] = {1, 2, 3, 4};
const int canAliveStateId = 5;
CANMessage rx1Msg;
CANMessage rx2Msg;


static BufferedSerial pc(USBTX, USBRX);

Vesc _vesc;

Thread canThread;
Thread can2Thread;
Thread aliveStateThread;
Thread publishVescThread;

// Kinematics Settings
swerve_drive swerve(0.542f, 0.550f, 0.073f, 7); // figure pls ref INVERSE_KINEMATICS_H
float joyX = 0, joyY = 0, joyW = 0;
float getDrivingRpm[4], getSteeringAngle[4];
float sendDrivingRpm[4], sendSteeringAngle[4];

// Motor control testting
uint8_t data = 0;

// Frequency Setting
int chassis2vesc_CAN_bus_frequency = 50; // Hz
int CCHS2nuc_alive_CAN_bus_frequency = 2; // Hz

void vesc_read()
{
    for (int i = 0; i < 4; i++)
    {
        _vesc.can_read(canDrivingId[i]);
        _vesc.can_read(canSteeringId[i]);
        // getDrivingRpm[i] = _vesc.read_rpm(canDrivingId[i]);
        // printf("%f\r\n", getDrivingRpm[i]);
        // getSteeringAngle[i] = _vesc.read_pos(canSteeringId[i]);
        // printf("%f\r\n", getSteeringAngle[i]);
    }
}

void printMsg(CANMessage &msg)
{
    // printf("  ID      = 0x%.3x\r\n", msg.id);
    // printf("  Type    = %d\r\n", msg.type);
    // printf("  Format  = %d\r\n", msg.format);
    // printf("  Length  = %d\r\n", msg.len);
    // printf("  Data    =");
    // for(int i = 0; i < msg.len; i++)
    //     printf(" 0x%.2X", msg.data[i]);
    // printf("\r\n");
    printf("  ID      = 0x%.3x   ", msg.id);
    printf("%02X", msg.data[0]);
    printf("\r\n");
}

float uint8ArrayToFloat32(const uint8_t *array)
{
    uint32_t floatAsUint32 = array[0] | (array[1] << 8) | (array[2] << 16) | (array[3] << 24); // Combine the individual bytes into a uint32_t value
    float *floatPointer = reinterpret_cast<float *>(&floatAsUint32);                           // Interpret the uint32_t value as a float pointer
    return *floatPointer;
}

void turn_around(int direction) {
    printf("Auto Adjustment\n");
    if (direction == 1){
        _vesc.set_rpm(69,1000);
        // the rpm 1000 is around the minimum value we can set
        ThisThread::sleep_for(50ms);

   
    }
    else{
        _vesc.set_rpm(69,-1000);
        ThisThread::sleep_for(50ms);


    }

    
}

void can1MessageHandler(CANMessage msg)
{
    int msgId = msg.id;
    switch (msgId)
    {
    case CAN_PACKET_CHASSIS_COMMAND_SPEED_X__Y:
    {
        uint8_t arrX[4] = {msg.data[0], msg.data[1], msg.data[2], msg.data[3]};
        joyX = uint8ArrayToFloat32(arrX);
        //printf("%f\r\n", joyX);

        uint8_t arrY[4] = {msg.data[4], msg.data[5], msg.data[6], msg.data[7]};
        joyY = uint8ArrayToFloat32(arrY);
        // printf("%f\r\n", joyY);
        break;
    }
    case CAN_PACKET_CHASSIS_COMMAND_SPEED_W:
    {
        uint8_t arrW[4] = {msg.data[0], msg.data[1], msg.data[2], msg.data[3]};
        joyW = uint8ArrayToFloat32(arrW);
        //printf("%f\r\n", joyW);

        break;
    }
    case CAN_MOTOR_CONTROL_TESTING:
    {
        data = {msg.data[0]};
        //printf("received data[0]: %f\r\n", data);

        break;
    }
    case 87:
    {
        int dir = msg.data[0];
        printf("direction: %d\r\n", dir);
        turn_around(dir);

        break;
    }
    default: // stop the motors on error
    {
        joyX = 0;
        joyY = 0;
        joyW = 0;
        //printf("ERROR IN READING CAN MESSAGE ID");
        break;
    }
    }
}
void can2MessageHandler(CANMessage msg){

    printf("CAN Message:\n");
    printf("ID: %u\n", msg.id);
    printf("Can_ID: %u\n", msg.id & 0xFF);
    printf("Data: ");
    for (int i = 0; i < msg.len; ++i) {
        printf("%02X ", msg.data[i]);
    }
    printf("\n");
    printf("Length: %u\n", msg.len);
    //printf("Format: %s\n", msg.format == STANDARD_FORMAT ? "Standard" : "Extended");
    //printf("Type: %s\n", msg.type == DATA_FRAME ? "Data Frame" : "Remote Frame");
    //int msgId = msg.id;
    // switch (msgId)
    // {
    //     case CAN_MOTOR_CONTROL_TESTING:
    //     {
    //         data = {msg.data[0]};
    //         printf("received data[0]: %f\r\n", data);
    //     }
    // }
}
void can1Listener()
{
    while (true)
    {
        while (can1.read(rx1Msg) == 1)
        {
            if (can1.rderror())
            {
                printf("\ncan error\n");
                can1.reset();
            }
            else
            {
                can1MessageHandler(rx1Msg);
            }
        }
        // printf("can ping\n");
    }
}
void can2Listener()
{
    while (true)
    {
        while (can2.read(rx2Msg) == 1)
        {
            if (can2.rderror())
            {
                printf("\ncan error\n");
                can2.reset();
            }
            else {
                //can2MessageHandler(rx2Msg);
            }
        }
    }
}
void stmAliveStateSender() // Send alive state to the CAN Bus
{
    while (true)
    {
        can1.write(CANMessage(canAliveStateId, "1", 1)); // msgID, msg, msgSize(8bits)
        ThisThread::sleep_for(500ms);                    // 2Hz
    }
}

bool flag = false;

void vescPublisher()
{
    while (true)
    {
        swerve.ik(joyX, joyY, joyW, getDrivingRpm, getSteeringAngle, sendDrivingRpm, sendSteeringAngle);
        //printf("joyX: %f, joyY: %f, joyW: %f, getDrivingRpm: %f, getSteeringAngle: %f, sendDrivingRpm: %f, sendSteeringAngle: %f\n", joyX, joyY, joyW, getDrivingRpm[0], getSteeringAngle[0], sendDrivingRpm[0], sendSteeringAngle[0]);

        for (int i = 0; i < 4; i++)
        {
            // if(joyX ==0 && joyY == 0)
            // {
                _vesc.set_rpm(canDrivingId[i], sendDrivingRpm[i]);
                // printf("%f RPM: %f\n", (float)canDrivingId[i], sendDrivingRpm[i]);     
            // }
            // else
            // {
            //     _vesc.set_current_brake(canDrivingId[i], 1);
            // }

        }
        vesc_read();
        for (int i = 0; i < 4; i++)
        {
            _vesc.set_pos(canSteeringId[i], sendSteeringAngle[i]* (180.0 / PI));
            // printf("%f Steering angle: %f\n", (float)canSteeringId[i] , sendSteeringAngle[i] * (180 / PI));     
        }
        // ThisThread::sleep_for(30ms); // unit in milliseconds

        if (data == 1)
        {
            data = 0;
            if (flag)
            {
                flag = false;
                printf("VESC off!");
                _vesc.set_rpm(69, 0);
            }
            else
            {
                flag = true;
                printf("VESC on!");
                _vesc.set_rpm(69, 5000);
                ThisThread::sleep_for(3000ms);
            }
        }

    }
}

vesc_control jumping_vesc = vesc_control(&can2, 69, canBaudRate);
Thread jumping_vesc_thread;

void jumping_vesc_vontrol() {
    while(true) {
        printf("running!");
        jumping_vesc.set_rpm(5000);
    }
}



// testing
//  void rpmHandler() {
//      while(true) {
//          if(can1.read(msg)) {
//              if(msg.id == VESC_CAN_ID) {
//                  int32_t erpm = (msg.data[0] << 24) | (msg.data[1] << 16) | (msg.data[2] << 8) | msg.data[3];
//                  float rpm = erpm / 7.0;
//                  printf("VESC RPM: %f\n", rpm);
//              }
//          }
//          wait_ms(100);
//      }
//  }

uint8_t button_detect = 0;

void button_handler() {
    button_detect = 1;
}



int main()
{
    button.rise(&button_handler);
    
    _vesc.vesc_init(&can2, canBaudRate); // VESC Initialization
    
    for (int i = 0; i < 4; i++)
    {
        _vesc.set_monitor_id(canDrivingId[i]);
        _vesc.set_monitor_id(canSteeringId[i]);
    }
    _vesc.set_monitor_id(69);
    // vesc_read();

    // Main Setup Program
    char msg[] = "Hello world";
    // debugMsg(msg);
    printf("%s\n", msg);

    // Threads
    canThread.start(can1Listener);               // listen to can1
    // can2Thread.start(can2Listener); 
    aliveStateThread.start(stmAliveStateSender); // send alive state
    publishVescThread.start(vescPublisher);      //
    
    //jumping_vesc_thread.start(jumping_vesc_vontrol);

    //printf("can write\n");
    //ThisThread::sleep_for(100ms); // small delay for updating
    //ThisThread::sleep_for(100ms);
    //_vesc.set_duty(1, 0.4);
    //ThisThread::sleep_for(100ms);
    //_vesc.set_current(1, 2);
    //ThisThread::sleep_for(100ms);
    //_vesc.set_current_brake(1, 1);
    //ThisThread::sleep_for(100ms);
    //_vesc.set_rpm(1, 1000);
    //ThisThread::sleep_for(100ms);

    // printf("can write\n");
    // printf("hihihi %d\n",123);
    // pc.write(msg, sizeof(msg));
    // can1.write(CANMessage(1, "hihi", 2)); //msgID, msg, msgSize(8bits)
    // can3.write(CANMessage(1, "hihi", 2, CANRemote));

    // float rpm = 9.9;
    /*
    ThisThread::sleep_for(1000ms);
    printf("write1\n");
    ThisThread::sleep_for(1000ms);
    printf("write2\n");
    ThisThread::sleep_for(1000ms);
    printf("write3\n");0 00 00 0
    ThisThread::sleep_for(1000ms);
    printf("write4\n");q wqw     
    /*
    _vesc.set_pos_origin(1, 0);
    ThisThread::sleep_for(100ms);
    */
    /*
    for (int i = 0; i < 300; i++) {
        _vesc.set_pos_spd(1, 0, 15000, 1500);
        ThisThread::sleep_for(30ms);
    }
    */
    
    /*
    printf("can write1\n");
    for (int i = 0; i < 100; i++) {
        _vesc.set_pos_spd(1, 90, 1000, 1000);
        ThisThread::sleep_for(30ms);
    }
    printf("can write2\n");
    for (int i = 0; i < 100; i++) {
        //_vesc.set_pos(1, 180);
        _vesc.set_pos_spd(1, 180, 1000, 1000); n
    printf("can write3\n"); 
        _vesc.set_pos_spd(1, 270, 1000, 1000);
        ThisThread::sleep_for(30ms);
    }
    printf("can write2\n");
    // */ 
    // while (true)
    // {
        // _vesc.set_rpm(6, 2000);
        // _vesc.set_pos(canSteeringId[0], 90);
        // vesc_read();
        //_-vesc.can_read(8);
        // rpm = _vesc.read_rpm(8);
        // printf("%f\n",rpm);
        //ThisThread::sleep_for(1ms); // small delay for updating
        //_vesc.set_rpm(1, 8000);
        //_vesc.set_pos(1, degree);
        //_vesc.set_pos_spd(1, degree, 15000, 15000);

        //_vesc.set_pos(1, degree);
        
        /*
        //if (counter++ >= 3)
        {
            counter = 0;
            degree += degree_delta;
            if (degree == 30)
                degree_delta = -3;
            else if (degree == 0)
                degree_delta = 3;
        }
        */
        //_vesc.set_pos(1, 0);
        //_vesc.set_pos_spd(1, 18000, 30000, 6000);
        // ThisThread::sleep_for(100ms); // small delay for updating

        // if (button_detect) {
        //     printf("button press\n");
        //     _vesc.set_solenoid(40, ~can_data);
        //     can_data <<= 1;
        //     if (can_data == 0)
        //         can_data = 1;
        //     button_detect = 0;
        // }

    // }
}

/*
STEPS:
1. try controlling vesc √(1-NOV-2023): fixed can bus buffer overflow issue
2. try communication with computer √(3-NOV-2023): can transceive CAN messages
3. receive joy data √(6-NOV-2023): can receive data from NUC -> next: process data
4. implement kinematics √(7-NOV-2023): waiting data from NUC to test
5. test kinematics √(13-NOV-2023): require fixing in kinematics
6. test vesc library (read pos & rpm)
7. try controlling the robot
*/

/*
TODO:
- point all wheels to same direction (red line) (to ensure all starting angle are same)
*/
