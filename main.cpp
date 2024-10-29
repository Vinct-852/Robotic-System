#include <ThisThread.h>
#include <mbed.h>

#include "Thread.h"
#include "vesc_control.h"
#include "vesc.h"
#include "kinematics.h"
//#include "cchs_can_bus.h"

#include <cstdint>

CAN can1(PD_0, PD_1, canBaudRate); // Can bus linux to STM 
CAN can2(PB_5, PB_6, canBaudRate); // Can bus STM to Linux 
CAN can3(PB_3, PB_4, canBaudRate); // Can bus motors to STM

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
    }
}

void printMsg(CANMessage &msg)
{
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


int main()
{    
    _vesc.vesc_init(&can2, canBaudRate); // VESC Initialization
    
    for (int i = 0; i < 4; i++)
    {
        _vesc.set_monitor_id(canDrivingId[i]);
        _vesc.set_monitor_id(canSteeringId[i]);
    }
    _vesc.set_monitor_id(69);

    // Main Setup Program
    char msg[] = "Hello world";
    printf("%s\n", msg);

    // Threads
    canThread.start(can1Listener);               // listen to can1
    aliveStateThread.start(stmAliveStateSender); // send alive state
    publishVescThread.start(vescPublisher);      
}

