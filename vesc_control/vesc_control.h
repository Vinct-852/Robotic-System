#include "mbed.h"
#include "vesc_id.h"

/**
 * @file       vesc_new.h
 * @author     Vincent dcy
 * @email      polyu.robocon@gmail.com
 * @brief      
 * @version    0.0.0
 * @date       
 * @copyright  Copyright (c) 2024
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
 *
 * @section    DESCRIPTION
 */

#define CAN_BAUD_RATE 1000000

class vesc_control {
    public:

        /*
        * Description:  init the vesc, setting up the can connection
        *
        * Intput: CAN* can: can object, int can_baud_rate:https://www.google.com/search?q=can_baud_rate&sca_esv=fe1d248b964710f2&sxsrf=ADLYWIJxJ-KT_Omkcby8_x_eJggpDRIXpA%3A1728043187569&ei=s9j_ZqSyIqSk2roPnInz4As&ved=0ahUKEwjk1-fC1vSIAxUkklYBHZzEHLwQ4dUDCA8&uact=5&oq=can_baud_rate&gs_lp=Egxnd3Mtd2l6LXNlcnAiDWNhbl9iYXVkX3JhdGUyBxAAGIAEGA0yBxAAGIAEGA0yBxAAGIAEGA0yBxAAGIAEGA0yBxAAGIAEGA0yBxAAGIAEGA0yBxAAGIAEGA0yBxAAGIAEGA0yBBAAGB4yBBAAGB5I4wFQAFgAcAB4AZABAJgBfqABfqoBAzAuMbgBA8gBAPgBAvgBAZgCAaAChgGYAwDiAwUSATEgQJIHAzAuMaAHpwY&sclient=gws-wiz-serp
        * 
        * Output: null
        */
        vesc_control(CAN* can, int can_id, int can_baud_rate = CAN_BAUD_RATE);
        ~vesc_control();

        /*
        * Description:
            // velocity control cmd 
            // set up the velocity pid control set point in rpm/erpm
            // same as vesc tools 
            // for Sersorless motor the rpm means the erpm of the motor
            // for Hall Sensor/ Encoder motor this means the rpm read form the encoder
            // VESC6 (HW) erpm limit +-150000 
            // VESC4 (HW) erpm limit +-60000
        *
        * Intput: float rpm: velocity
        * 
        * Output: null
        */
        void set_rpm(float rpm);                        //set the velocity of the motor in rpm
        
        /*
        * Description:
            // position control cmd 
            // set up the position pid control set point in deg
            // same as vesc tools 
            // for Sersorless motor not recommand to use this function
            // for Hall Sensor/ Encoder motor this means the position read form the encoder
        *
        * Intput: float pos: position
        * 
        * Output: null
        */
        void set_pos(float pos);                        //set the position of the motor in deg

        //void set_duty(int id,float duty);                       //set the duty cycle set point for the motor in %
        //void set_current(int id, float current);                //set the current set point for the motor in A
        //void set_current_brake(int id, float current);          //set the braking current of the motor in A

    protected:   

        int can_id;
        
        // Class can msg define
        CAN* can;
        // CANMessage Rxmsg;

        /*
        * Description: Send the msg to the canbus from the buffer packet
        *
        * Intput: int id: ID for the CAN message, uint8_t packet[]: message
        * 
        * Output: success = 0 *ALWAYS RETURN 0, ERROR CANNOT BE DETECTED*
        */
        uint32_t can_send(int can_msg_id,uint8_t packet[]);
        uint32_t can_std_send(int can_msg_id,uint8_t packet[]);

        /*
        * Description: Format the data in to canbus msg
        *
        * Intput: int32_t number: message data
        * 
        * Output: uint8_t* buffer: converted message data, int32_t *index : length of the message
        */
        void package_msg(uint8_t* buffer, int32_t number, int32_t *index);
        //bool can_std_send(int id, uint8_t packet[], int32_t len);
};
