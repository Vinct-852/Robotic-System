#include "vesc_control.h"
#include "can_helper.h"
#include <cstdint>

vesc_control::vesc_control(CAN* can, int can_id, int can_baud_rate){
    // Setting the can bus message type
    this->can = can;
    this->can_id = can_id;
    can->frequency(can_baud_rate);
    // Rxmsg.format = CANExtended;
}

vesc_control::~vesc_control(){}


uint32_t vesc_control::can_send(int id, uint8_t packet[]) {
  CANMessage Txmsg = CANMessage();
  Txmsg.id = id;
  Txmsg.format = CANExtended;
  Txmsg.type = CANData;
  Txmsg.len = sizeof(packet);
  for (int i = 0; i < sizeof(packet); i++) {
    Txmsg.data[i] = packet[i];
  }
    //printf("can write: id:%x, len:%d, data:%x,%x,%x,%x,%x,%x,%x,%x \n", (Txmsg.id & 0xFF), sizeof(packet), packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7]);

  while(!can->write(Txmsg)); //to ensure the message is sent

  return 0;
}

uint32_t vesc_control::can_std_send(int id, uint8_t packet[]) {
  CANMessage Txmsg = CANMessage();
  Txmsg.id = id;
  Txmsg.format = CANStandard;
  Txmsg.type = CANData;
  Txmsg.len = sizeof(packet);
  for (int i = 0; i < sizeof(packet); i++) {
    Txmsg.data[i] = packet[i];
  }

  //printf("can write: id:%x, len:%d, data:%x,%x,%x,%x,%x,%x,%x,%x \n", Txmsg.id, sizeof(packet), packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7]);

  while(!can->write(Txmsg)); //to ensure the message is sent

  return 0;
}

void vesc_control::package_msg(uint8_t *buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void vesc_control::set_rpm(float rpm) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  package_msg(buffer, (int32_t)rpm, &send_index);
  printf("%d", this->can_id);
  printf("%x", this->can_id | ((uint32_t)CAN_PACKET_SET_RPM << 8));
  can_send(this->can_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer);
}

void vesc_control::set_pos(float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  package_msg(buffer, (int32_t)(pos * 1000000.0), &send_index);
  can_send(this->can_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer);
}
