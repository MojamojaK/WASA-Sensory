#define CONTROL_SERIAL Serial3            // 操舵基板通信用シリアル
#define SENSORY_COMM_MAX_BYTES 64

#ifdef DEBUG_CONTROL
uint32_t loop_time_control = 0;
#endif

uint32_t wait_time;
uint8_t servo_rx_packet[SENSORY_COMM_MAX_BYTES] = {0};

void initControl() {
#ifdef DEBUG_CONTROL
  DEBUG_PORT.println("INIT CONTROL");
#endif
  CONTROL_SERIAL.begin(9600);
}

uint8_t i = 1;
uint8_t len = 0;
byte sum = 0;

uint8_t checksum(uint8_t * data, uint8_t len) {
  sum = 0;
  for (i = 2; i < len; i++) sum ^= data[i];
  return sum;
}

void readControl() {
#ifdef DEBUG_CONTROL
  loop_time_control = millis();
  DEBUG_PORT.println("READ CONTROL");
#endif
  wait_time = millis();
  while (!CONTROL_SERIAL.available()) if ((uint32_t)(millis() - wait_time) > 10UL) return;
  wait_time = millis();
  while ((servo_rx_packet[0] = CONTROL_SERIAL.read()) != 0x7C) if ((uint32_t)(millis() - wait_time) > 25UL) return;
  wait_time = millis();
  for (i = 1; i < 3; i++) {
    while (!CONTROL_SERIAL.available()) if ((uint32_t)(millis() - wait_time) > 3UL) return;
    servo_rx_packet[i] = CONTROL_SERIAL.read();
    wait_time = millis();
  }
  len = servo_rx_packet[2] + 4;
  for (i = 3; i < len; i++) {
    while (!CONTROL_SERIAL.available()) if ((uint32_t)(millis() - wait_time) > 4UL) return;
    servo_rx_packet[i] = CONTROL_SERIAL.read();
    wait_time = millis();
  }
  if (!(servo_rx_packet[0] == 0x7C && servo_rx_packet[1] == 0xC7 && servo_rx_packet[len - 1] == checksum(servo_rx_packet, len - 1))) return;
#ifdef DEBUG_CONTROL
  DEBUG_PORT.println("Received Control Info!");
#endif
#ifdef DEBUG_CONTROL
  DEBUG_PORT.print("Control Loop Time: ");
  DEBUG_PORT.print(millis() - loop_time_control);
  DEBUG_PORT.print("\n");
  DEBUG_PORT.flush();
#endif
}

void packControl(uint8_t *payload) {
  // ここでpayload[53~90]を占有
  payload[53] = servo_rx_packet[3 ]; // RUDDER flag
  
// 目標位置,時間,最大トルクは使いません
//  payload[] = servo_rx_packet[4 ]; // RUDDER position L
//  payload[] = servo_rx_packet[5 ]; // RUDDER position H
//  payload[] = servo_rx_packet[6 ]; // RUDDER goal time L
//  payload[] = servo_rx_packet[7 ]; // RUDDER goal time H
//  payload[] = servo_rx_packet[8 ]; // RUDDER max torque

  payload[54] = servo_rx_packet[9 ]; // RUDDER torque mode
  payload[55] = servo_rx_packet[10]; // RUDDER present position L
  payload[56] = servo_rx_packet[11]; // RUDDER present position H

// 実時間と速度は使いません
//  payload[] = servo_rx_packet[12]; // RUDDER present time L
//  payload[] = servo_rx_packet[13]; // RUDDER present time H
//  payload[] = servo_rx_packet[14]; // RUDDER present speed L
//  payload[] = servo_rx_packet[15]; // RUDDER present speed H
  
  payload[57] = servo_rx_packet[16]; // RUDDER present load L
  payload[58] = servo_rx_packet[17]; // RUDDER present load H
  payload[59] = servo_rx_packet[18]; // RUDDER present temperature L
  payload[60] = servo_rx_packet[19]; // RUDDER present temperature H
  payload[61] = servo_rx_packet[20]; // RUDDER voltage L
  payload[62] = servo_rx_packet[21]; // RUDDER voltage H
  
  payload[63] = servo_rx_packet[22]; // ELEVATOR flag
  
// 目標位置,時間,最大トルクは使いません
//  payload[] = servo_rx_packet[23]; // ELEVATOR position L
//  payload[] = servo_rx_packet[24]; // ELEVATOR position H
//  payload[] = servo_rx_packet[25]; // ELEVATOR goal time L
//  payload[] = servo_rx_packet[26]; // ELEVATOR goal time H
//  payload[] = servo_rx_packet[27]; // ELEVATOR max torque
  
  payload[64] = servo_rx_packet[28]; // ELEVATOR torque mode
  payload[65] = servo_rx_packet[29]; // ELEVATOR present position L
  payload[66] = servo_rx_packet[30]; // ELEVATOR present position H

// 実時間と速度は使いません
//  payload[] = servo_rx_packet[31]; // ELEVATOR present time L
//  payload[] = servo_rx_packet[32]; // ELEVATOR present time H
//  payload[] = servo_rx_packet[33]; // ELEVATOR present speed L
//  payload[] = servo_rx_packet[34]; // ELEVATOR present speed H

  payload[67] = servo_rx_packet[35]; // ELEVATOR present load L
  payload[68] = servo_rx_packet[36]; // ELEVATOR present load H
  payload[69] = servo_rx_packet[37]; // ELEVATOR present temperature L
  payload[70] = servo_rx_packet[38]; // ELEVATOR present temperature H
  payload[71] = servo_rx_packet[39]; // ELEVATOR voltage L
  payload[72] = servo_rx_packet[40]; // ELEVATOR voltage H
}

