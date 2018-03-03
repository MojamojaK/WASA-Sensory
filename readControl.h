#define CONTROL_SERIAL Serial3            // 操舵基板通信用シリアル
#define SENSORY_COMM_MAX_BYTES 64

uint32_t wait_time;
uint8_t servo_rx_packet[SENSORY_COMM_MAX_BYTES] = {0};

void initControl() {
#ifdef DEBUG
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
#ifdef DEBUG
  DEBUG_PORT.println("READ CONTROL");
#endif
  wait_time = millis();
  while (!CONTROL_SERIAL.available()) if (millis() - wait_time > 10UL) return;
  wait_time = millis();
  while ((servo_rx_packet[0] = CONTROL_SERIAL.read()) != 0x7C) if (millis() - wait_time > 25UL) return;
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
#ifdef DEBUG
  DEBUG_PORT.println("Received!");
#endif
}

void packControl(uint8_t *payload) {
  // ここでpayload[63~100]を占有
  payload[56] = servo_rx_packet[3 ]; // RUDDER flag
  payload[57] = servo_rx_packet[4 ]; // RUDDER position L
  payload[58] = servo_rx_packet[5 ]; // RUDDER position H
  payload[59] = servo_rx_packet[6 ]; // RUDDER goal time L
  payload[60] = servo_rx_packet[7 ]; // RUDDER goal time H
  payload[61] = servo_rx_packet[8 ]; // RUDDER max torque
  payload[62] = servo_rx_packet[9 ]; // RUDDER torque mode
  payload[63] = servo_rx_packet[10]; // RUDDER present position L
  payload[64] = servo_rx_packet[11]; // RUDDER present position H
  payload[65] = servo_rx_packet[12]; // RUDDER present time L
  payload[66] = servo_rx_packet[13]; // RUDDER present time H
  payload[67] = servo_rx_packet[14]; // RUDDER present speed L
  payload[68] = servo_rx_packet[15]; // RUDDER present speed H
  payload[69] = servo_rx_packet[16]; // RUDDER present load L
  payload[70] = servo_rx_packet[17]; // RUDDER present load H
  payload[71] = servo_rx_packet[18]; // RUDDER present temperature L
  payload[72] = servo_rx_packet[19]; // RUDDER present temperature H
  payload[73] = servo_rx_packet[20]; // RUDDER voltage L
  payload[74] = servo_rx_packet[21]; // RUDDER voltage H
  payload[75] = servo_rx_packet[22]; // ELEVATOR flag
  payload[76] = servo_rx_packet[23]; // ELEVATOR position L
  payload[77] = servo_rx_packet[24]; // ELEVATOR position H
  payload[78] = servo_rx_packet[25]; // ELEVATOR goal time L
  payload[79] = servo_rx_packet[26]; // ELEVATOR goal time H
  payload[80] = servo_rx_packet[27]; // ELEVATOR max torque
  payload[81] = servo_rx_packet[28]; // ELEVATOR torque mode
  payload[82] = servo_rx_packet[29]; // ELEVATOR present position L
  payload[83] = servo_rx_packet[30]; // ELEVATOR present position H
  payload[84] = servo_rx_packet[31]; // ELEVATOR present time L
  payload[85] = servo_rx_packet[32]; // ELEVATOR present time H
  payload[86] = servo_rx_packet[33]; // ELEVATOR present speed L
  payload[87] = servo_rx_packet[34]; // ELEVATOR present speed H
  payload[88] = servo_rx_packet[35]; // ELEVATOR present load L
  payload[89] = servo_rx_packet[36]; // ELEVATOR present load H
  payload[90] = servo_rx_packet[37]; // ELEVATOR present temperature L
  payload[91] = servo_rx_packet[38]; // ELEVATOR present temperature H
  payload[92] = servo_rx_packet[39]; // ELEVATOR voltage L
  payload[93] = servo_rx_packet[40]; // ELEVATOR voltage H
}
