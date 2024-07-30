#include <Arduino.h>

//#define RS485_DEBUG

#define WIND_SPEED  1
#define WIND_DIR    2
#define WIND_BOTH   3

#define WIND_TYPE   WIND_BOTH

#define RS485 Serial1
#define RS485_SPEED 9600
#define RS485_DELAY 50 // 50 ms

#define RS485_TX  5
#define RS485_RX  4
#define RS485_DE  3

#if WIND_TYPE == WIND_BOTH
#define ADDR_SPEED  1
#define ADDR_DIR    2
#endif

static uint16_t crc16(const uint8_t *data, uint8_t length, uint16_t crc = 0xFFFF) {
  while (length--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x01)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

static void rs485_write(const uint8_t *data, uint8_t length) {
  uint16_t crc;

  crc = crc16(data, length);
#ifdef RS485_DEBUG
  if (RS485.available()) {
    Serial.print("Input remains:");
    while (RS485.available()) {
      Serial.printf(" %02X", RS485.read());
    }
    Serial.println();
  }
#else
  while (RS485.available()) {
    RS485.read();
  }
#endif
  digitalWrite(RS485_DE, HIGH);
  RS485.write(data, length);
  RS485.write((uint8_t*)&crc, sizeof(crc));
  RS485.flush();
  digitalWrite(RS485_DE, LOW);
}

static bool rs485_read(uint8_t *data, uint8_t length) {
#ifdef RS485_DEBUG
  size_t len;
#endif
  uint16_t crc;

#ifdef RS485_DEBUG
  len = RS485.read(data, length);
  if (len) {
    Serial.printf("Read %u (expected %u) bytes:", len, length);
    for (uint8_t i = 0; i < len; ++i) {
      Serial.printf(" %02X", data[i]);
    }
    Serial.println();
    if (len == length) {
      if (RS485.read((uint8_t*)&crc, sizeof(crc)) == sizeof(crc)) {
        uint16_t _crc;

        _crc = crc16(data, length);
        Serial.printf("CRC %04X (expected %04X)\n", crc, _crc);
        if (_crc == crc)
          return true;
      } else {
        Serial.println("Error reading CRC!");
      }
    }
  }
  return false;
#else
  return (RS485.read(data, length) == length) && (RS485.read((uint8_t*)&crc, sizeof(crc)) == sizeof(crc)) &&
    (crc16(data, length) == crc);
#endif
}

static bool wind_getaddress(uint8_t *addr) {
  const uint8_t READ_DATA[] = { 0x00, 0x03, 0x00, 0x01, 0x00, 0x01 };

  uint8_t response[5];

  rs485_write(READ_DATA, sizeof(READ_DATA));
  delay(RS485_DELAY);
  if (rs485_read(response, sizeof(response))) {
    if ((response[0] == READ_DATA[0]) && (response[1] == READ_DATA[1]) &&
      (response[2] == 2) && (response[3] == 0)) {
      *addr = response[4];
      return true;
    }
  }
  return false;
}

static bool wind_setaddress(uint8_t addr) {
  uint8_t WRITE_DATA[] = { 0x00, 0x10, 0x01, 0x00 };
  uint8_t response[4];

  WRITE_DATA[3] = addr;
  rs485_write(WRITE_DATA, sizeof(WRITE_DATA));
  delay(RS485_DELAY);
  if (rs485_read(response, sizeof(response))) {
    if ((response[0] == WRITE_DATA[0]) && (response[1] == WRITE_DATA[1]) &&
      (response[2] == 0x01) && (response[3] == 0x01)) {
      return true;
    }
  }
  return false;
}

static bool wind_getdata(uint8_t addr, uint16_t *value) {
  uint8_t READ_DATA[] = { 0x00, 0x03, 0x00, 0x00, 0x00, 0x01 };
  uint8_t response[5];

  READ_DATA[0] = addr;
  rs485_write(READ_DATA, sizeof(READ_DATA));
  delay(RS485_DELAY);
  if (rs485_read(response, sizeof(response))) {
    if ((response[0] == READ_DATA[0]) && (response[1] == READ_DATA[1]) && (response[2] == 0x02)) {
      *value = (response[3] << 8) | response[4];
      return true;
    }
  }
  return false;
}

static bool wind_getbaudrate(uint8_t addr, uint16_t *value) {
  uint8_t READ_DATA[] = { 0x00, 0x03, 0x00, 0x10, 0x00, 0x01 };
  uint8_t response[5];

  READ_DATA[0] = addr;
  rs485_write(READ_DATA, sizeof(READ_DATA));
  delay(RS485_DELAY);
  if (rs485_read(response, sizeof(response))) {
    if ((response[0] == READ_DATA[0]) && (response[1] == READ_DATA[1]) && (response[2] == 0x02)) {
      *value = (response[3] << 8) | response[4];
      return true;
    }
  }
  return false;
}

static bool wind_setbaudrate(uint8_t addr, uint16_t value) {
  uint8_t WRITE_DATA[] = { 0x00, 0x10, 0x10, 0x00, 0x00 };
  uint8_t response[4];

  WRITE_DATA[0] = addr;
  WRITE_DATA[3] = value >> 8;
  WRITE_DATA[4] = value & 0xFF;
  rs485_write(WRITE_DATA, sizeof(WRITE_DATA));
  delay(RS485_DELAY);
  if (rs485_read(response, sizeof(response))) {
    if ((response[0] == WRITE_DATA[0]) && (response[1] == WRITE_DATA[1]) &&
      (response[2] == 0x01) && (response[3] == 0x01)) {
      return true;
    }
  }
  return false;
}

#if WIND_TYPE != WIND_BOTH
uint8_t addr = 0;
#endif

void setup() {
  Serial.begin(115200);

  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);
  RS485.begin(RS485_SPEED, SERIAL_8N1, RS485_RX, RS485_TX);
  RS485.setTimeout(500);
}

void loop() {
#if WIND_TYPE == WIND_BOTH
  uint16_t value;

  if (wind_getdata(ADDR_SPEED, &value)) {
    Serial.printf("Wind speed is %u.%u m/s\n", value / 10, value % 10);
  } else {
    Serial.println("Error getting wind speed!");
  }
  if (wind_getdata(ADDR_DIR, &value)) {
    Serial.printf("Wind direction is %u degrees\n", value);
  } else {
    Serial.println("Error getting wind direction!");
  }
#else // (WIND_TYPE == WIND_SPEED) || (WIND_TYPE == WIND_DIR)
  if (! addr) {
    if (wind_getaddress(&addr)) {
#if WIND_TYPE == WIND_SPEED
      Serial.printf("Wind speed device address is 0x%02X\n", addr);
    } else {
      Serial.println("Error getting wind speed device address!");
    }
#else // WIND_TYPE == WIND_DIR
      Serial.printf("Wind direction device address is 0x%02X\n", addr);
    } else {
      Serial.println("Error getting wind direction device address!");
    }
#endif
  } else {
    uint16_t value;

    if (wind_getdata(addr, &value)) {
#if WIND_TYPE == WIND_SPEED
      Serial.printf("Wind speed is %u.%u m/s\n", value / 10, value % 10);
    } else {
      Serial.println("Error getting wind speed!");
    }
#else // WIND_TYPE == WIND_DIR
      Serial.printf("Wind direction is %u degree\n", value);
    } else {
      Serial.println("Error getting wind direction!");
    }
#endif
  }
#endif

  delay(1000);
}
