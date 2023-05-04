#include <Encoder.h>

#define RIGHT_ENCODER_PIN1 11
#define RIGHT_ENCODER_PIN2 12
#define RIGHT_CURRENT_SENSOR_VO A2
#define RIGHT_MOTOR_ENABLE 21
#define RIGHT_MOTOR_DIRECTION 5
#define RIGHT_BRAKE_ENABLE 23
#define RIGHT_BRAKE_IN1 7
#define RIGHT_BRAKE_IN2 8

#define LEFT_ENCODER_PIN1 14
#define LEFT_ENCODER_PIN2 15
#define LEFT_CURRENT_SENSOR_VO A3
#define LEFT_MOTOR_ENABLE 20
#define LEFT_MOTOR_DIRECTION 6
#define LEFT_BRAKE_ENABLE 22
#define LEFT_BRAKE_IN1 9
#define LEFT_BRAKE_IN2 10

#define X_FORCE_SENSOR_CLK 1
#define X_FORCE_SENSOR_DAT 1

#define Y_FORCE_SENSOR_CLK 1
#define Y_FORCE_SENSOR_DAT 1

#define RIGHT_CURRENT_SENSOR_ID 1
#define LEFT_CURRENT_SENSOR_ID 2
#define RIGHT_ENCODER_SENSOR_ID 3
#define LEFT_ENCODER_SENSOR_ID 4
#define X_FORCE_SENSOR_ID 5
#define Y_FORCE_SENSOR_ID 6

#define RIGHT_MOTOR_ID 1
#define LEFT_MOTOR_ID 2
#define RIGHT_BRAKE_ID 3
#define LEFT_BRAKE_ID 4

Encoder rightEncoder(RIGHT_ENCODER_PIN1, RIGHT_ENCODER_PIN2);
Encoder leftEncoder(LEFT_ENCODER_PIN1, LEFT_ENCODER_PIN2);

bool started = false;

void setup()
{
  Serial.begin(115200);

  analogReadRes(13);

  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_DIRECTION, OUTPUT);
  analogWrite(RIGHT_MOTOR_ENABLE, 0);

  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(LEFT_MOTOR_DIRECTION, OUTPUT);
  analogWrite(LEFT_MOTOR_ENABLE, 0);

  pinMode(RIGHT_BRAKE_ENABLE, OUTPUT);
  pinMode(RIGHT_BRAKE_IN1, OUTPUT);
  pinMode(RIGHT_BRAKE_IN2, OUTPUT);
  analogWrite(RIGHT_BRAKE_ENABLE, 0);

  pinMode(LEFT_BRAKE_ENABLE, OUTPUT);
  pinMode(LEFT_BRAKE_IN1, OUTPUT);
  pinMode(LEFT_BRAKE_IN2, OUTPUT);
  analogWrite(LEFT_BRAKE_ENABLE, 0);
}

void writeSensorMessage(uint8_t sensorID, int sensorValue)
{
  uint16_t sensorValueMagnitude = abs(sensorValue);
  uint8_t sensorValueDirection = 0;
  if (sensorValue < 0)
  {
    sensorValueDirection = 1;
  }

  uint8_t message[8] = {
    0x76, 0x76, // Header
    0x02, // Message ID
    0x04, // Body length
    sensorID,
    (sensorValueMagnitude >> 8) & 0xFF,
    sensorValueMagnitude & 0xFF,
    sensorValueDirection
  };
  Serial.write(message, 8);
}

void readMessage()
{
  if (Serial.available() >= 2 && Serial.read() == 0x76 && Serial.read() == 0x76)
  {
    uint8_t messageMetadata[2];
    size_t nread = Serial.readBytes(messageMetadata, 2);
    if (nread != 2)
    {
      return;
    }

    uint8_t messageType = messageMetadata[0];
    uint8_t messageLength = min(4, messageMetadata[1]);

    uint8_t message[4];
    if (messageLength > 0)
    {
      nread = Serial.readBytes(message, messageLength);
      if (nread != messageLength)
      {
        return;
      }
    }
    if (messageType == 0x00) // START message
    {
      started = true;
    }

    if (!started)
    {
      return;
    }

    if (messageType == 0x01) // POWER message
    {
      int motorID = message[0];
      int motorPower = message[1];
      int motorDirection = message[2];

      if (motorID == RIGHT_MOTOR_ID)
      {
        analogWrite(RIGHT_MOTOR_ENABLE, motorPower);
        digitalWrite(RIGHT_MOTOR_DIRECTION, motorDirection);
      }
      else if (motorID == LEFT_MOTOR_ID)
      {
        analogWrite(LEFT_MOTOR_ENABLE, motorPower);
        digitalWrite(LEFT_MOTOR_DIRECTION, motorDirection);
      }
      else if (motorID == RIGHT_BRAKE_ID)
      {
        analogWrite(RIGHT_BRAKE_ENABLE, motorPower);
        digitalWrite(RIGHT_BRAKE_IN1, 0);
        digitalWrite(RIGHT_BRAKE_IN2, 1);
      }
      else if (motorID == LEFT_BRAKE_ID)
      {
        analogWrite(LEFT_BRAKE_ENABLE, motorPower);
        digitalWrite(LEFT_BRAKE_IN1, 0);
        digitalWrite(LEFT_BRAKE_IN2, 1);
      }
    }
  }
}

void loop()
{
  readMessage();

  if (!started)
  {
    return;
  }

  writeSensorMessage(RIGHT_CURRENT_SENSOR_ID, analogRead(RIGHT_CURRENT_SENSOR_VO));
  writeSensorMessage(LEFT_CURRENT_SENSOR_ID, analogRead(LEFT_CURRENT_SENSOR_VO));
  writeSensorMessage(RIGHT_ENCODER_SENSOR_ID, rightEncoder.read());
  writeSensorMessage(LEFT_ENCODER_SENSOR_ID, leftEncoder.read());
}
