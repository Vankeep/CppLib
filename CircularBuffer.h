//Version 0.1.2 Убрано дебильное переключение заднего хода
#include <CrsfSerial.h>
#include <Servo.h>
#include <math.h>
#include <median.h>

// #define SERIAL_LOG
#define CRSF_ON_USART2

#define VBAT_INTERVAL   500                 // Интервал чтения напряжения батареи (в миллисекундах)
#define VBAT_SMOOTH     5                   // Количество значений для медианного усреднения
#define VBAT_SCALE      1.008               // Коэффициент масштабирования значения напряжения батареи
#define VBAT_R1         2000                // Значение резистора R1 делителя напряжения 
#define VBAT_R2         122                 // Значение резистора R2 делителя напряжения
#define MIN_VOLT        413                 // Минимальное значение напряжения батареи
#define MAX_VOLT        544                 // Максимальное значение напряжения батареи
#define FORWARD_BACKWARD_CHANNEL      6     // Номер канала, контролирующего движение вперёд/назад.
#define TRAL_SWITCH_CHANNEL           7
#define ARM_SWITCH_CHANNEL            5     // Номер канала, отвечающего за "arm"/"diarm" системы.
#define LEFT_STICK_VERTICAL_CHANNEL   3     // Номер канала, связанного с вертикальным перемещением левого джойстика.
#define RIGHT_STICK_HORIZONTAL_CHANNEL  4   // Номер канала, связанного с горизонтальным перемещением правого джойстика.
#define PERSENT_OR_VOLTS_CHANNEL        8   // Номер канала, переключающий лобо на проценты либо на вольтаж батареи.
#define MIN_JOYSTICK_VALUE_FOR_ARMING   1050// Минимальное значение, которое должен достигнуть джойстик для активации arm.
#define MIN_TRIPLE_SWITCH               1200// Если ниже этого значения джойстик в минимальном положении
#define MAX_TRIPLE_SWITCH               1700// Если выше этого значения джойстик в максимальном положении

#define SMOOTHING_FACTOR                0.0001f // Константа для экспоненциального сглаживания

const uint8_t LEFT_MOTOR_PIN = PB9;          // Пин, к которому подключён левый мотор.
const uint8_t RIGHT_MOTOR_PIN = PB8;         // Пин, к которому подключён правый мотор.
const uint8_t LED_PIN = PA8;                 // Пин, к которому подключён светодиод (LED).
const uint8_t APIN_VBAT = PA0;               // Аналоговый пин, используемый для измерения напряжения аккумулятора.
const uint8_t TRAL_PIN = PB1;                // Пин для включения\выключения трала


bool isSystemArmed = false;
bool isQuickArmSwitch = false;
bool isFirstStart = true;

unsigned int leftMotorPwm = 1500;
unsigned int rightMotorPwm = 1500;

Servo leftMotorServo;
Servo rightMotorServo;

#ifdef SERIAL_LOG
uint32_t lastSerialPrint;
unsigned int values[2];
#endif

#ifdef CRSF_ON_USART2 
// К А3 паять TX, к A2 паять RX 
CrsfSerial crsf(Serial2, CRSF_BAUDRATE);
#else 
CrsfSerial crsf(Serial1, CRSF_BAUDRATE);
#endif



// Структура для хранения состояния подключения и сглаживания значений напряжения
static struct tagConnectionState {
     uint32_t lastVbatRead;                                  // Время последнего чтения напряжения
     MedianAvgFilter<unsigned int, VBAT_SMOOTH> vbatSmooth;  // Фильтр для сглаживания значений напряжения
     unsigned int vbatValue;                                 // Текущее значение напряжения батареи
} g_State;

static struct switchStateTracker {
     bool preArmValue;
     bool nowArmValue;
} switchState;

// Функция для установки скорости моторов в зависимости от значений джойстиков
void setMotorSpeed(unsigned int leftStickValue, unsigned int rigthStickValue, bool isForwardDrive) {
     unsigned int leftMotorSpeedDifference = 1500 - rigthStickValue;
     unsigned int rightMotorSpeedDifference = 1500 - (3000 - rigthStickValue);

     if (isForwardDrive) {
          leftStickValue = map(leftStickValue, 1000, 2000, 1500, 2000);
          leftStickValue = constrain(leftStickValue, 1500, 2000);
          leftMotorPwm = leftStickValue;
          rightMotorPwm = leftStickValue;
          smoothLeftMotorPwm();
          smoothRightMotorPwm();

          if (rigthStickValue < 1450)
               leftMotorPwm = constrain(leftMotorPwm - leftMotorSpeedDifference, 1500, 2000);

          if (rigthStickValue > 1550)
               rightMotorPwm = constrain(rightMotorPwm - rightMotorSpeedDifference, 1500, 2000);
     } else {
          leftStickValue = map(leftStickValue, 1000, 2000, 1500, 1000);
          leftStickValue = constrain(leftStickValue, 1000, 1500);
          leftMotorPwm = leftStickValue;
          rightMotorPwm = leftStickValue;
          smoothLeftMotorPwm();
          smoothRightMotorPwm();

          if (rigthStickValue < 1450)
               leftMotorPwm = constrain(leftMotorPwm + leftMotorSpeedDifference, 1000, 1500);

          if (rigthStickValue > 1550)
               rightMotorPwm = constrain(rightMotorPwm + rightMotorSpeedDifference, 1000, 1500);
     }
}

// Для остановки моторов
void stopMotors() {
      leftMotorPwm = 1500;
      rightMotorPwm = 1500;
      smoothLeftMotorPwm();
      smoothRightMotorPwm();
}

//   Перевод в проценты
void persentOrVolts(uint16_t& scaledVoltage) {
     if (crsf.getChannel(PERSENT_OR_VOLTS_CHANNEL) < MAX_TRIPLE_SWITCH) {
          int pr = (((scaledVoltage - MIN_VOLT) * 100) / (MAX_VOLT - MIN_VOLT))*10;
          if (pr >= 0)
               scaledVoltage = pr;
          else
               scaledVoltage = 0;
#ifdef SERIAL_LOG
          values[0] = 1;
          values[1] = scaledVoltage;
#endif
     }
#ifdef SERIAL_LOG
     else {

          values[0] = 0;
          values[1] = scaledVoltage;
     }
#endif
}

// Проверка и обновление напряжения батареи
static void checkVbatt() {
     if (millis() - g_State.lastVbatRead < (VBAT_INTERVAL / VBAT_SMOOTH))
          return;
     g_State.lastVbatRead = millis();

     unsigned int idx = g_State.vbatSmooth.add(analogRead(APIN_VBAT));
     if (idx != 0)
          return;

     unsigned int adc = g_State.vbatSmooth;
     g_State.vbatValue = 330U * adc * (VBAT_R1 + VBAT_R2) / VBAT_R2 / ((1 << 12) - 1);

     crsf_sensor_battery_t crsfbatt = { 0 };
     uint16_t scaledVoltage = g_State.vbatValue * VBAT_SCALE / 10;

     persentOrVolts(scaledVoltage);

     crsfbatt.voltage = htobe16(scaledVoltage);
     crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfbatt, sizeof(crsfbatt));
}

void smoothLeftMotorPwm(){
  static float finVal = 0;
  finVal += (leftMotorPwm - finVal) * SMOOTHING_FACTOR;
  leftMotorPwm = static_cast<unsigned int>(finVal);
  leftMotorPwm = leftMotorPwm < 1520 && leftMotorPwm > 1480 ? 1500 : leftMotorPwm;
}
void smoothRightMotorPwm(){
  static float finVal = 0;
  finVal += (rightMotorPwm - finVal) * SMOOTHING_FACTOR;
  rightMotorPwm = static_cast<unsigned int>(finVal);
  rightMotorPwm = rightMotorPwm < 1520 && rightMotorPwm > 1480 ? 1500 : rightMotorPwm;
}



void setup() {
     Serial.begin(115200);
     crsf.begin();

     pinMode(LED_PIN, OUTPUT);
     pinMode(TRAL_PIN, OUTPUT);

     leftMotorServo.attach(LEFT_MOTOR_PIN);
     rightMotorServo.attach(RIGHT_MOTOR_PIN);

     leftMotorServo.writeMicroseconds(1500);
     rightMotorServo.writeMicroseconds(1500);

     crsf.loop();
     isFirstStart = true;
}

void loop() {
     switchState.preArmValue = crsf.getChannel(ARM_SWITCH_CHANNEL) > 1500;

     crsf.loop();

     switchState.nowArmValue = crsf.getChannel(ARM_SWITCH_CHANNEL) > 1500;

     if (switchState.preArmValue != switchState.nowArmValue)
          isQuickArmSwitch = true;

     if (isQuickArmSwitch && crsf.getChannel(LEFT_STICK_VERTICAL_CHANNEL) < MIN_JOYSTICK_VALUE_FOR_ARMING)
          isQuickArmSwitch = false;

     isSystemArmed = crsf.getChannel(ARM_SWITCH_CHANNEL) > 1500 && crsf.isLinkUp() && isQuickArmSwitch == false;

     if (isFirstStart) {
          isSystemArmed = false;
          isFirstStart = crsf.getChannel(LEFT_STICK_VERTICAL_CHANNEL) < MIN_JOYSTICK_VALUE_FOR_ARMING;
     }
      
      if (isSystemArmed) {
        setMotorSpeed(crsf.getChannel(LEFT_STICK_VERTICAL_CHANNEL), crsf.getChannel(RIGHT_STICK_HORIZONTAL_CHANNEL), crsf.getChannel(FORWARD_BACKWARD_CHANNEL) < 1500);
      } else {
        stopMotors();
      }


    leftMotorServo.writeMicroseconds(leftMotorPwm);
    rightMotorServo.writeMicroseconds(rightMotorPwm);

#ifdef SERIAL_LOG
     if (millis() - lastSerialPrint >= 50) {
          lastSerialPrint = millis();
          Serial.print(values[0] ? "Persent: " : "Volts: ");
          Serial.print(values[1]);
          Serial.print(" L:");
          Serial.println(leftMotorPwm);
          Serial.print(" R:");
          Serial.println(rightMotorPwm);
     }
#endif
     
     digitalWrite(LED_PIN, isSystemArmed ? LOW : HIGH);
     checkVbatt();

    if(crsf.getChannel(ARM_SWITCH_CHANNEL) > 1500 && crsf.getChannel(TRAL_SWITCH_CHANNEL) > 1500) {
        digitalWrite(TRAL_PIN, HIGH);
    } else {
        digitalWrite(TRAL_PIN, LOW);
    }
}
