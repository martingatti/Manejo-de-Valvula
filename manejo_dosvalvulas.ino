#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Dirección I2C

const uint8_t led1Channel = 0;  // Canal del LED 1
const uint8_t led2Channel = 1;  // Canal del LED 2
const uint8_t pinLed = 2;       // LED indicador de error en la ESP32

const uint16_t minPWM = 50;    // Valor mínimo permitido
const uint16_t maxPWM = 4095;  // Valor máximo permitido
const uint8_t steps = 50;      // Número de pasos

void errorHandler(const char* mensaje) {
    Serial.println(mensaje);
    while (true) {
        digitalWrite(pinLed, HIGH);
        delay(500);
        digitalWrite(pinLed, LOW);
        delay(500);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(pinLed, OUTPUT);

    Serial.println("Iniciando I2C...");
    Wire.begin(21, 22); // Pines SDA y SCL en ESP32
    delay(100);

    Serial.println("Configurando PWM...");
    pwm.begin();
    pwm.setPWMFreq(200); // Configurar la frecuencia del PWM
    Serial.println("PCA9685 inicializado correctamente.");

    digitalWrite(pinLed, HIGH);
    delay(1000);
    digitalWrite(pinLed, LOW);
}

void aumentarBrillo(uint8_t ledChannel) {
    uint16_t stepSize = (maxPWM - minPWM) / steps;
    if (stepSize < 1) stepSize = 1;  // Evitar pasos demasiado pequeños

    Serial.print("Aumentando brillo LED ");
    Serial.println(ledChannel);

    for (uint16_t i = minPWM; i <= maxPWM; i += stepSize) {
        Serial.println(i); // Depuración para verificar los valores de i
        pwm.setPWM(ledChannel, 0, i);
        delay(30);
    }
}

void disminuirBrillo(uint8_t ledChannel) {
    uint16_t stepSize = (maxPWM - minPWM) / steps;
    if (stepSize < 1) stepSize = 1;  // Evitar pasos demasiado pequeños

    Serial.print("Disminuyendo brillo LED ");
    Serial.println(ledChannel);

    for (int16_t i = maxPWM; i >= minPWM; i -= stepSize) {
        Serial.println(i); // Depuración para verificar los valores de i
        pwm.setPWM(ledChannel, 0, i);
        delay(45);
    }
}

void loop() {
    // Aumentar y disminuir brillo del LED 1
    aumentarBrillo(led1Channel);
    disminuirBrillo(led1Channel);
    
    // Pequeña pausa entre LEDs
    delay(500);

    // Aumentar y disminuir brillo del LED 2
    aumentarBrillo(led2Channel);
    disminuirBrillo(led2Channel);

    Serial.println("Ciclo completo.");
    delay(1000);
}