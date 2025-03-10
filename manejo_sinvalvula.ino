#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Dirección I2C

const uint8_t ledChannel = 0;  // Canal donde conectaste el LED
const uint8_t pinLed = 2;  
const uint16_t minPWM = 50;    // Valor mínimo permitido
const uint16_t maxPWM = 4095;  // Valor máximo permitido
const uint8_t steps = 50;      // Número de pasos

void errorHandler(const char* mensaje) {
    Serial.println(mensaje);
    // Indicar error con parpadeo del LED
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
    if (!pwm.begin()) {
        errorHandler("Error: No se pudo iniciar PCA9685.");
    }

    pwm.setPWMFreq(200); // Frecuencia de PWM
    Serial.println("PCA9685 inicializado correctamente.");

    digitalWrite(pinLed, HIGH);
    delay(1000);
    digitalWrite(pinLed, LOW);
}

void loop() {
    Serial.println("Aumentando brillo...");

    uint16_t stepSize = (maxPWM - minPWM) / steps;

    for (uint16_t i = minPWM; i <= maxPWM; i += stepSize) {
        if (pwm.setPWM(ledChannel, 0, i) != 0) {  // Verifica error en setPWM()
            errorHandler("Error: No se pudo configurar el PWM (aumento).");
        }
        delay(30);
    }

    Serial.println("Disminuyendo brillo...");

    for (int16_t i = maxPWM; i >= minPWM; i -= stepSize) {  
        if (pwm.setPWM(ledChannel, 0, i) != 0) {  // Verifica error en setPWM()
            errorHandler("Error: No se pudo configurar el PWM (disminución).");
        }
        delay(45);
    }

    Serial.println("Ciclo completo.");
    digitalWrite(pinLed, LOW);
    delay(500);
}

/*
Código viejo!!  Semana del 7/03

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Dirección I2C

const uint8_t ledChannel = 0; // Canal donde conectaste el LED
const uint8_t pinLed = 2;  
void setup() {
    Serial.begin(115200);
    Wire.begin(21,22); // Pines SDA (21) y SCL (22) para ESP32
    pinMode(pinLed,OUTPUT);
    Serial.println("Configurando PWM");
    delay(10);
    if (!pwm.begin()){
      Serial.println("Error:No se inicio en pca");

      while(1){
        digitalWrite(pinLed,HIGH);
        delay(500);
        digitalWrite(pinLed,LOW);
        delay(500);
        }
      
      }
    
    delay(10000); // Pequeño delay después de iniciar el PCA9685
    digitalWrite(pinLed,HIGH);
    pwm.setPWMFreq(200); // Frecuencia de PWM para LED

    

}

void loop() {
    Serial.println("Aumentar brillo...");
    
    for (uint16_t i = 0; i < 4096; i += 50) {
        //pwm.setPWM(ledChannel, 0, i);
        if (pwm.setPWM(ledChannel, 0, i)){
      Serial.println("Error:No se seteo pwm +");

      while(1){
        digitalWrite(pinLed,HIGH);
        delay(500);
        digitalWrite(pinLed,LOW);
        delay(500);
        }
      
      }
        delay(30);
    }

    Serial.println("Disminuir brillo...");
    // Disminuir brillo
    for (int16_t i = 4095; i < 0; i -= 50) {
         if (pwm.setPWM(ledChannel, 0, i)){
      Serial.println("Error:No se seteo pwm -");

      while(1){
        digitalWrite(pinLed,HIGH);
        delay(500);
        digitalWrite(pinLed,LOW);
        delay(500);
        }
      
      }
        delay(45);
    }
    Serial.println("Reset");
    digitalWrite(pinLed,LOW);
    delay(100);
}
*/
