#include "MotorDriver.h"

// Crea un oggetto L298N per controllare due motori
L298N motorDriver(0, 1, 1000, 8, 2);  // (pin 0 e 1 per PWM, frequenza 1000Hz, risoluzione 8 bit, 2 motori)

// Pin di controllo per i motori
int enA = 9;
int in1 = 10;
int in2 = 11;
int in3 = 12;
int in4 = 13;
int enB = 3;

void setup() {
  // Inizializzazione del driver con 6 pin per due motori
  motorDriver.init(enA, in1, in2, in3, in4, enB);

  // Se vuoi controllare solo un motore, puoi usare il metodo init per 3 pin
  // motorDriver.init(enA, in1, in2);

  Serial.begin(9600);
}

void loop() {
  // Muovi il motore in avanti con una velocità di PWM di 255 (massima velocità)
  motorDriver.forward(255);
  delay(2000);  // Esegui per 2 secondi

  // Muovi il motore indietro con una velocità di PWM di 255
  motorDriver.backward(255);
  delay(2000);  // Esegui per 2 secondi

  // Ferma entrambi i motori
  motorDriver.stop();
  delay(1000);  // Ferma per 1 secondo
}

