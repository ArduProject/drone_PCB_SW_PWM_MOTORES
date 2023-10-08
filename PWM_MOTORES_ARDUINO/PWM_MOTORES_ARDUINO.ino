/*
   Generación de señales PWM para control de motores
   Versión para Arduino.
*/

// Tiempo de ciclo
#define tiempo_ciclo 5000

// Pines asignados
#define pin_RC        2   // Pin para lectura del mando vía PPM
#define pin_motor1    3  // Pin motor 1
#define pin_motor2    4  // Pin motor 2
#define pin_motor3    5  // Pin motor 3
#define pin_motor4    6  // Pin motor 4

// Receptor RC
#define numero_canales 8
/*
   Mando_canal[0] = -
   Mando_canal[1] = ROLL
   Mando_canal[2] = PITCH
   Mando_canal[3] = THROTTLE
   Mando_canal[4] = YAW
   Mando_canal[5] = SWD º
   Mando_canal[6] = SWC
   Mando_canal[7] = -
*/

long pulso_instante[numero_canales * 2 + 2], rise_instante_ant;
int Mando_canal[numero_canales], canal_ant[numero_canales], Mando_Throttle;
int contador_flaco = 1;

// Tiempo ciclo
long tiempo_nuevo_ciclo, tiempo_motores_start, contador_ciclos;

// Periodo de las señales PWM
int ESC1_us, ESC2_us, ESC3_us, ESC4_us;

void setup() {
  Serial.begin(115200);   // Para Serial.print()

  // Declarar pines PWM y poner señales a LOW
  pinMode(pin_motor1, OUTPUT);    // MOTOR 1
  pinMode(pin_motor2, OUTPUT);    // MOTOR 2
  pinMode(pin_motor3, OUTPUT);    // MOTOR 3
  pinMode(pin_motor4, OUTPUT);    // MOTOR 4
  digitalWrite(pin_motor1, LOW);  // MOTOR 1
  digitalWrite(pin_motor2, LOW);  // MOTOR 2
  digitalWrite(pin_motor3, LOW);  // MOTOR 3
  digitalWrite(pin_motor4, LOW);  // MOTOR 4

  // Declarar interrupción en pin_RC. CHANGE = se activa tanto con flanco positivo como con flanco negativo
  pinMode(pin_RC, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_RC), interrupt_RC, CHANGE);
}

void loop() {
  // Un nuevo ciclo cada 5000us (5ms)
  while (micros() - tiempo_nuevo_ciclo < tiempo_ciclo);
  tiempo_nuevo_ciclo = micros();

  PWM_GEN();     // Generar señales PWM para los motores

  // Visualizar por Monitor Serie
  Serial.println(Mando_Throttle);
  contador_ciclos++;
}

void RECEPTOR_RC() {
  // Solo ejecutamos esta parte si hemos recibido toda la ráfaga, los 18 flancos con la informacion
  // de todos los canales.
  if (contador_flaco == 18) {
    for (uint8_t i = 1; i <= numero_canales; i++) {
      // De estos 18 flancos, el primero y el último no nos aportan información. Recorremos los demás
      // flancos. Para calcular la lontigud de cada pulso, hacemos la resta del flanco actual, menos el
      // flanco anterior. Al haber guardado el instante (micros()) en el que se da cada flanco, con esta
      // resta calculamos la anchura de cada pulso.
      Mando_canal[i] = pulso_instante[2 * i] - pulso_instante[2 * i - 1];

      // De forma aleatoria el repector envía señales erroneas (ruido). Es necesario filtrar.
      if (i != 5 && canal_ant[i] > 500 && abs(Mando_canal[i] - canal_ant[i]) > 500)Mando_canal[i] = canal_ant[i];
      if (abs(Mando_canal[5] - canal_ant[5]) > 2000)Mando_canal[5] = canal_ant[5];
      canal_ant[i] = Mando_canal[i];
    }
  }

  // Mapeamos las lecturas del mando RC de 1000 a 2000.
  Mando_Throttle  = map(Mando_canal[3], 729, 1600, 1000, 2000);
}

void interrupt_RC() {
  // Aunque el receptor es de 6 canales recibimos 8 pulsos, recibimos 18 flancos (8*2+2). Para
  // transmitir n canales, recibiremos n+2 flancos tanto positivos como negativos.
  // Se pone el contador a 0:
  if (micros() - pulso_instante[contador_flaco - 1] > 2500) contador_flaco = 0;
  // Guardamos en esta variable el instante (micros()) en el que se lee un flanco, tanto positivo como negativo:
  // Índice del array de 0 a 17 --> 18 elementos
  pulso_instante[contador_flaco] = micros();
  contador_flaco++;
}

void PWM_GEN() {

  // Para generar las 4 señales PWM, el primer paso es poner estas señales a 1 (HIGH).
  digitalWrite(pin_motor1, HIGH);  // MOTOR 1
  digitalWrite(pin_motor2, HIGH);  // MOTOR 2
  digitalWrite(pin_motor3, HIGH);  // MOTOR 3
  digitalWrite(pin_motor4, HIGH);  // MOTOR 4
  tiempo_motores_start = micros();

  RECEPTOR_RC(); // Leer mando RC

  // La consigna throttle del mando pasa directamente a los motores
  ESC1_us = Mando_Throttle;
  ESC2_us = Mando_Throttle;
  ESC3_us = Mando_Throttle;
  ESC4_us = Mando_Throttle;

  if (ESC1_us > 2000)ESC1_us = 2000;
  if (ESC2_us > 2000)ESC2_us = 2000;
  if (ESC3_us > 2000)ESC3_us = 2000;
  if (ESC4_us > 2000)ESC4_us = 2000;
  if (ESC1_us < 1000)ESC1_us = 1000;
  if (ESC2_us < 1000)ESC2_us = 1000;
  if (ESC3_us < 1000)ESC3_us = 1000;
  if (ESC4_us < 1000)ESC4_us = 1000;

  // Cuando se cumpa el tiempo de PWM definido en ESCx_us, se pasa cada señal a 0 (LOW) para terminar el ciclo PWM.
  // Más detalles en https://arduproject.es/control-de-estabilidad-y-pid/
  while (digitalRead(pin_motor1) == HIGH || digitalRead(pin_motor2) == HIGH || digitalRead(pin_motor3) == HIGH || digitalRead(pin_motor4) == HIGH) {
    if (tiempo_motores_start + ESC1_us <= micros()) digitalWrite(pin_motor1, LOW); // MOTOR 1
    if (tiempo_motores_start + ESC2_us <= micros()) digitalWrite(pin_motor2, LOW); // MOTOR 2
    if (tiempo_motores_start + ESC3_us <= micros()) digitalWrite(pin_motor3, LOW); // MOTOR 3
    if (tiempo_motores_start + ESC4_us <= micros()) digitalWrite(pin_motor4, LOW); // MOTOR 4
  }
}
