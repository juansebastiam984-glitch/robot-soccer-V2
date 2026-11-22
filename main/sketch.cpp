// Paso 4: giroscopio MPU6050 con correccion de rumbo para ir recto.
// Motores y buzzer igual que en el paso 3.
//
// Motor A = rueda DERECHA (la mas debil), motor B = rueda IZQUIERDA (se frena un 15 %).
// Control de rumbo PID (KP 2, KI 1, KD 0.15). La correccion aprendida de cada sentido se guarda en la
// flash al parar, asi que el carro arranca ya compensado. Ver MODO_COMPETICION mas abajo.
//
// Al encender, deja el robot QUIETO en el suelo 2 s: se calibra el giroscopio.
//   1 pitido corto = calibracion correcta, 2 pitidos = fallo (sin correccion hasta reiniciar).
// Signo comprobado: girar a la IZQUIERDA hace subir el Yaw (SIGNO_GIRO = 1).
//
// Correccion de rumbo: al avanzar o retroceder con R2/L2 y el stick centrado, el robot recuerda
// hacia donde apunta y corrige las ruedas para no desviarse. Si tocas el stick, lo manejas tu;
// al soltarlo espera 150 ms y recuerda el rumbo nuevo.
//
// L1 = apaga/enciende la correccion de rumbo. OPTIONS = recalibrar el giroscopio (carro plano y quieto).
// Solo con MODO_COMPETICION = false:
// Modo prueba (con las ruedas en el aire): mientras mantienes un boton, gira a velocidad fija
//   Cuadrado = solo motor A (derecha)   Circulo = solo motor B (izquierda)
//   Triangulo = A y B adelante          X (Cruz) = A y B atras
// R1 (carro en el SUELO, quieto, con espacio): autocalibracion de motores, mide cuanto empuja cada uno.
// Modo conducir: R2 = adelante, L2 = atras, stick izquierdo X = girar.

#include <Arduino.h>
#include <Wire.h>
#include <Bluepad32.h>
#include <Preferences.h>

// TB6612FNG
const int pinSTBY = 13;
const int pinAIN1 = 27;
const int pinAIN2 = 26;
const int pinPWMA = 14;
const int pinBIN1 = 25;
const int pinBIN2 = 33;
const int pinPWMB = 32;

// MPU6050 (I2C)
const int PIN_SDA = 21;
const int PIN_SCL = 22;

// Buzzer pasivo: GPIO18 -> resistencia 100-220 ohm -> buzzer -> GND
const int pinBuzzer = 18;
const int BUZZER_FREQ_HZ = 2700;
const int BUZZER_VOLUMEN = 30;                    // 0 a 100
const unsigned long DURACION_BUSCANDO = 1000;     // ms sonando / ms callado sin mando
const unsigned long DURACION_CONEXION = 2000;     // ms de pitido al conectar

// Si una rueda gira al reves de lo esperado, cambia su constante a true.
const bool INVERTIR_A = false;
const bool INVERTIR_B = false;

// Equilibrio fijo entre motores: la rueda A (derecha) empuja mas que la B, asi que se frena un %.
// Se mide aparte hacia adelante y hacia atras (hacia atras la diferencia es mayor).
// El giroscopio solo corrige lo que queda. Cambia 'A'/'B' o los % si el carro sigue desviandose.
const char MOTOR_MAS_LENTO = 'B';
const int REDUCCION_ADELANTE_PORC = 15;
const int REDUCCION_ATRAS_PORC = 15;
// Zona muerta del motor A (el debil): necesita mas PWM para empezar a girar, asi que al arrancar B se
// adelanta y el carro da un golpe de rumbo. El PWM de A empieza en este valor y se reescala hasta 255.
const int ZONA_MUERTA_A = 0;    // 30 no quito el golpe del arranque (medido): se deja en 0

// Giroscopio
const int SIGNO_GIRO = 1;                         // 1 o -1: debe subir el Yaw al girar a la izquierda
const float SENSIBILIDAD_GIRO = 65.5;             // LSB por grado/s (rango +-500 grados/s)
const float ZONA_MUERTA_GIRO = 0.3;               // grados/s que se ignoran
const unsigned long INTERVALO_LECTURA = 20;       // ms entre lecturas del MPU
const int MAX_ERRORES_I2C = 25;                   // errores seguidos antes de dar el MPU por perdido

// Como esta montada la placa (hacia donde apunta "arriba" en sus ejes) con el robot plano en el suelo.
// Medida en este carro, plano en el suelo (el MPU de este carro no esta montado igual que el del carro anterior).
const float ARRIBA_REF_X = -0.2967;
const float ARRIBA_REF_Y = -0.0261;
const float ARRIBA_REF_Z = -0.9546;
const float MAX_DESVIO_ARRIBA = 15.0;             // grados maximos de inclinacion al calibrar

// Correccion de rumbo (PID). Sube KP si tarda en corregir; baja KP o sube KD si oscila.
const bool CORREGIR_RUMBO = true;
const float KP = 2.0;                             // PWM por grado de error (suave: el carro es muy sensible)
const float KD = 0.15;                            // freno segun la velocidad de giro: amortigua el zigzag de 1-3 Hz
const float FILTRO_GIRO = 0.25;                   // 0 a 1: menor = mas suave (quita la sacudida del chasis)
const float CORRECCION_PASO_MAX = 2.0;            // PWM maximos que puede cambiar la correccion cada 20 ms
const float KI = 1.0;                             // corrige el desvio constante que deja la compensacion fija
const float INTEGRAL_MAX_PWM = 50;                // tope de la parte integral (en PWM)
const float LIMITE_ERROR_INTEGRAL = 5.0;          // la integral solo acumula si el error es menor que esto
const float ZONA_MUERTA_RUMBO = 0.5;              // grados de error que se ignoran (una zona grande provoca vaiven)
const float DESVIO_MAXIMO = 45.0;                 // si se desvia mas, acepta el rumbo nuevo
const int CORRECCION_MAXIMA = 70;                 // PWM maximo de correccion (poco: la compensacion fija ya hace lo grueso)
// Arranque hacia atras: A acelera mas lento que B y el carro da un golpe de rumbo (~20 grados) en los primeros
// 0.6 s. Al empezar una pasada atras desde parado se suma esta correccion extra, que se desvanece en ARRANQUE_MS.
const float ARRANQUE_ATRAS_PWM = -35;            // mismo signo que la correccion que lo evitaba (-39 => golpe de 5 grados)
const unsigned long ARRANQUE_MS = 800;
const unsigned long RETARDO_RELOCK = 150;         // ms tras soltar el stick antes de fijar rumbo nuevo
const float KD_FRENO_GIRO = 0.2;                  // freno del giro durante esa espera

// Suavizado del manejo, para que no vaya a tirones.
const float RAMPA_ACELERAR = 20;                  // PWM maximos que sube la velocidad cada 10 ms
const float RAMPA_FRENAR = 40;                    // PWM maximos que baja cada 10 ms (frena mas rapido)
const float FILTRO_STICK = 0.35;                  // 0 a 1 cada 10 ms: menor = giro mas suave, mayor = mas rapido
const float CURVA_GIRO = 1.2;                     // mayor que 1: giro mas fino cerca del centro del stick
const int GIRO_MAX_MOVIENDO = 255;                // giro maximo (PWM) al ir avanzando o retrocediendo
const int GIRO_MAX_EN_SITIO = 255;                // giro maximo (PWM) girando sin avanzar

// Reajuste lento del offset del giroscopio con el robot parado y quieto.
const unsigned long PARADO_MIN_AUTOCERO = 500;    // ms parado antes de empezar
const float UMBRAL_AUTOCERO = 150;               // LSB (~2.3 grados/s) maximo para considerarlo quieto
const float FACTOR_AUTOCERO = 0.002;             // filtro lento (~10 s)

// Version de competicion: true = sin mensajes por serie, sin registro/trazas en la flash y sin botones de
// prueba (R1, SHARE, Cuadrado/Circulo/Triangulo/X, cruceta abajo). La memoria de rumbo se sigue guardando.
// false = todo el diagnostico vuelve (para medir sin cable con grabar.py).
const bool MODO_COMPETICION = true;

const bool MOSTRAR_YAW = false;                   // imprime el Yaw cuando cambia (para pruebas)
const bool MOSTRAR_MOTORES = false;               // imprime los valores de los motores
const bool DEBUG_RUMBO = !MODO_COMPETICION;                    // imprime el control de rumbo (para afinar)
const bool DEBUG_VIBRACION = !MODO_COMPETICION;                // al parar, imprime el giro medido cada 20 ms durante la marcha

const int VEL_PRUEBA = 150;                       // 0 a 255, para el modo prueba
const unsigned long TIEMPO_MAX_SIN_DATOS = 200;   // ms sin datos del mando => frenar
const int ZONA_MUERTA_GATILLO = 15;               // gatillos 0 a 1023
const int ZONA_MUERTA_STICK = 40;                 // stick -511 a 512

ControllerPtr miMando = nullptr;
unsigned long ultimoDato = 0;
bool cortePorFaltaDeDatos = false;
int ultimoA = 0, ultimoB = 0;
unsigned long buzzerHasta = 0;

// Estado del giroscopio
uint8_t mpuAddr = 0x68;
int16_t ax, ay, az, gx, gy, gz;
bool mpuListo = false;
bool giroCalibrado = false;
float gxOffset = 0, gyOffset = 0, gzOffset = 0;      // offsets del giroscopio (LSB)
float arribaX = 0, arribaY = 0, arribaZ = 0;         // vector "arriba" en los ejes del sensor
float anguloZ = 0;                                    // rumbo en grados (sube al girar a la izquierda)
float velAngularZ = 0;                                // grados/s
float velAngularFiltrada = 0;                         // grados/s, suavizada para el termino D
float yawImpreso = 0;
bool primeraLectura = true;
unsigned long ultimaLectura = 0, tiempoAnterior = 0;
int erroresI2CSeguidos = 0;

// Estado de la correccion de rumbo
bool corrigiendo = false;           // true = esta manteniendo un rumbo
float rumboObjetivo = 0;
float integralRumbo = 0;
// Correccion aprendida: [0] adelante, [1] atras. Empieza en 0 porque la compensacion fija
// (REDUCCION_PORC) ya cubre la diferencia grande entre motores; el control ajusta lo que queda.
float memoriaIntegral[2] = {0.0, 0.0};
float memoriaGuardadaPWM[2] = {0.0, 0.0};   // lo ultimo escrito en la flash (en PWM)
int direccionActual = 0;            // 0 = adelante, 1 = atras
float correccion = 0;               // PWM de correccion: positivo = girar a la izquierda
bool esperandoRelock = false;
unsigned long ultimoParado = 0, inicioPasada = 0;   // para el empuje del arranque
bool pasadaAtras = false;
unsigned long finManual = 0;
float velSuave = 0, giroSuave = 0;  // velocidad y giro ya suavizados (rampa y filtro)
const int VIB_N = 300;              // 6 s de registro a 50 Hz
float vibBuf[VIB_N];
int vibN = 0;
float vibSuma = 0;
bool motoresParados = true;
bool rumboActivo = CORREGIR_RUMBO;  // se puede apagar/encender con L1 mientras manejas
bool l1Anterior = false;
unsigned long cortesMando = 0;      // veces que el mando dejo de enviar datos (frenada por falta de datos)
unsigned long huecoMaxMando = 0;    // hueco mas largo sin datos en la ventana actual (ms)
int ordenA = 0, ordenB = 0;        // ultimo valor pedido a cada motor (para el registro de depuracion)
int ultimoEjeX = 0;                 // stick izquierdo X, para el registro de depuracion

// ---------- Motores ----------

void motor(int pinIN1, int pinIN2, int pinPWM, int velocidad, bool invertir) {
  if (invertir) velocidad = -velocidad;
  velocidad = constrain(velocidad, -255, 255);
  if (velocidad >= 0) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
  } else {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
    velocidad = -velocidad;
  }
  ledcWrite(pinPWM, velocidad);
}

// Pone los dos motores y avisa por serie solo cuando cambian de forma notable.
void mover(int a, int b) {
  a = constrain(a, -255, 255);
  b = constrain(b, -255, 255);

  // El motor mas rapido se frena un porcentaje (el numero que se imprime es el pedido, sin frenar).
  // Cada rueda se frena segun hacia donde gira (adelante o atras).
  int factorA = 100 - ((a >= 0) ? REDUCCION_ADELANTE_PORC : REDUCCION_ATRAS_PORC);
  int factorB = 100 - ((b >= 0) ? REDUCCION_ADELANTE_PORC : REDUCCION_ATRAS_PORC);
  int salidaA = (MOTOR_MAS_LENTO == 'A') ? a * factorA / 100 : a;
  int salidaB = (MOTOR_MAS_LENTO == 'B') ? b * factorB / 100 : b;
  if (salidaA != 0) {
    int m = abs(salidaA);
    m = ZONA_MUERTA_A + m * (255 - ZONA_MUERTA_A) / 255;
    salidaA = salidaA > 0 ? m : -m;
  }
  motor(pinAIN1, pinAIN2, pinPWMA, salidaA, INVERTIR_A);
  motor(pinBIN1, pinBIN2, pinPWMB, salidaB, INVERTIR_B);

  motoresParados = (a == 0 && b == 0);
  ordenA = a;
  ordenB = b;

  bool cambio = abs(a - ultimoA) >= 25 || abs(b - ultimoB) >= 25 ||
                ((a == 0) != (ultimoA == 0)) || ((b == 0) != (ultimoB == 0));
  if (cambio) {
    if (MOSTRAR_MOTORES) Serial.printf("Motor A (der) = %d   Motor B (izq) = %d\n", a, b);
    ultimoA = a;
    ultimoB = b;
  }
}

void detener() {
  velSuave = 0;
  giroSuave = 0;
  mover(0, 0);
}

// Correccion extra solo al arrancar hacia atras desde parado; baja a 0 en ARRANQUE_MS.
float extraArranque(unsigned long ahora) {
  if (!pasadaAtras) return 0;
  unsigned long t = ahora - inicioPasada;
  if (t >= ARRANQUE_MS) return 0;
  return ARRANQUE_ATRAS_PWM * (1.0f - (float)t / ARRANQUE_MS);
}

// Avanza o retrocede a 'velocidad' y gira con 'giro' (positivo = a la derecha). Si una rueda se pasa
// de 255, el exceso se resta a la otra: asi el giro NO se pierde aunque vayas a fondo con R2.
void moverConGiro(int velocidad, int giro) {
  int a = velocidad - giro;   // rueda derecha (A)
  int b = velocidad + giro;   // rueda izquierda (B)
  if (b > 255) { a -= b - 255; b = 255; }
  else if (b < -255) { a += -255 - b; b = -255; }
  if (a > 255) { b -= a - 255; a = 255; }
  else if (a < -255) { b += -255 - a; a = -255; }
  mover(a, b);
}

// Avanza o retrocede a 'velocidad' con una correccion c (positivo = girar a la izquierda:
// la rueda derecha A mas rapida y la izquierda B mas lenta). Sirve igual hacia atras.
void moverConCorreccion(int velocidad, float c) {
  int limite = (CORRECCION_MAXIMA < abs(velocidad)) ? CORRECCION_MAXIMA : abs(velocidad);
  int cc = constrain((int)c, -limite, limite);
  int a = velocidad + cc;   // rueda derecha
  int b = velocidad - cc;   // rueda izquierda
  // Si una rueda pasa del maximo, se le resta el exceso a la otra para no perder correccion.
  if (a > 255) { b -= a - 255; a = 255; }
  else if (a < -255) { b += -255 - a; a = -255; }
  if (b > 255) { a -= b - 255; b = 255; }
  else if (b < -255) { a += -255 - b; b = -255; }
  mover(a, b);
}

// ---------- Buzzer ----------

void pitar(unsigned long ms) { buzzerHasta = millis() + ms; }

void buzzerEncender(bool encender) {
  static bool encendido = false;
  if (encendido == encender) return;
  encendido = encender;
  ledcWrite(pinBuzzer, encender ? constrain(BUZZER_VOLUMEN, 0, 100) : 0);
}

void actualizarBuzzer(unsigned long ahora) {
  bool sonar = false;
  if ((long)(buzzerHasta - ahora) > 0) {
    sonar = true;                                              // pitido pedido (conexion)
  } else if (miMando == nullptr) {
    sonar = (ahora % (2 * DURACION_BUSCANDO)) < DURACION_BUSCANDO;   // buscando mando
  }
  buzzerEncender(sonar);
}

// Pitido bloqueante, solo para avisos durante el arranque.
void pitidoBloqueante(unsigned long ms) {
  ledcWrite(pinBuzzer, constrain(BUZZER_VOLUMEN, 0, 100));
  delay(ms);
  ledcWrite(pinBuzzer, 0);
}

// ---------- Giroscopio MPU6050 ----------

float diferenciaAngular(float actual, float anterior) {
  float d = actual - anterior;
  while (d > 180) d -= 360;
  while (d < -180) d += 360;
  return d;
}

int16_t leer16() {
  int16_t alto = Wire.read();
  int16_t bajo = Wire.read();
  return (int16_t)((alto << 8) | bajo);
}

bool escribirRegistro(uint8_t registro, uint8_t valor) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(registro);
  Wire.write(valor);
  return Wire.endTransmission() == 0;
}

// Busca el MPU en 0x68 y, si no, en 0x69.
bool buscarMPU() {
  uint8_t candidatas[2] = {mpuAddr, (uint8_t)(mpuAddr == 0x68 ? 0x69 : 0x68)};
  for (int i = 0; i < 2; i++) {
    Wire.beginTransmission(candidatas[i]);
    if (Wire.endTransmission() == 0) {
      mpuAddr = candidatas[i];
      return true;
    }
  }
  return false;
}

bool iniciarMPU() {
  if (!buscarMPU()) return false;
  escribirRegistro(0x6B, 0x01);   // despierta, reloj del giroscopio X
  escribirRegistro(0x1A, 0x03);   // filtro pasa-bajos ~42 Hz (menos ruido)
  escribirRegistro(0x1B, 0x08);   // giroscopio +-500 grados/s
  escribirRegistro(0x1C, 0x00);   // acelerometro +-2 g (explicito: el MPU conserva el valor al reiniciar la ESP32)
  return true;
}

int leerID() {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint16_t)mpuAddr, (uint8_t)1, true) != 1) return -1;
  return Wire.read();
}

// 0 = bien, 1 = error de I2C, 2 = lectura vacia (todo en cero).
int leerMPU() {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return 1;
  if (Wire.requestFrom((uint16_t)mpuAddr, (uint8_t)14, true) != 14) return 1;

  ax = leer16();
  ay = leer16();
  az = leer16();
  leer16();   // temperatura
  gx = leer16();
  gy = leer16();
  gz = leer16();

  if (ax == 0 && ay == 0 && az == 0 && gx == 0 && gy == 0 && gz == 0) return 2;
  return 0;
}

// Con el robot quieto en el suelo: mide el offset del giroscopio y hacia donde es "arriba".
bool calibrarGiroscopio() {
  Serial.println("Calibrando giroscopio: deja el robot QUIETO en el suelo (2 segundos)...");
  double sumaA[3] = {0, 0, 0};
  double sumaG[3] = {0, 0, 0};
  int16_t minimo[3] = {32767, 32767, 32767};
  int16_t maximo[3] = {-32768, -32768, -32768};
  int validas = 0;
  for (int i = 0; i < 200; i++) {
    if (leerMPU() == 0) {
      int16_t a[3] = {ax, ay, az};
      int16_t g[3] = {gx, gy, gz};
      for (int k = 0; k < 3; k++) {
        sumaA[k] += a[k];
        sumaG[k] += g[k];
        if (g[k] < minimo[k]) minimo[k] = g[k];
        if (g[k] > maximo[k]) maximo[k] = g[k];
      }
      validas++;
    }
    delay(10);
  }

  if (validas < 150) {
    Serial.println("Calibracion fallida: pocas lecturas buenas del sensor.");
    return false;
  }
  for (int k = 0; k < 3; k++) {
    if (maximo[k] - minimo[k] > 500) {
      Serial.println("Calibracion descartada: el robot se movio. Reinicia y dejalo quieto.");
      return false;
    }
  }

  float mx = sumaA[0] / validas;
  float my = sumaA[1] / validas;
  float mz = sumaA[2] / validas;
  float modulo = sqrt(mx * mx + my * my + mz * mz);
  if (modulo < 12000 || modulo > 21000) {
    Serial.println("Calibracion descartada: la aceleracion no es 1 G (se mueve o el sensor falla).");
    return false;
  }

  // El robot debe estar plano en el suelo: "arriba" no puede diferir mucho de como esta montada la placa.
  float nx = mx / modulo, ny = my / modulo, nz = mz / modulo;
  float punto = nx * ARRIBA_REF_X + ny * ARRIBA_REF_Y + nz * ARRIBA_REF_Z;
  float desvioArriba = acos(constrain(punto, -1.0f, 1.0f)) * 180.0 / PI;
  if (desvioArriba > MAX_DESVIO_ARRIBA) {
    Serial.printf("Calibracion descartada: el robot esta inclinado %.0f grados respecto a su posicion normal. Ponlo plano en el suelo.\n",
                  desvioArriba);
    return false;
  }

  gxOffset = sumaG[0] / validas;
  gyOffset = sumaG[1] / validas;
  gzOffset = sumaG[2] / validas;
  arribaX = nx;
  arribaY = ny;
  arribaZ = nz;

  float c = fabsf(arribaZ);
  if (c > 1) c = 1;
  Serial.printf("Calibracion lista. Offsets gx=%.1f gy=%.1f gz=%.1f  |  placa inclinada %.1f grados (%s)\n",
                gxOffset, gyOffset, gzOffset, acos(c) * 180.0 / PI,
                arribaZ < 0 ? "boca abajo" : "boca arriba");
  Serial.printf("Calibracion arriba=(%.4f, %.4f, %.4f) desvio de la referencia=%.1f grados\n",
                arribaX, arribaY, arribaZ, desvioArriba);
  return true;
}

// ---------- Correccion de rumbo ----------

// Recuerda hacia donde apunta el robot ahora y empieza a corregir.
void fijarRumbo() {
  rumboObjetivo = anguloZ;
  integralRumbo = memoriaIntegral[direccionActual];   // arranca con lo aprendido, no desde cero
  correccion = KI * integralRumbo;                    // y ya con su correccion, sin esperar
  velAngularFiltrada = 0;
  corrigiendo = rumboActivo && mpuListo && giroCalibrado && !primeraLectura;
  if (DEBUG_RUMBO && corrigiendo) Serial.printf("RUMBO FIJADO en %.1f\n", rumboObjetivo);
}

// PID: calcula 'correccion' segun cuanto se desvio del rumbo. giro = velocidad de giro (grados/s).
void corregirRumbo(float giro, float dt) {
  float error = diferenciaAngular(rumboObjetivo, anguloZ);   // positivo = hay que girar a la izquierda

  if (fabsf(error) > DESVIO_MAXIMO) {   // golpe fuerte o giro grande: acepta el rumbo nuevo
    rumboObjetivo = anguloZ;
    error = 0;
    Serial.println("Desvio mayor a 45 grados: nuevo rumbo.");
  }

  if (error > ZONA_MUERTA_RUMBO) error -= ZONA_MUERTA_RUMBO;
  else if (error < -ZONA_MUERTA_RUMBO) error += ZONA_MUERTA_RUMBO;
  else error = 0;

  // Con error pequeno, la integral aprende. Con error grande no se carga de mas, pero si se puede
  // DESCARGAR (error de signo contrario a la integral): asi no queda atascada con una memoria que sobra.
  bool descarga = (error > 0 && integralRumbo < 0) || (error < 0 && integralRumbo > 0);
  if ((fabsf(error) < LIMITE_ERROR_INTEGRAL || descarga) && KI > 0) {
    integralRumbo += error * dt;
    float tope = INTEGRAL_MAX_PWM / KI;
    integralRumbo = constrain(integralRumbo, -tope, tope);
    memoriaIntegral[direccionActual] = integralRumbo;
  }

  float deseada = KP * error + KI * integralRumbo - KD * giro;
  // La correccion no puede cambiar de golpe: asi las ruedas no dan tirones.
  correccion += constrain(deseada - correccion, -CORRECCION_PASO_MAX, CORRECCION_PASO_MAX);

  if (DEBUG_RUMBO) {
    static int n = 0;
    if (++n % 5 == 0) {   // una linea cada ~100 ms
      Serial.printf("R yaw=%.1f err=%.1f int=%.1f giro=%.1f corr=%.0f A=%d B=%d x=%d\n",
                    anguloZ, diferenciaAngular(rumboObjetivo, anguloZ), integralRumbo, giro,
                    correccion, ordenA, ordenB, ultimoEjeX);
    }
  }
}

// ---------- Registro de manejo por tramos ----------

// Resumen por tramo (no muestras sueltas): cabe en la NVS y se guarda con el carro parado, asi se
// puede manejar sin cable USB y leerlo despues. Tramo = correccion ON/OFF x
// {adelante, atras} x {baja <100, media 100-179, alta >=180} (stick centrado), curva (stick tocado) o giro en sitio.
struct Tramo {
  uint32_t n, nCorr, nTope;               // ticks de 20 ms; con correccion activa; con |correccion| al tope
  float sumA, sumB, sumGiro, sumCorr, maxErr;
};
const int RUTA_TRAMOS = 16;
Tramo ruta[RUTA_TRAMOS];
bool rutaSucia = false;
bool marchaSinGuardar = false;           // el carro anduvo: hay que guardar la memoria de rumbo al parar
                   // hay datos nuevos sin guardar en la flash
bool enPrueba = false;                    // modo prueba (Cuadrado/Circulo/...): no se registra
unsigned long ultimaMarcha = 0, ultimoGuardadoRuta = 0;

const char* nombreTramo(int i) {
  static const char* nombres[8] = {"adelante-baja", "adelante-media", "adelante-alta",
                                   "atras-baja", "atras-media", "atras-alta", "curva", "giro-en-sitio"};
  return nombres[i % 8];
}

void rutaAcumular(float giroGrados) {
  float v = (ordenA + ordenB) / 2.0;
  int t;
  if (fabsf(v) < 15) t = 7;
  else if (abs(ultimoEjeX) > ZONA_MUERTA_STICK) t = 6;
  else {
    float av = fabsf(v);
    t = (v >= 0 ? 0 : 3) + (av < 100 ? 0 : (av < 180 ? 1 : 2));
  }
  Tramo& r = ruta[(rumboActivo ? 0 : 8) + t];
  r.n++;
  r.sumA += ordenA;
  r.sumB += ordenB;
  r.sumGiro += giroGrados;
  if (corrigiendo) {
    r.nCorr++;
    r.sumCorr += correccion;
    if (fabsf(correccion) >= CORRECCION_MAXIMA - 1) r.nTope++;
    float e = fabsf(diferenciaAngular(rumboObjetivo, anguloZ));
    if (e > r.maxErr) r.maxErr = e;
  }
  rutaSucia = true;
}

// Traza de la ultima pasada recta con correccion (>= 2 s): los ultimos 6 s a 50 Hz, para ver el zigzag.
// Valores x10 en int16 (error en grados, giro en grados/s, correccion en PWM).
const int TRAZA_N = 300;
struct Traza { int16_t n, pwm; int16_t err[TRAZA_N], giro[TRAZA_N], corr[TRAZA_N]; };
Traza traza[2];     // la ultima pasada completa de cada sentido: [0] adelante, [1] atras (la que se guarda)
Traza trazaAct;     // la pasada en curso (buffer circular)
int trazaPos = 0;
uint32_t trazaTot = 0;
float trazaSumPwm = 0;
bool trazaSucia = false;

int16_t x10(float v) { return (int16_t)constrain(v * 10.0f, -32000.0f, 32000.0f); }

void trazaPaso(float giroGrados) {
  if (corrigiendo && !enPrueba) {
    if (trazaPos < TRAZA_N) {   // se queda con los PRIMEROS 6 s de la pasada (el arranque)
      trazaAct.err[trazaPos] = x10(diferenciaAngular(rumboObjetivo, anguloZ));
      trazaAct.giro[trazaPos] = x10(giroGrados);
      trazaAct.corr[trazaPos] = x10(correccion);
      trazaPos++;
    }
    trazaTot++;
    trazaSumPwm += (ordenA + ordenB) / 2.0;
    return;
  }
  if (trazaTot >= 100) {   // la pasada acaba de terminar y duro al menos 2 s: se queda como "la ultima"
    int n = trazaTot < TRAZA_N ? trazaTot : TRAZA_N;
    Traza& tr = traza[trazaSumPwm >= 0 ? 0 : 1];   // sentido de la pasada segun su PWM medio
    int ini = 0;
    tr.n = n;
    tr.pwm = (int16_t)(trazaSumPwm / trazaTot);
    for (int i = 0; i < n; i++) {
      int k = (ini + i) % TRAZA_N;
      tr.err[i] = trazaAct.err[k];
      tr.giro[i] = trazaAct.giro[k];
      tr.corr[i] = trazaAct.corr[k];
    }
    trazaSucia = true;
  }
  trazaTot = 0;
  trazaPos = 0;
  trazaSumPwm = 0;
}

// Lee el MPU cada INTERVALO_LECTURA ms e integra el giro vertical para obtener el rumbo.
void actualizarMPU(unsigned long ahora) {
  if (!mpuListo) return;
  if (ahora - ultimaLectura < INTERVALO_LECTURA) return;
  ultimaLectura = ahora;

  if (leerMPU() != 0) {
    erroresI2CSeguidos++;
    if (erroresI2CSeguidos == 5) correccion = 0;   // dato viejo: mejor no corregir
    if (erroresI2CSeguidos >= MAX_ERRORES_I2C) {
      mpuListo = false;
      corrigiendo = false;
      correccion = 0;
      Serial.println("MPU6050 sin respuesta: se desactiva el giroscopio (reinicia para reintentar).");
    }
    return;
  }
  erroresI2CSeguidos = 0;

  if (primeraLectura) {
    tiempoAnterior = ahora;
    primeraLectura = false;
    return;
  }

  float dt = (ahora - tiempoAnterior) / 1000.0;
  tiempoAnterior = ahora;

  // Reajuste lento del offset del giroscopio mientras el robot esta parado y quieto.
  static bool estabaParado = false;
  static unsigned long paradoDesde = 0;
  if (motoresParados && !estabaParado) paradoDesde = ahora;
  estabaParado = motoresParados;
  if (motoresParados && giroCalibrado && ahora - paradoDesde >= PARADO_MIN_AUTOCERO &&
      fabsf(gx - gxOffset) < UMBRAL_AUTOCERO && fabsf(gy - gyOffset) < UMBRAL_AUTOCERO &&
      fabsf(gz - gzOffset) < UMBRAL_AUTOCERO) {
    gxOffset += (gx - gxOffset) * FACTOR_AUTOCERO;
    gyOffset += (gy - gyOffset) * FACTOR_AUTOCERO;
    gzOffset += (gz - gzOffset) * FACTOR_AUTOCERO;
  }

  // Velocidad de giro alrededor del eje vertical (grados/s), positiva al girar a la izquierda.
  float giroVertical = (gx - gxOffset) * arribaX + (gy - gyOffset) * arribaY + (gz - gzOffset) * arribaZ;
  float vel = SIGNO_GIRO * giroVertical / SENSIBILIDAD_GIRO;
  if (fabsf(vel) <= ZONA_MUERTA_GIRO) vel = 0;
  velAngularZ = vel;
  anguloZ += vel * dt;
  if (!motoresParados && !enPrueba) {
    if (!MODO_COMPETICION) rutaAcumular(vel);
    ultimaMarcha = ahora;
    marchaSinGuardar = true;
  }

  // Registro rapido de la vibracion: guarda el giro (grados/s) cada 20 ms mientras los motores
  // andan, y lo imprime de una vez cuando el carro para.
  if (DEBUG_VIBRACION) {
    if (!motoresParados) {
      if (vibN < VIB_N) {
        vibBuf[vibN++] = vel;
        vibSuma += (abs(ordenA) + abs(ordenB)) / 2.0;
      }
    } else if (vibN > 0) {
      if (vibN >= 40) {
        Serial.printf("VIB n=%d dt_ms=20 pwm_medio=%.0f datos:", vibN, vibSuma / vibN);
        for (int i = 0; i < vibN; i++) Serial.printf(" %.0f", vibBuf[i]);
        Serial.println();
      }
      vibN = 0;
      vibSuma = 0;
    }
  }

  // Registro: lectura del giroscopio con el carro sin corregir (parado o con ruedas en el aire).
  if (DEBUG_RUMBO && !corrigiendo) {
    static unsigned long ultimoP = 0;
    if (ahora - ultimoP >= 500) {
      ultimoP = ahora;
      Serial.printf("P giro=%.2f gz=%d motores=%d,%d\n", giroVertical / SENSIBILIDAD_GIRO * SIGNO_GIRO,
                    (int)(gz - gzOffset), ordenA, ordenB);
    }
  }

  velAngularFiltrada += FILTRO_GIRO * (vel - velAngularFiltrada);
  if (!MODO_COMPETICION) trazaPaso(vel);
  if (corrigiendo) {
    corregirRumbo(velAngularFiltrada, dt);
  } else if (esperandoRelock) {
    // Acaba de soltar el stick: frena suavemente el giro que quedaba (la correccion cambia poco a poco).
    float deseada = (giroCalibrado ? -KD_FRENO_GIRO * velAngularFiltrada : 0);
    correccion += constrain(deseada - correccion, -CORRECCION_PASO_MAX, CORRECCION_PASO_MAX);
  }

  // Solo imprime cuando el rumbo cambia mas de 2 grados.
  if (MOSTRAR_YAW && fabsf(anguloZ - yawImpreso) > 2.0) {
    Serial.printf("Yaw = %.1f grados\n", anguloZ);
    yawImpreso = anguloZ;
  }
}

// ---------- Autodiagnostico ----------

// Con el carro EN EL AIRE y quieto: mide cuanto "gira" el giroscopio con los motores parados y
// en marcha. Si con los motores en marcha lee un giro, es ruido o vibracion de los motores.
// Se activa con SHARE; cancelalo con O (Circulo). Dura unos 20 s.
void autodiagnostico() {
  struct Fase { const char* nombre; int a, b; unsigned long ms; };
  Fase fases[] = {
    {"motores parados",   0,    0,    3000},
    {"adelante 150",      150,  150,  4000},
    {"adelante 255",      255,  255,  4000},
    {"atras 150",         -150, -150, 4000},
    {"motores parados",   0,    0,    3000},
  };
  Serial.println("AUTODIAG: empieza. Carro EN EL AIRE y sin moverlo. Cancela con O (Circulo).");

  for (auto& f : fases) {
    mover(f.a, f.b);
    unsigned long inicio = millis();
    while (millis() - inicio < 800) { BP32.update(); vTaskDelay(1); }   // deja que se estabilice

    double suma = 0, suma2 = 0;
    int n = 0;
    inicio = millis();
    while (millis() - inicio < f.ms) {
      BP32.update();
      if (!miMando || !miMando->isConnected() || miMando->b()) {
        detener();
        Serial.println("AUTODIAG: cancelado.");
        return;
      }
      if (leerMPU() == 0) {
        float g = SIGNO_GIRO * ((gx - gxOffset) * arribaX + (gy - gyOffset) * arribaY + (gz - gzOffset) * arribaZ)
                  / SENSIBILIDAD_GIRO;
        suma += g;
        suma2 += g * g;
        n++;
      }
      delay(10);
    }
    if (n > 0) {
      double media = suma / n;
      double sd = sqrt(max(0.0, suma2 / n - media * media));
      Serial.printf("AUTODIAG %-16s media=%+.2f grados/s  ruido(sd)=%.2f  n=%d\n", f.nombre, media, sd, n);
    }
  }
  detener();
  Serial.println("AUTODIAG: fin.");
  while (miMando && miMando->isConnected() && miMando->miscSelect()) { BP32.update(); vTaskDelay(1); }
  primeraLectura = true;   // el tiempo que paso aqui no cuenta para el rumbo
}

// ---------- Autocalibracion de motores ----------

// Mueve UNA rueda a 'pwm' (sin la reduccion fija) mientras la otra queda frenada (PWM 0 = freno corto
// del TB6612): el carro pivota y el MPU mide cuanto empuja ese motor. Devuelve false si se cancela.
// 'media' = giro medio en grados/s (positivo = a la izquierda); NAN si no hubo lecturas.
bool medirPivote(bool ruedaA, int pwm, float& media) {
  motor(pinAIN1, pinAIN2, pinPWMA, ruedaA ? pwm : 0, INVERTIR_A);
  motor(pinBIN1, pinBIN2, pinPWMB, ruedaA ? 0 : pwm, INVERTIR_B);
  double suma = 0;
  int n = 0;
  unsigned long inicio = millis();
  while (millis() - inicio < 2500) {
    BP32.update();
    if (!miMando || !miMando->isConnected() || miMando->b()) return false;
    if (millis() - inicio >= 1000 && leerMPU() == 0) {   // el primer segundo deja que se estabilice
      suma += SIGNO_GIRO * ((gx - gxOffset) * arribaX + (gy - gyOffset) * arribaY + (gz - gzOffset) * arribaZ)
              / SENSIBILIDAD_GIRO;
      n++;
    }
    delay(10);
  }
  media = n > 0 ? suma / n : NAN;
  return true;
}

// Resultados de la autocalibracion, guardados en la flash (NVS) cada vez que se mide un paso:
// se conservan sin cable y se imprimen solos al encender (setup).
const int CAL_N = 3;
const int calNiveles[CAL_N] = {100, 150, 200};
float calGiro[2][2][CAL_N];   // [rueda: 0=A 1=B][sentido: 0=adelante 1=atras][nivel], NAN = sin medir
Preferences prefs;

void calClave(char* k, int rueda, int sentido, int i) { snprintf(k, 8, "g%d%d%d", rueda, sentido, i); }

void calCargar() {
  prefs.begin("calib", true);
  for (int r = 0; r < 2; r++)
    for (int s = 0; s < 2; s++)
      for (int i = 0; i < CAL_N; i++) {
        char k[8]; calClave(k, r, s, i);
        calGiro[r][s][i] = prefs.getFloat(k, NAN);
      }
  prefs.end();
}

void calGuardar(int rueda, int sentido, int i) {
  char k[8]; calClave(k, rueda, sentido, i);
  prefs.begin("calib", false);
  prefs.putFloat(k, calGiro[rueda][sentido][i]);
  prefs.end();
}

void calBorrar() {
  prefs.begin("calib", false);
  prefs.clear();
  prefs.end();
  for (int r = 0; r < 2; r++)
    for (int s = 0; s < 2; s++)
      for (int i = 0; i < CAL_N; i++) calGiro[r][s][i] = NAN;
}

// Imprime lo guardado y el resultado. Esperado: A adelante gira a la izquierda (+) y B adelante a la
// derecha (-); hacia atras, al reves.
void calImprimir() {
  int hay = 0;
  for (int r = 0; r < 2; r++)
    for (int s = 0; s < 2; s++)
      for (int i = 0; i < CAL_N; i++)
        if (!isnan(calGiro[r][s][i])) {
          Serial.printf("CALIB motor %c %-8s pwm=%3d giro=%+7.1f grados/s%s\n", r == 0 ? 'A' : 'B',
                        s == 0 ? "adelante" : "atras", calNiveles[i], calGiro[r][s][i],
                        fabsf(calGiro[r][s][i]) > 450 ? "  (SATURADO)" : "");
          hay++;
        }
  if (hay == 0) { Serial.println("CALIB: sin datos guardados. Pulsa R1 para medir."); return; }

  Serial.printf("CALIB RESULTADO (%d de 12 pasos medidos; cociente |giro A| / |giro B| con el mismo PWM):\n", hay);
  for (int s = 0; s < 2; s++) {
    double sumaR = 0;
    int nR = 0;
    for (int i = 0; i < CAL_N; i++) {
      float a = calGiro[0][s][i], b = calGiro[1][s][i];
      const char* sen = s == 0 ? "adelante" : "atras";
      if (isnan(a) || isnan(b)) { Serial.printf("  %s pwm=%d: falta medir\n", sen, calNiveles[i]); continue; }
      bool signoA = (s == 0) ? (a > 0) : (a < 0);
      bool signoB = (s == 0) ? (b < 0) : (b > 0);
      if (fabsf(a) < 5 || fabsf(b) < 5 || !signoA || !signoB) {
        Serial.printf("  %s pwm=%d: A=%+.1f B=%+.1f  <- una rueda no se mueve o gira al reves de lo esperado\n",
                      sen, calNiveles[i], a, b);
        continue;
      }
      float r = fabsf(a) / fabsf(b);
      Serial.printf("  %s pwm=%d: A/B = %.2f\n", sen, calNiveles[i], r);
      sumaR += r;
      nR++;
    }
    if (nR > 0) {
      double r = sumaR / nR;
      if (r >= 1) Serial.printf("  => %s: A empuja %.2f veces mas. Reduccion sugerida de A: %.0f %% (MOTOR_MAS_LENTO='A')\n",
                                s == 0 ? "ADELANTE" : "ATRAS", r, (1 - 1 / r) * 100);
      else        Serial.printf("  => %s: B empuja %.2f veces mas. Reduccion sugerida de B: %.0f %% (MOTOR_MAS_LENTO='B')\n",
                                s == 0 ? "ADELANTE" : "ATRAS", 1 / r, (1 - r) * 100);
    }
  }
}

// Guardado en la flash (NVS, espacio "ruta"). Solo se escribe con el carro parado.
void rutaCargar() {
  memset(ruta, 0, sizeof(ruta));
  if (prefs.begin("ruta", true)) {
    if (prefs.getBytesLength("tramos") == sizeof(ruta)) prefs.getBytes("tramos", ruta, sizeof(ruta));
    prefs.end();
  }
}

void rutaGuardar() {
  prefs.begin("ruta", false);
  prefs.putBytes("tramos", ruta, sizeof(ruta));
  prefs.end();
  rutaSucia = false;
}

// Correccion aprendida (memoriaIntegral) en la flash: al encender, el carro arranca ya compensado.
// Se guarda en PWM (KI * integral) para que siga valiendo aunque cambie KI.
void memoriaCargar() {
  if (prefs.begin("rumbo", true)) {
    float tope = INTEGRAL_MAX_PWM / KI;
    for (int d = 0; d < 2; d++) {
      memoriaGuardadaPWM[d] = prefs.getFloat(d == 0 ? "adelante" : "atras", 0);
      memoriaIntegral[d] = constrain(memoriaGuardadaPWM[d] / KI, -tope, tope);
    }
    prefs.end();
  }
  Serial.printf("MEMORIA RUMBO: adelante=%+.1f PWM  atras=%+.1f PWM\n", KI * memoriaIntegral[0], KI * memoriaIntegral[1]);
}

void memoriaGuardarSiCambio() {
  bool cambio = false;
  for (int d = 0; d < 2; d++) if (fabsf(KI * memoriaIntegral[d] - memoriaGuardadaPWM[d]) >= 1.0) cambio = true;
  if (!cambio) return;
  prefs.begin("rumbo", false);
  for (int d = 0; d < 2; d++) {
    memoriaGuardadaPWM[d] = KI * memoriaIntegral[d];
    prefs.putFloat(d == 0 ? "adelante" : "atras", memoriaGuardadaPWM[d]);
  }
  prefs.end();
}

void trazaGuardar() {
  prefs.begin("ruta", false);
  prefs.putBytes("traza0", &traza[0], sizeof(Traza));
  prefs.putBytes("traza1", &traza[1], sizeof(Traza));
  if (prefs.isKey("traza")) prefs.remove("traza");   // la traza unica de la version anterior
  prefs.end();
  trazaSucia = false;
}

void trazaCargar() {
  memset(traza, 0, sizeof(traza));
  if (prefs.begin("ruta", true)) {
    for (int d = 0; d < 2; d++) {
      const char* k = d == 0 ? "traza0" : "traza1";
      if (prefs.getBytesLength(k) == sizeof(Traza)) prefs.getBytes(k, &traza[d], sizeof(Traza));
    }
    prefs.end();
  }
}

void trazaImprimir() {
  for (int d = 0; d < 2; d++) {
    const Traza& tr = traza[d];
    if (tr.n <= 0) { Serial.printf("TRAZA sentido=%s: sin datos.\n", d == 0 ? "adelante" : "atras"); continue; }
    Serial.printf("TRAZA sentido=%s n=%d dt_ms=20 pwm_medio=%d (columnas: err grados, giro grados/s, corr PWM)\n",
                  d == 0 ? "adelante" : "atras", tr.n, tr.pwm);
    for (int i = 0; i < tr.n; i++)
      Serial.printf("T%d %d %.1f %.1f %.1f\n", d, i, tr.err[i] / 10.0, tr.giro[i] / 10.0, tr.corr[i] / 10.0);
  }
  Serial.println("TRAZA: fin.");
}

void rutaBorrar() {
  memset(ruta, 0, sizeof(ruta));
  rutaGuardar();
  memset(traza, 0, sizeof(traza));
  trazaGuardar();
}

void rutaImprimir() {
  int hay = 0;
  Serial.println("RUTA (giro>0 = a la izquierda; corr>0 = corrige hacia la izquierda; tope = % del tiempo con la correccion al maximo):");
  for (int i = 0; i < RUTA_TRAMOS; i++) {
    const Tramo& r = ruta[i];
    if (r.n == 0) continue;
    hay++;
    Serial.printf("RUTA rumbo=%s %-15s t=%5.1fs A=%+5.0f B=%+5.0f giro=%+6.1f", i < 8 ? "ON " : "OFF", nombreTramo(i),
                  r.n * 0.02, r.sumA / r.n, r.sumB / r.n, r.sumGiro / r.n);
    if (r.nCorr > 0)
      Serial.printf(" corr=%+5.1f tope=%3.0f%% errmax=%.1f\n", r.sumCorr / r.nCorr, 100.0 * r.nTope / r.nCorr, r.maxErr);
    else
      Serial.println(" (sin correccion)");
  }
  if (hay == 0) Serial.println("RUTA: sin datos.");
}

// Con el carro EN EL SUELO, plano y con unos 30 cm libres alrededor (da vueltas pivotando).
// Mide el giro de cada motor por separado, adelante y atras, y lo guarda en la flash paso a paso:
// se puede hacer SIN cable USB y leer despues (los datos se imprimen al reiniciar la placa).
// Se activa con R1; cancela con O (Circulo). Dura ~1 minuto. Motores hasta PWM 200.
// Al terminar: 3 pitidos cortos = completo, 1 pitido largo = cancelado.
void autocalibrarMotores() {
  Serial.println("CALIB: empieza. Carro en el SUELO con espacio libre. Cancela con O (Circulo).");
  calBorrar();

  for (int rueda = 0; rueda < 2; rueda++) {
    for (int sentido = 0; sentido < 2; sentido++) {
      for (int i = 0; i < CAL_N; i++) {
        int pwm = sentido == 0 ? calNiveles[i] : -calNiveles[i];
        if (!medirPivote(rueda == 0, pwm, calGiro[rueda][sentido][i])) {
          detener();
          calGiro[rueda][sentido][i] = NAN;
          Serial.println("CALIB: cancelado (lo medido hasta ahora queda guardado).");
          pitidoBloqueante(600);
          while (miMando && miMando->isConnected() && miMando->b()) { BP32.update(); vTaskDelay(1); }
          primeraLectura = true;
          return;
        }
        calGuardar(rueda, sentido, i);
        detener();
        unsigned long pausa = millis();
        while (millis() - pausa < 1200) { BP32.update(); vTaskDelay(1); }   // que el carro se pare
      }
    }
  }

  calImprimir();
  Serial.println("CALIB: fin.");
  for (int k = 0; k < 3; k++) { pitidoBloqueante(120); delay(150); }
  while (miMando && miMando->isConnected() && miMando->r1()) { BP32.update(); vTaskDelay(1); }
  primeraLectura = true;   // el tiempo que paso aqui no cuenta para el rumbo
}

// Recalibra con el robot plano y quieto (boton OPTIONS). Si falla, conserva la calibracion anterior.
void recalibrar() {
  detener();
  if (calibrarGiroscopio()) {
    giroCalibrado = true;
    anguloZ = 0;
    corrigiendo = false;
    pitidoBloqueante(150);                             // 1 pitido: bien
  } else {
    pitidoBloqueante(150); delay(150); pitidoBloqueante(150);   // 2 pitidos: fallo
  }
  while (miMando && miMando->isConnected() && miMando->miscStart()) { BP32.update(); vTaskDelay(1); }
  primeraLectura = true;
}

// ---------- Mando ----------

void onConnectedController(ControllerPtr ctl) {
  if (miMando != nullptr) {
    Serial.println("Ya hay un mando conectado; se desconecta este.");
    ctl->disconnect();
    return;
  }
  miMando = ctl;
  ultimoDato = millis();
  cortePorFaltaDeDatos = false;
  pitar(DURACION_CONEXION);
  ControllerProperties p = ctl->getProperties();
  Serial.printf("Mando conectado: %s  MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
                ctl->getModelName().c_str(),
                p.btaddr[0], p.btaddr[1], p.btaddr[2], p.btaddr[3], p.btaddr[4], p.btaddr[5]);
}

void onDisconnectedController(ControllerPtr ctl) {
  if (miMando == ctl) {
    miMando = nullptr;
    detener();
    Serial.println("Mando desconectado: motores parados. Pulsa PS para reconectar.");
  }
}

void conducir() {
  bool cuadrado = miMando->x();
  bool circulo = miMando->b();
  bool triangulo = miMando->y();
  bool cruz = miMando->a();

  // Modo prueba: identifica cada motor y su sentido.
  if (!MODO_COMPETICION && (cuadrado || circulo || triangulo || cruz)) {
    int a = 0, b = 0;
    if (cuadrado) a = VEL_PRUEBA;
    if (circulo) b = VEL_PRUEBA;
    if (triangulo) { a = VEL_PRUEBA; b = VEL_PRUEBA; }
    if (cruz) { a = -VEL_PRUEBA; b = -VEL_PRUEBA; }
    corrigiendo = false;
    correccion = 0;
    esperandoRelock = false;
    velSuave = 0;
    giroSuave = 0;
    enPrueba = true;
    mover(a, b);
    return;
  }
  enPrueba = false;

  // Modo conducir.
  int r2 = miMando->throttle();
  int l2 = miMando->brake();
  if (r2 < ZONA_MUERTA_GATILLO) r2 = 0;
  if (l2 < ZONA_MUERTA_GATILLO) l2 = 0;
  int velocidadCruda = map(r2, 0, 1023, 0, 255) - map(l2, 0, 1023, 0, 255);

  int ejeX = miMando->axisX();
  ultimoEjeX = ejeX;
  bool stickTocado = abs(ejeX) > ZONA_MUERTA_STICK;
  float giroCrudo = 0;
  if (stickTocado) {
    float x = ((float)abs(ejeX) - ZONA_MUERTA_STICK) / (512.0 - ZONA_MUERTA_STICK);
    x = powf(constrain(x, 0.0f, 1.0f), CURVA_GIRO);   // curva: mas fino cerca del centro
    float maximo = (abs(velocidadCruda) >= 15) ? GIRO_MAX_MOVIENDO : GIRO_MAX_EN_SITIO;
    giroCrudo = ((ejeX > 0) ? x : -x) * maximo;
  }

  unsigned long ahora = millis();

  // Suavizado cada 10 ms: la velocidad sube con rampa (y baja mas rapido) y el giro con filtro.
  static unsigned long tSuave = 0;
  if (ahora - tSuave >= 10) {
    tSuave = ahora;
    float paso = (fabsf(velocidadCruda) >= fabsf(velSuave)) ? RAMPA_ACELERAR : RAMPA_FRENAR;
    velSuave += constrain(velocidadCruda - velSuave, -paso, paso);
    giroSuave += FILTRO_STICK * (giroCrudo - giroSuave);
  }
  int velocidad = (int)velSuave;
  int giro = (int)giroSuave;

  if (velocidad != 0) direccionActual = (velocidad > 0) ? 0 : 1;

  if (abs(velocidad) >= 15 && !stickTocado) {
    // Avanzando o retrocediendo con el stick centrado: corrige el rumbo.
    if (esperandoRelock) {
      // Acaba de soltar el stick: espera a que se asiente y despues fija el rumbo nuevo.
      // (la correccion de frenado la calcula actualizarMPU, poco a poco)
      moverConCorreccion(velocidad, correccion);
      if (ahora - finManual >= RETARDO_RELOCK) {
        esperandoRelock = false;
        correccion = 0;
        fijarRumbo();
      }
      return;
    }
    if (!corrigiendo) {
      fijarRumbo();
      pasadaAtras = (velocidad < 0) && (ahora - ultimoParado < 500);   // pasada nueva desde parado
      inicioPasada = ahora;
    }
    moverConCorreccion(velocidad, corrigiendo ? correccion + extraArranque(ahora) : 0);
  } else {
    // Parado, girando en el sitio o con el stick tocado: manejo manual, sin correccion.
    corrigiendo = false;
    correccion = 0;
    esperandoRelock = stickTocado;
    if (stickTocado) finManual = ahora;
    if (abs(velocidad) < 15) ultimoParado = ahora;
    // Stick a la derecha => rueda izquierda (B) mas rapida que la derecha (A) => gira a la derecha.
    moverConGiro(velocidad, giro);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.printf("=== Paso 4: correccion de rumbo (%s) ===\n", MODO_COMPETICION ? "COMPETICION" : "diagnostico");

  pinMode(pinSTBY, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  ledcAttach(pinPWMA, 5000, 8);
  ledcAttach(pinPWMB, 5000, 8);
  detener();
  digitalWrite(pinSTBY, HIGH);

  ledcAttach(pinBuzzer, BUZZER_FREQ_HZ, 8);
  ledcWrite(pinBuzzer, 0);

  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Giroscopio: iniciar y calibrar con el robot quieto.
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(10);   // ms: un bus colgado no debe bloquear el loop
  if (iniciarMPU() && leerID() == 0x68) {
    Serial.printf("MPU6050 encontrado en 0x%02X.\n", mpuAddr);
    mpuListo = true;
    // Hasta 5 intentos: si el robot se movia o no estaba plano, espera y reintenta.
    for (int intento = 1; intento <= 5 && !giroCalibrado; intento++) {
      giroCalibrado = calibrarGiroscopio();
      if (!giroCalibrado && intento < 5) {
        pitidoBloqueante(100);
        delay(1500);
      }
    }
    if (giroCalibrado) {
      pitidoBloqueante(150);                           // 1 pitido: todo bien
    } else {
      Serial.println("No se pudo calibrar: sin correccion de rumbo. Pon el robot plano y quieto y pulsa OPTIONS.");
      pitidoBloqueante(150); delay(150); pitidoBloqueante(150);   // 2 pitidos: fallo
    }
  } else {
    Serial.println("MPU6050 NO encontrado: revisa VCC (3.3V), GND, SDA (GPIO21) y SCL (GPIO22).");
    pitidoBloqueante(150); delay(150); pitidoBloqueante(150);
  }
  ultimaLectura = millis();
  tiempoAnterior = millis();
  primeraLectura = true;

  Serial.printf("Correccion de rumbo: %s (KP=%.2f KD=%.2f KI=%.2f)%s\n",
                (CORREGIR_RUMBO && mpuListo && giroCalibrado) ? "ACTIVADA" : "DESACTIVADA",
                KP, KD, KI, (mpuListo && giroCalibrado) ? "" : "  <- el giroscopio no esta listo");
  Serial.println("Esperando mando... pulsa PS.");
  // Limpieza unica de los datos de R1 (espacio "calib"); no se repite al reiniciar.
  bool limpio = false;
  if (prefs.begin("ruta", true)) { limpio = prefs.getUChar("limpio", 0) == 1; prefs.end(); }
  if (!limpio) {
    calBorrar();
    prefs.begin("ruta", false);
    prefs.putUChar("limpio", 1);
    prefs.end();
    Serial.println("MEMORIA: datos de R1 borrados (una sola vez).");
  }
  calCargar();
  if (!MODO_COMPETICION) calImprimir();   // lo que quedo guardado de la ultima autocalibracion (R1)
  rutaCargar();
  if (!MODO_COMPETICION) rutaImprimir();  // lo que quedo registrado al manejar
  trazaCargar();
  memoriaCargar();
  if (!MODO_COMPETICION) trazaImprimir(); // el arranque de la ultima pasada recta de cada sentido, para ver el zigzag
}

void loop() {
  unsigned long ahora = millis();
  BP32.update();

  if (miMando && miMando->isConnected()) {
    // L1: enciende/apaga la correccion de rumbo (para comparar). Pitido corto = ON, largo = OFF.
    bool l1 = miMando->l1();
    if (l1 && !l1Anterior) {
      rumboActivo = !rumboActivo;
      if (!rumboActivo) { corrigiendo = false; correccion = 0; }
      pitar(rumboActivo ? 100 : 400);
      Serial.printf("Correccion de rumbo: %s\n", rumboActivo ? "ACTIVADA" : "DESACTIVADA");
    }
    l1Anterior = l1;

    if (miMando->hasData()) {
      unsigned long hueco = ahora - ultimoDato;
      if (hueco > huecoMaxMando) huecoMaxMando = hueco;
      ultimoDato = ahora;
      if (cortePorFaltaDeDatos) {
        cortePorFaltaDeDatos = false;
        Serial.println("Volvieron los datos del mando.");
      }
    }
    if (ahora - ultimoDato > TIEMPO_MAX_SIN_DATOS) {
      if (!cortePorFaltaDeDatos) {
        cortePorFaltaDeDatos = true;
        cortesMando++;
        Serial.printf("Sin datos del mando por mas de %lu ms: motores parados.\n", TIEMPO_MAX_SIN_DATOS);
      }
      detener();
    } else if (!MODO_COMPETICION && miMando->miscSelect() && motoresParados && mpuListo) {
      autodiagnostico();
    } else if (!MODO_COMPETICION && miMando->r1() && motoresParados && mpuListo && giroCalibrado) {
      autocalibrarMotores();
    } else if (miMando->miscStart() && motoresParados && mpuListo) {
      recalibrar();
    } else {
      conducir();
    }
  } else {
    detener();
  }

  actualizarMPU(ahora);

  // Registro de manejo: se guarda en la flash solo con el carro parado y quieto desde hace un momento.
  if ((rutaSucia || trazaSucia || marchaSinGuardar) && motoresParados && ahora - ultimaMarcha >= 500 && ahora - ultimoGuardadoRuta >= 5000) {
    if (rutaSucia) rutaGuardar();
    memoriaGuardarSiCambio();
    marchaSinGuardar = false;
    if (trazaSucia) trazaGuardar();
    ultimoGuardadoRuta = ahora;
  }
  // Cruceta abajo 2 s (con el carro parado): borra el registro de manejo.
  {
    static unsigned long abajoDesde = 0;
    bool abajo = !MODO_COMPETICION && miMando && miMando->isConnected() && (miMando->dpad() & 0x02) && motoresParados;
    if (!abajo) abajoDesde = 0;
    else if (abajoDesde == 0) abajoDesde = ahora;
    else if (ahora - abajoDesde >= 2000) {
      rutaBorrar();
      abajoDesde = 0;
      Serial.println("RUTA: registro de manejo borrado.");
      pitidoBloqueante(120); delay(150); pitidoBloqueante(120);
      while (miMando && miMando->isConnected() && (miMando->dpad() & 0x02)) { BP32.update(); vTaskDelay(1); }
    }
  }

  // Registro del Bluetooth cada 2 s: cortes acumulados y hueco mas largo sin datos en esa ventana.
  if (DEBUG_RUMBO && miMando) {
    static unsigned long ultimoBT = 0;
    if (ahora - ultimoBT >= 2000) {
      ultimoBT = ahora;
      Serial.printf("BT cortes=%lu hueco_max=%lums rumbo=%s\n", cortesMando, huecoMaxMando, rumboActivo ? "ON" : "OFF");
      huecoMaxMando = 0;
    }
  }

  actualizarBuzzer(millis());

  vTaskDelay(1);   // cede tiempo al Bluetooth; sin esto el mando llega con retraso
}
