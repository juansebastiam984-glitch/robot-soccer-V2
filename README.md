# Robot de fútbol RC V2 (ESP32 + DualShock 4)

Carro de fútbol radiocontrolado de ~10x10 cm (categoría RoboFut), manejado con un mando **DualShock 4** por Bluetooth.
Usa un **giroscopio MPU6050** para ir recto aunque los dos motores no sean iguales: mide hacia dónde apunta el carro
y corrige las ruedas en tiempo real.

Versión anterior: [robot-soccer-V1](https://github.com/juansebastiam984-glitch/robot-soccer-V1).

## Hardware

| Pieza | Conexión |
|---|---|
| ESP32 clásico (ESP32-D0WD-V3, USB CP2102) | — |
| Driver de motores TB6612FNG | STBY GPIO 13 |
| Motor A (rueda **derecha**) | AIN1 27, AIN2 26, PWMA 14 |
| Motor B (rueda **izquierda**) | BIN1 25, BIN2 33, PWMB 32 |
| MPU6050 (I2C, 0x68) | SDA 21, SCL 22 |
| Buzzer pasivo (con resistencia 100-220 Ω) | GPIO 18 |
| 2 motores TT, batería de 2 celdas en serie | — |

## Controles

| Botón | Acción |
|---|---|
| R2 / L2 | Adelante / atrás (proporcional) |
| Stick izquierdo (X) | Girar |
| L1 | Apagar / encender la corrección de rumbo (pitido corto = ON, largo = OFF) |
| OPTIONS | Recalibrar el giroscopio (carro plano y quieto) |

Al encender, deja el carro **quieto y plano 2 s**: se calibra el giroscopio (1 pitido corto = bien, 2 = falló).
Al conectar el mando suena 2 s; sin mando, pita 1 s sí / 1 s no.

## Cómo funciona la corrección de rumbo

- Al avanzar o retroceder con el stick centrado, el carro guarda el rumbo y un **control PID** (KP 2.0, KI 1.0, KD 0.15)
  ajusta las ruedas para mantenerlo. Al tocar el stick manejas tú; al soltarlo, fija el rumbo nuevo.
- Una **compensación fija** frena la rueda más fuerte (aquí la B, un 15 %); el giroscopio corrige el resto.
- La parte integral **solo aprende con errores pequeños** (< 5°) pero siempre puede descargarse, para no pasarse
  del rumbo tras un golpe.
- La **corrección aprendida se guarda en la flash** (NVS) al parar: al encender, el carro arranca ya compensado.
- Al arrancar hacia atrás se suma un empuje extra que se desvanece en 0.8 s, porque una rueda acelera más lento.

Resultado medido: unos 3-5 cm de desvío en 3 m.

### Modo diagnóstico

En `main/sketch.cpp`, `MODO_COMPETICION = true` es la versión para jugar. Con `false` se activan:

- Un **registro de manejo en la flash** (por sentido y velocidad: PWM de cada rueda, desvío, corrección) y
  **trazas** del arranque de la última pasada recta de cada sentido, cada 20 ms. Se graban sin cable y se imprimen
  por el puerto serie al volver a encender (115200 baudios). Cruceta abajo 2 s las borra.
- Botones de prueba: Cuadrado / Círculo / Triángulo / X mueven motor A / B / ambos adelante / ambos atrás;
  SHARE hace un autodiagnóstico del giroscopio con el carro en el aire; R1 una autocalibración por pivote.

## Compilar y subir

Proyecto ESP-IDF 5.5 con Arduino como componente y [Bluepad32](https://github.com/ricardoquesada/bluepad32),
compilado con PlatformIO. El código del robot está en `main/sketch.cpp`.

1. Descarga este repositorio (botón **Code → Download ZIP**, o `git clone`).
2. Descarga las dos dependencias grandes (Arduino-ESP32 y BTstack) en las versiones exactas que usa el robot:
   ```bash
   cd robot-soccer-V2
   bash instalar_dependencias.sh
   ```
3. Compila y sube:
   ```bash
   pio run -e esp32dev
   esptool --chip esp32 --port /dev/ttyUSB0 --baud 115200 write-flash 0x10000 .pio/build/esp32dev/firmware.bin
   ```

Notas:
- Con Python 3.14 PlatformIO falla; funciona con `uvx --python 3.13 --from platformio==6.1.18 platformio run -e esp32dev`.
- Sube a 115200 baudios (a 460800 falla). El primer intento de `esptool` a veces no conecta: reinténtalo.
- Las dependencias de `managed_components/` se descargan solas al compilar.

## Créditos y licencia

Basado en la plantilla [esp-idf-arduino-bluepad32-template](https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template)
de Ricardo Quesada (su README original está en `README-plantilla-bluepad32.md`). Incluye
[Arduino-ESP32](https://github.com/espressif/arduino-esp32) y [BTstack](https://github.com/bluekitchen/btstack), que se
descargan con `instalar_dependencias.sh`. Licencias (ver `LICENSE`): el código del robot y de la plantilla, Apache 2.0;
`bluepad32_arduino`, Apache 2.0 y LGPL-2.1; BTstack tiene su propia licencia (libre para uso no comercial).
