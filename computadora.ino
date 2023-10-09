#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <SFE_BMP180.h>
#include <MPU6050.h>

// ARCHIVOS DE RECUPERACION DE DATOS
File datos;

// Sensor de presion barometrico
SFE_BMP180 bmp;
// Presion de referencia, Altura inicial y maxima
double presionBase, altitudBase, altitudMaxima; 

// Modulo MPU6050 Acelerometro y giroscopio
MPU6050 mpu;

// Valores RAW sin procesar del acelerometro en los ejes x,y,z
int ax, ay, az;

// Valores RAW sin procesae del girscopio en los ejes x,y,z
int gx, gy, gz;

// Altura despliegue paracaidas
double despliegue;

// Tiempo de despliegue
long tDespliegue;

int numdespliegue;

// Funcion para activar el buzzer en senal de exito
void buzzer(int pin) {
  digitalWrite(pin, HIGH);
  delay(500);
  digitalWrite(pin, LOW);  
  delay(500);
  digitalWrite(pin, HIGH);
  delay(500);
  digitalWrite(pin, LOW);  
  delay(500);
}

void buzzerf(int pin) {
  digitalWrite(pin, HIGH);
  delay(500);
  digitalWrite(pin, LOW);  
}

void checaComponentes() {
  if (!SD.begin(9) || !bmp.begin() || !mpu.testConnection()) {
    digitalWrite(8, HIGH);
    delay(200);
    digitalWrite(8, LOW);  
    delay(200);
  }
}

// FUNCION PARA PREPARAR LA SD
void preparaSD() {
  // Crea o abre archivo, archivo | null
  datos = SD.open("datos.csv", FILE_WRITE);

  // Columnas de prueba.csv
  if (datos) {
    datos.print("t,h,p,ax,ay,az");
    datos.println(",gX,gY,gZ,dp,tp");
    datos.close();
  }
}

// FUNCION PARA PREPARAR EL SENSOR DE PRESION BAROMETRICO BMP180
void preparaBMP() {
  // Obtenemos la presion base
  presionBase = obtienePresion();

  // obtenemos la altitud base
  altitudBase = 0;

  // Actualizamos la altitud maxima
  altitudMaxima = 0;
}

// Funcion para obetener la presion
double obtienePresion() {
  char status;
  double T, P;

  // Empieza la temperatura
  status = bmp.startTemperature();
  if (status != 0) {

    delay(status);

    // obtiene la temperatura
    status = bmp.getTemperature(T);
    if (status != 0) {

      // Empieza el calculo de la presion en alto rendimiento
      status = bmp.startPressure(3);
      if (status != 0) {
        delay(status);
        // Obtiene la presion
        status = bmp.getPressure(P, T);
        if (status != 0) {
          // retorna la presion
          return (P);
        }
      }
    }
  }
}

// Funcion para preparar el modulo MPU6050 con acelerometro y giroscopio.
void preparaMPU() {
  mpu.initialize();
}

// Funcion para preparar el MOSFET
void preparaMOSFET(int pin) {
  pinMode(pin, OUTPUT);
}

void setup() {
  while (!Serial) {}

  Serial.begin(9600);

  // Buzzer
  pinMode(8, OUTPUT);

  Wire.begin();  // Inicializamos el puerto I2C

  checaComponentes();

  // [-----BMP180-----]
  preparaBMP();

  // [-----SD-----]
  preparaSD();

  // [-----MPU6050-----]
  preparaMPU();

  // [-----MOSFET-----]
  preparaMOSFET(6);
  
  despliegue = 0;
  tDespliegue = 0;  
  numdespliegue = 0;  
  // Todos los sensores listos
  buzzer(8);
}

void loop() {

  // Presion actual
  double presion = obtienePresion();
  
  double altitud = bmp.altitude(presion, presionBase);

  // Aceleracion MPU6050
  mpu.getAcceleration(&ax, &ay, &az);

  // Rotacion MPU6050
  mpu.getRotation(&gx, &gy, &gz);
  
  // Sistema de recuperacion 
  //if (altitud > altitudBase + 25 && altitud < altitudMaxima - 25) {  // Rango entre 25 metros
  if (numdespliegue == 0 && altitud > altitudBase + 25 && altitud < altitudMaxima - 25) {  
    despliegue = altitud;
    tDespliegue = millis();
    numdespliegue = 1;
    digitalWrite(6, HIGH);
    delay(1000);
    digitalWrite(6, LOW);
    buzzerf(8);
  } else if (altitud > altitudMaxima) {
    altitudMaxima = altitud;
  }


  // Archivo de datos
  datos = SD.open("datos.csv", FILE_WRITE);

  // Escribimos en la tarjeta sd
  if (datos) {
    // tiempo,altitud,presion,ax,ay,az,gx,gy,gz,dp
    datos.print(millis());  // Tiempo
    datos.print(",");
    datos.print(altitud);  // altitud
    datos.print(",");
    datos.print(presion);  // presion
    datos.print(",");
    datos.print(ax); 
    datos.print(",");
    datos.print(ay); 
    datos.print(",");
    datos.print(az);
    datos.print(",");
    datos.print(gx); 
    datos.print(",");
    datos.print(gy);
    datos.print(",");
    datos.print(gz);
    datos.print(",");
    datos.print(despliegue);
    datos.print(",");
    datos.println(tDespliegue);
    datos.flush();
  }
  
  datos.close();
  
  Serial.println(altitud);
  //display_freeram();
}

void display_freeram() {
  Serial.print(F("- SRAM left: "));
  Serial.println(freeRam());
}

int freeRam() {
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0  
    ? (int)&__heap_start : (int) __brkval);  
}
