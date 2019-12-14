#include <Timer.h>
#include <Event.h>

#include <RF24.h>
#include <printf.h>
#include <nRF24L01.h>
#include <RF24_config.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define BLINK_LED_ACTIVITY (1000) // Frecuencia para el led "testingo"

#define SERIAL_DISPLAY_FRECUENCY (1000) // Frecuencia para mostrar informacion de orientacion

#define BNO055_SAMPLERATE_DELAY_MS (1000) // Espera entre muestreo de BNO055
#define SOLAR_PANEL_SAMPLE_RELE_TIMEOUT (1000) // Espera para estabilizar salida de Rele
#define SOLAR_PANEL_SAMPLE_TIMEOUT (5000) /// Espera entre lecturas de los parametros del panel

#define RELE1_PIN (2)
#define RELE2_PIN (3)
#define BOTON1_PIN (4)
#define BOTON2_PIN (5)
#define LED1_PIN (6)
#define LED2_PIN (7)

#define ANALOG_VOLTAGE_PANEL_PIN (0)
#define ANALOG_VOLTAGE_BATERIA_PIN (1)

// Crear objeto RF24
RF24 radio(9, 8);  // CE, CSN

//address through which two modules communicate.
const byte address[6] = "00020";


// Verificar la direccion del puerto del dispositivo (por defecto es 0x29 o 0x28)
//                                   
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // (id, address)
sensors_event_t lastEventLinearAccel;
sensors_event_t lastEvent;

Timer timer;


void setup(void)
{
  Serial.begin(115200);
  Serial.println("");

  // Setup pins
  pinMode(RELE1_PIN, OUTPUT);
  pinMode(RELE2_PIN, OUTPUT);
  pinMode(BOTON1_PIN, INPUT);
  pinMode(BOTON2_PIN, INPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  analogReference(DEFAULT);
  pinMode(ANALOG_VOLTAGE_PANEL_PIN, INPUT);
  pinMode(ANALOG_VOLTAGE_BATERIA_PIN, INPUT);

  digitalWrite(RELE1_PIN, HIGH);
  digitalWrite(RELE2_PIN, HIGH);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
   
  setupBNO055();
  delay(1000);
   
  /* Display some basic information on this sensor */
  displayBNO055Detalles();
  
  delay(1000);

  // timer.oscillate(LED1_PIN, 500, HIGH);
  // timer.oscillate(LED2_PIN, 500, HIGH);
  timer.every(BNO055_SAMPLERATE_DELAY_MS, obtenerInformacionDeBNO055, (void*)0);
  timer.every(SOLAR_PANEL_SAMPLE_TIMEOUT, eventoMuestraDePanelSolar, (void*)0);
  // timer.every(SERIAL_DISPLAY_FRECUENCY, serialInfoAll, (void*)0);
  timer.every(1000, blinkit, (void*)0);

  // Configuracion RF
  radio.begin();
   
  // Direccion de radio
  radio.openWritingPipe(address);
  
  // Solo transmitir
  radio.stopListening();

  
}


void setupBNO055()
{
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  bno.setExtCrystalUse(true);
}

int stopped = 0;
void control()
{
  int botton = digitalRead(BOTON1_PIN);
  if (botton) stopped = !stopped;
  botton = digitalRead(BOTON2_PIN);
  if (botton) stopped = !stopped;
}

int ledState = LOW;
void blinkit(){
  if (ledState==LOW) ledState = HIGH;
  else ledState = LOW;
  
  digitalWrite(LED1_PIN, ledState);
}

void displayBNO055Detalles()
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


void loop(void)
{
  if (!stopped) timer.update();
}


int timerRele;

void eventoMuestraDePanelSolar(void *context)
{
  digitalWrite(RELE1_PIN, LOW);
  digitalWrite(RELE2_PIN, LOW);
  timerRele = timer.after(SOLAR_PANEL_SAMPLE_RELE_TIMEOUT, lecturaDePanelSolar, (void*)0);
}

void lecturaDePanelSolar(void *context){
  Serial.println("Lectura del panel Solar, ");
  int rawVoltajePanel = analogRead(ANALOG_VOLTAGE_PANEL_PIN);
  int rawVoltajeBateria = analogRead(ANALOG_VOLTAGE_BATERIA_PIN);
  float voltajePanel = calcularVoltaje(rawVoltajePanel);
  float voltajeBateria = calcularVoltaje(rawVoltajeBateria);
  Serial.print("panel-raw: (");Serial.print(rawVoltajePanel);Serial.print("),");
  Serial.print("bateria-raw: (");Serial.print(rawVoltajeBateria);Serial.print(") ; ");
  Serial.print("voltaje-Panel: ");Serial.print(voltajePanel);Serial.print("V ");
  Serial.print("voltaje-Bateria: ");Serial.print(voltajeBateria);Serial.print("V \n");
  digitalWrite(RELE1_PIN, HIGH);
  digitalWrite(RELE2_PIN, HIGH);
  timer.stop(timerRele);
  //Enviar mensaje a receptor
  String textToSend = "RVP:";
  textToSend.concat(rawVoltajePanel);
  textToSend.concat(";RVB:");
  textToSend.concat(rawVoltajeBateria);
  textToSend.concat(";VP:");
  textToSend.concat(voltajePanel);
  textToSend.concat(";VB:");
  textToSend.concat(voltajeBateria);
  
  char * buffer = (char *)textToSend.c_str();
  Serial.print(buffer);
  radio.write(buffer, textToSend.length()+1);
}

float calcularVoltaje(int rawVoltaje)
{
  float voltaje = (float)rawVoltaje;
  return 5.0*(voltaje*5.0)/1024.0;
}


void obtenerInformacionDeBNO055(void *context){
  
  bno.getEvent(&lastEventLinearAccel, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Para la orientacion
  sensors_event_t event; 
  bno.getEvent(&lastEvent);

  //Enviar mensaje a receptor
  String textToSend = "OX:";
  textToSend.concat(lastEvent.orientation.x);
  textToSend.concat(";OY:");
  textToSend.concat(lastEvent.orientation.y);
  textToSend.concat(";OZ:");
  textToSend.concat(lastEvent.orientation.z);
  
  char * buffer = (char *)textToSend.c_str();
  radio.write(buffer, textToSend.length()+1);
}


void serialInfoAll()
{
  serialInfoAllBNO055();
}

void serialInfoAllBNO055()
{
  serialInfoGiroscopio(); // En cuaternions
  serialInfoAcelerometro();
  serialInfoCalibrationData();
  serialInfoOrientacion();
}

void serialInfoOrientacion()
{
  Serial.print("X: ");
  Serial.print(lastEvent.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(lastEvent.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(lastEvent.orientation.z, 4);
  Serial.println("");
}

void serialInfoGiroscopio()
{
  // Frecuencia de muestreo
  Serial.print(F("Giroscopio: "));
  Serial.print((float)lastEventLinearAccel.acceleration.x);
  Serial.print(F(" "));
  Serial.print((float)lastEventLinearAccel.acceleration.y);
  Serial.print(F(" "));
  Serial.print((float)lastEventLinearAccel.acceleration.z);
  Serial.println(F(""));
}

void serialInfoAcelerometro()
{
  Serial.print(F("Acceleration: "));
  Serial.print((float)lastEventLinearAccel.acceleration.x);
  Serial.print(F(" "));
  Serial.print((float)lastEventLinearAccel.acceleration.y);
  Serial.print(F(" "));
  Serial.print((float)lastEventLinearAccel.acceleration.z);
  Serial.println(F(""));
}

void serialInfoCalibrationData()
{
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(F("Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);

}



