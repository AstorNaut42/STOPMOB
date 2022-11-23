#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "UbidotsEsp32Mqtt.h"
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <EEPROM.h>

// Definerer tilstander --------------------------------------------------------
#define EEPROM_SIZE 1 // Ønsker å akksessere 1 byte av EEPROM

#define AWAKE 0
#define DEEPSLEEP 1
#define SEMISLEEP 2

// -----------------------------------------------------------------------------

// I2C instansering ------------------------------------------------------------
// ICM gyrosensor

#define ICM_SDA 21
#define ICM_SCL 22

// BME280 temperatur, luftfuktighet og lufttrykk sensor
#define BME_SDA 33
#define BME_SCL 32

TwoWire I2C_ICM = TwoWire(0);
TwoWire I2C_BME = TwoWire(1);

Adafruit_BME280 bme;
Adafruit_ICM20948 icm;
Adafruit_Sensor *accel;
//--------------------------------------------------------------------------------

// definerer variabler---------------------------------------------------------------------------------------------------------

int mobStatus; // kan endres i: callback(), buttonPress(). Brukt i: callback(), buttonPress(), ledControl(), motorControl()
int awake_state = AWAKE;
int prev_awake_state = awake_state;
const int buttonPin = 25;
bool buttonState = false;

float x;
float y;
float z;

bool motion = false;
int motion_counter = 0;
bool allsafe = true;
bool ubidots_init = false;

int count = 0;
float tempTot = 0;
const char *LikelySurvivalTime = "LikelySurvivalTimeMinM";
const char *TEMPERATURE = "TEMPERATUREM";
int LEDcount = 0;
const int WarningOFF = 18;
const int WarningON = 23;
bool blinkState = false;
const int WarningState = 19;
bool healthCheck;
const int photoPin = 34;

// definerer ubidots variabler. Alle brukes i ubidotsInitialization()
const char *UBIDOTS_TOKEN = "BBFF-H2Wsoo1L7XxRmN16prd6DuPAreozkW";
const char *WIFI_SSID = "Daniel's Galaxy S10";
const char *WIFI_PASS = "Passord1337";
const char *DEVICE_LABEL = "stopmob"; // brukes i: ubidotsInitialization(), publishToUbidots()
const char *VARIABLE_LABEL = "MOB";   // brukes i: ubidotsInitialization(), publishToUbidots()
const char *ID = "MaggyJuicy";

//----------------------------------------------------------------------------------------------------------------------------

// instansiering av objekter------
Ubidots ubidots(UBIDOTS_TOKEN);
RTC_DATA_ATTR int bootCount = 0;
//--------------------------------

// definerer timers-------------------------------------------------

unsigned long buttonTimer = 0; // timer for å forhindre knappeprell. Brukes i: buttonPress(). Endres i: buttonPress()
unsigned long blinkTimer = 0;  // timer for blinking av led. Brukes i: ledControl(). Endres i: ledControl()
unsigned long awake_timer = 0;
unsigned long semi_timer = 0;
unsigned long motion_timer = 0;

unsigned long timer = 0;
unsigned long timer2;
unsigned long timer3;
unsigned long timer4;
unsigned long timer5 = 0;

//-----------------------------------------------------------------

// funksjoner knyttet til ubidots ----------------------------------------------------

void crewState(char state)
{
  switch (state)
  {
  case '0':
    digitalWrite(WarningON, LOW);
    digitalWrite(WarningOFF, HIGH);
    blinkState = false;
    allsafe = true;
    count = 0;
    tempTot = 0;
    LEDcount = 0;
    break;

  case '1':
    digitalWrite(WarningOFF, LOW);
    blinkState = true;
    timer3 = millis();
    healthCheck = false;
    break;

  case '2':
    digitalWrite(WarningON, HIGH);
    digitalWrite(WarningOFF, LOW);
    blinkState = false;
    break;
  }
}

// funksjon som oppdaterer mobStatus hvis det skjer en endring i mobStatus via ubidots
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  crewState(payload[0]);
  Serial.println();
}

// initialiserer ubidots. Bruker: callback()
bool ubidotsInitialization()
{
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL);

  return ubidots_init = true;
}

// funksjon som publiserer til mobStatus til ubidots
void publishToUbidots()
{
  if (!ubidots.connected())
  {
    ubidots.connect();
  }
  if (ubidots.connected())
  {
    int value = mobStatus;
    ubidots.add(VARIABLE_LABEL, value);
    ubidots.publish(DEVICE_LABEL);
  }
  else
  {
    ubidots.disconnect();
  }
}

void ubidotsLoop()
{
  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }
  ubidots.loop();
}

//-----------------------------------------------------------------------------------

// Funksjoner knyttet til dvalemodus-------------------------------------------------

int awakeFunction(int awake_time, bool motion_detected)
{ // sekunder

  if (millis() - awake_timer >= awake_time * 1000 && motion_detected == false)
  {
    Serial.println("Legger meg.");
    awake_timer = millis();
    awake_state = SEMISLEEP;
  }
  else
    (awake_state = AWAKE);

  return awake_state;
}

void semisleep(int awake_time)
{
  if (millis() - semi_timer >= awake_time * 1000)
  {
    semi_timer = millis();
    awake_state = DEEPSLEEP;
    Serial.println("Sovner.");
  }
}

void deepsleep(int wake_time)
{
  // Aktiverer vekking etter wake_time sekunder
  esp_sleep_enable_timer_wakeup(wake_time * 1000000);
  // esp_sleep_enable_ext0_wakeup(GPIO_NUM_32, 1); // 1 = High, 0 = Low
  //   Setter enheten i dyp søvn
  EEPROM.write(0, 2); // skriver tilstand SEMISLEEP til EEPROM
  EEPROM.commit();    // lagrer endringen
  esp_deep_sleep_start();
}

void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wake_up_source;

  wake_up_source = esp_sleep_get_wakeup_cause();

  switch (wake_up_source)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wake-up from external signal with RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wake-up from external signal with RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wake up caused by a timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wake up caused by a touchpad");
    break;
  default:
    Serial.printf("Wake up not caused by Deep Sleep: %d\n", wake_up_source);
    break;
  }
}

void deepsleepInitialize()
{
  ++bootCount;
  Serial.println("----------------------");
  Serial.println(String(bootCount) + "th Boot ");

  // Displays the wake-up source
  print_wakeup_reason();
}

// ----------------------------------------------------------------------------------

// funksjon som intisialiserer pins

void I2C_initialization()
{

  I2C_ICM.begin(ICM_SDA, ICM_SCL, 100000);
  I2C_BME.begin(BME_SDA, BME_SCL, 100000);

  bool status1 = icm.begin_I2C(0x69, &I2C_ICM);
  if (!status1)
  {
    Serial.println("Failed to find ICM20X chip");
    while (1)
      ;
  }

  icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_5_7_HZ);

  // Get an Adafruit_Sensor compatible object for the ICM20X's accelerometer
  accel = icm.getAccelerometerSensor();

  bool status2 = bme.begin(0x76, &I2C_BME);
  if (!status2)
  {
    Serial.println("Could not find a valid BME280_2 sensor, check wiring!");
    while (1)
      ;
  }
}

void pinInitialization()
{
  // button pin
  pinMode(buttonPin, INPUT);

  // led pins
  pinMode(WarningOFF, OUTPUT);
  pinMode(WarningON, OUTPUT);
  pinMode(WarningState, INPUT);
  pinMode(photoPin, INPUT);
  /*
      //potentiometer pin
      pinMode(potPin, INPUT);

      //motor pins
      pinMode(enA, OUTPUT);
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);*/
}

// definerer andre funksjoner---------------------------------------------------------------------

// funksjon som oppdaterer mob status i henold til knappetrykk. Bruker: pubishToUbidots()
bool buttonPress()
{
  if (digitalRead(buttonPin))
  {
    while (digitalRead(buttonPin))
    {
      delay(10);
    }
    buttonState = !buttonState;
  }
  return buttonState;
}

/*
//funksjon som styrer led lysene.
void ledControl(){
  if (mobStatus == 0){
    digitalWrite(redLedPin, LOW);
    digitalWrite(greenLedPin, HIGH);
  }
  else if (mobStatus == 1){
    digitalWrite(greenLedPin, LOW);

    if ((millis() - blinkTimer) >= 500){
      switch (digitalRead(redLedPin)){
        case 1:
        digitalWrite(redLedPin, LOW);
        break;

        case 0:
        digitalWrite(redLedPin, HIGH);
        break;
      }
      blinkTimer = millis();
    }
  }
  else{
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
  }
}

*/

/*

//funksjon som styrer motorens hastighet
void motorControlHelp(){
  int potReading = analogRead(potPin);

  if (potReading >= 2048){
    potReading = map(potReading,2048,4095,0,255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(enA,potReading);
  }
  else{
    potReading = map(potReading,0,2047,0,255);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(enA,potReading);
  }
}


//funksjon som sjekker hva mob status er, og styrer motoren deretter. Bruker: motorControlHelp()
void motorControl(){
  switch(mobStatus){
    case 0:
      motorControlHelp();
    break;

    case 1:
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(enA,LOW);
    break;

    case 2:
      motorControlHelp();
      break;
  }
}*/

// Funksjon som sjekker om enheten har beveget seg hvert sekund, returnerer true hvis den har beveget seg, false hvis den ikke har beveget seg
bool checkMotionSensor(int interval_time_ms)
{
  if (motion_counter >= 5)
  {
    motion_counter = 0;
  }

  float prev_x = x;
  float prev_y = y;
  float prev_z = z;

  // Get a new normalized sensor event
  sensors_event_t a;
  // fill the event with the most recent data
  accel->getEvent(&a);

  if (millis() - motion_timer > interval_time_ms)
  {

    x = a.acceleration.x;
    y = a.acceleration.y;
    z = a.acceleration.z;

    motion_timer = millis();
    if (((prev_x + 0.1 > x) && (x > prev_x - 0.1)) && ((prev_y + 0.1 > y) && (y > prev_y - 0.1)) && ((prev_z + 0.1 > z) && (z > prev_z - 0.1)))
    {
      motion = false;
    }
    else
    {
      motion = true;
      motion_counter++;
    }
  }

  return motion;
}

void blinkCrew()
{
  if (millis() - timer3 >= 500)
  {
    digitalWrite(WarningON, !digitalRead(WarningState));
    timer3 = millis();
  }
}

int howLong()
{
  float temp = bme.readTemperature();
  float LikelySurvivalTime_mins;

  if (healthCheck == false)
  {
    timer2 = millis();
  }

  if (temp <= 0.3)
  {
    LikelySurvivalTime_mins = 15;
  }
  else if (0.3 < temp and temp <= 10)
  {
    LikelySurvivalTime_mins = 120;
  }
  else if (10 < temp and temp <= 15.5)
  {
    LikelySurvivalTime_mins = 240;
  }
  else if (15.5 < temp and temp <= 21)
  {
    LikelySurvivalTime_mins = 240;
  }
  else if (21 < temp)
  {
    LikelySurvivalTime_mins = 1020;
  }
  healthCheck = true;
  return (LikelySurvivalTime_mins - (((millis() - timer2) / 1000) / 60));
}

// Funksjon som kontrollerer hva som skjer dersom enheten havner i vannet, samt håndterer kommunikasjon med ubidots
void keepSafe()
{

  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }
  // Måling og Varsling
  if (allsafe == false)
  {
    if (count == 0 and millis() - timer5 >= 1000)
    {
      ubidots.add(ID, 2);
      timer5 = millis();
      ubidots.publish(DEVICE_LABEL);
    }
    if ((millis() - timer4 >= 30000) and count < 10)
    {
      tempTot += (float)bme.readTemperature();
      timer4 = millis();
      count += 1;
      ubidots.add(TEMPERATURE, (float)bme.readTemperature());
      ubidots.publish(DEVICE_LABEL);
    }
    if (count == 10)
    {
      ubidots.add(TEMPERATURE, tempTot / (float)10);
      count += 1;
    }
    if (millis() - timer >= 60000)
    {
      ubidots.add(LikelySurvivalTime, howLong());
      timer = millis();
      ubidots.publish(DEVICE_LABEL);
    }
  }
  else if (millis() - timer >= 5000 && LEDcount < 50)
  {
    if (LEDcount > 10)
    {
      ubidots.add(ID, 1);
    }
    else
    {
      ubidots.add(ID, 0);
    }
    timer = millis();
    ubidots.publish(DEVICE_LABEL);
  }

  if (digitalRead(WarningState) and LEDcount <= 10)
  {
    LEDcount += 1;
  }

  ubidots.loop();
  /*
    // Erstatning for enkel simulering istedenfor trykk
    if (analogRead(photoPin) <= 3000 and allsafe != false)
    {
      allsafe = false;
      timer4 = millis();
    }
  */

  // BME unsafe trigger
  float pressure = (float)bme.readPressure() / (float)1000;
  if (pressure >= 102)
  {
    allsafe = false;
    timer4 = millis();
  }

  // Serial.println(pressure);

  if (blinkState)
  {
    blinkCrew();
  }
}

// Hovedfunksjon som inneholder tilstandene til enheten og bestemmer hva som skal skje i hvert tilstand
void mainFunction()
{

  if (awake_state >= 3) // Resetter tilstandsvariabel dersom variabelen har blitt for stor
  {
    awake_state = 0;
  }

  // Tilstandsmaskin
  switch (awake_state)
  {
  case AWAKE:
  {
    if (!ubidots_init) // Initialiserer ubidots dersom det ikke er gjort
    {
      ubidotsInitialization();
    }
    // Oppdaterer tilstandsvariabelen basert på hva som returneres fra awakeFunction.
    // awakeFunction tar inn to parametere, hvor den første er hvor mange sekunder enheten skal være i tilstanden, og den andre er hva som returneres fra checkMotionSensor
    awake_state = awakeFunction(30, checkMotionSensor(2000));
    keepSafe(); // Kjører keepSafe funksjonen for å håndtere kommunikasjon med ubidots
    break;
  }

  case DEEPSLEEP:
  { // Case hvor enheten er i dyp søvn, MEN vekkes med jevne mellomrom for å sjekke om bevegelsessensoren har registrert bevegelse

    if (allsafe == false) // Sjekker om noen enheter er i fare, vil da tvinge denne enheten fra å gå i søvnmodus
    {
      awake_state = AWAKE;
      break;
    }
    ubidots.add(ID, 3);            // Sier ifra til ubidots at enheten er i søvnmodus
    ubidots.publish(DEVICE_LABEL); // Publiserer data til ubidots
    deepsleep(10);                 // Kjører deepsleep funksjonen, vekkes etter 10 sekunder
    break;
  }

  case SEMISLEEP:
  {                       // Tilstand hvor bevegselsessensoren kjøres en liten periode for å sjekke om personen beveger seg, hvis ikke går den til dyp søvn
    if (allsafe == false) // Sjekker om noen enheter er i fare, vil da tvinge denne enheten fra å gå i søvnmodus
    {
      awake_state = AWAKE;
      break;
    }

    // Sjekker om det er registrert bevegelse og om tilstanden har vært den samme i 2 iterasjoner
    if (checkMotionSensor(500) && prev_awake_state == awake_state)
    {
      if (motion_counter >= 3) // Unngår å gå i semisleep under oppstart
      {
        EEPROM.write(0, AWAKE); // skriver til EEPROM at enheten skal være i awake state neste gang den starter
        EEPROM.commit();        // commiter endringen til EEPROM
        awake_state = AWAKE;
        break;
      }
    }
    semisleep(15); // Kjører semisleep funksjonen, går tilbake i deepsleep etter 15 sekunder med mindre enheten vekkes
    break;
  }
  }
}

//-----------------------------------------------------------------------------------------------

// Setup funksjonen som kjøres en gang ved oppstart og initialiserer alt som trengs
void setup()
{
  EEPROM.begin(EEPROM_SIZE); // Start EEPROM for å lagre og lese data

  awake_state = EEPROM.read(0); // Leser fra EEPROM adresse 0 for å finne ut hvilken tilstand enheten var i sist den ble slått av

  Serial.begin(115200);                                            // Starter seriekommunikasjon
  Serial.println("Nåværende tilstand er: " + String(awake_state)); // Printer nåværende tilstand under oppstart

  deepsleepInitialize(); // Initialiserer deepsleep

  I2C_initialization(); // Initialiserer begge I2C bussene, og sjekker om de er tilkoblet. Gjeder BME og ICM

  pinInitialization(); // initialiserer pinner

  if (EEPROM.read(0) == AWAKE) // Sjekker om enheten var i AWAKE tilstand sist den ble slått av, starter i så fall ubidots
  {
    ubidotsInitialization(); // initiaiserer ubidots
  }
}

// Funksjon som printer tilstanden enheten er i, dersom den bytter tilstand
void print_state()
{
  if (awake_state != prev_awake_state)
  {
    Serial.print("Nåværende tilstand: ");
    Serial.println(awake_state);
    prev_awake_state = awake_state;
  }
}

void loop()
{
  print_state();

  if (motion)
  {
    awake_timer = millis();
    semi_timer = millis();
  }
  mainFunction(); // kjører main funksjonen
}