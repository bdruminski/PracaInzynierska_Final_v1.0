#include <Wire.h>
#include "Adafruit_SGP30.h"
#include "Adafruit_BME280.h"
#include "SdsDustSensor.h"
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "ThingSpeak.h"

#define PrzelicznikNaSekundy 1000000ULL //Konwersja z mikrosekund na sekundy
#define CzasUsypiania 1200                //Czas po jakim uklad ESP32 sie usypia
#define I2C_SDA 21                      //Linia i2c
#define I2C_SCL 22                      //Linia i2c
#define BME280_Adres 0x76               //Adres i2c sensora BME280
#define Wysokosc 100.0                  //Wysokosc n.p.m Warszawy
#define Tranzystor_SGP30 4              //Pin sterujacy tranzystorem

SdsDustSensor sds(Serial2);
Adafruit_SGP30 sgp;
WiFiClient client;
Adafruit_BME280 bme;


float PM25 = 0;
float PM10 = 0;
float temperatura = 0;
float wilgotnosc = 0;
float cisnienie = 0;
float eCO2;
float tVOC;

//Konfiguracja zmiennych dla chmury ThingSpeak

String data;
String NazwaServera = "https://api.thingspeak.com/update?api_key=JDC4RERC2KWQ8W5K";
const char *ssid = "SQ4BRT";
const char *haslo = "12385498rR";
const char *server = "api.thingspeak.com";

RTC_DATA_ATTR int LiczbaUruchomien = 0;

uint32_t FormulaAproksymacyjna_SGP30(float Temperatura, float Wilgotnosc)
{

  //formula aproksymacyjna zgodnie z nota katologowa sensora Sensiron SGP30
  const float absoluteHumidity = 216.7f * ((Wilgotnosc / 100.0f) * 6.112f * exp((17.62f * Temperatura) / (243.12f + Temperatura)) / (273.15f + Temperatura)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                  // [mg/m^3]
  return absoluteHumidityScaled;
}

void PowodRestartu()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {

  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Uspianie przez zegar!");
    break;

  default:
    Serial.printf("Restart nie byl spowodowany przez Deep Sleep! %d\n", wakeup_reason);
    break;
  }
}

//Sprawdzanie polaczenia z sensorem SGP30

void BME280_Test()
{
  bool status = bme.begin(BME280_Adres);
  if (!status)
  {
    Serial.println("Błąd połączenia z sensorem BME280!");
    while (1)
      ;
  }
}





float PomiarTemperatury()
{
  temperatura = bme.readTemperature();
}

float PomiarWilgnotnosci()
{
  wilgotnosc = bme.readHumidity();
} 


float PomiarCisnienia()
{
  cisnienie = bme.readPressure();
  cisnienie = bme.seaLevelForAltitude(Wysokosc, cisnienie);
  cisnienie = cisnienie / 100.0F;
}

//Funkcja wysylajaca wyniki do chmury ThingSpeak

void WysylanieDanych(float Temperatura, float Wilgotnosc, float Cisnienie, float PM25, float PM10, float eCO2, float tVOC)
{
  HTTPClient http;

  String url = NazwaServera + "&field1=" + Temperatura + "&field2=" + Wilgotnosc + "&field3=" + Cisnienie + "&field4=" + PM25 + "&field5=" + PM10 + "&field6=" + eCO2 + "&field7=" + tVOC;

  http.begin(url.c_str());

  int httpResponseCode = http.GET();

  if (httpResponseCode > 0)
  {
    Serial.print("Odpowiedz HTML: ");
    Serial.println(httpResponseCode);
  }
  else
  {
    Serial.print("Kod bledu: ");
    Serial.println(httpResponseCode);
  }
  http.end();
}

void setup()
{
  Serial.begin(9600);
  WiFi.begin(ssid, haslo);
  Wire.begin(I2C_SDA, I2C_SCL);

  pinMode(Tranzystor_SGP30, OUTPUT);
  digitalWrite(Tranzystor_SGP30, HIGH); //Wlaczenie tranzystora wlaczajacego sensor SGP30
  BME280_Test();                        //Sprawdzenie polaczenia z sensorem BME280
  delay(1000);

  float Temperatura = PomiarTemperatury() / 100; //Pomiar temperatury
  float Wilgotnosc = PomiarWilgnotnosci();       //Pomiar wilgnotnosci
  float Cisnienie = PomiarCisnienia() / 100;     //Pomiar cisnienia

  sds.begin();                                            //Uruchomienie sensora SDS011
  Serial.println(sds.setQueryReportingMode().toString()); //Ustawienie trybu pracy
  sds.wakeup();                                           //Wybudzenie sensora SDS011
  delay(2000);


  PmResult pm = sds.queryPm(); //Zadanie pomiaru SDS011

  if (pm.isOk()){                //Sprawdzenie połączenia z sensorem SDS011
    PM25 = pm.pm25;              //Pomiar PM 2,5
    PM10 = pm.pm10;              //Pomiar PM 10
  }
  else{
    Serial.println("Błąd połączenia z sensorem SDS011!");
  }  

  sgp.begin();                                                           //Uruchomienie sensora SGP30
  sgp.setHumidity(FormulaAproksymacyjna_SGP30(Temperatura, Wilgotnosc)); //Aproksymacja na podstawie wynikow pomiaru temp,wilg
  delay(2000);
  float eCO2;
  float tVOC;
  sgp.IAQmeasure(); //Zadanie pomiaru SGP30

  //Zapewnienie odpowiedniego czasu, po ktorym pomiar SGP30 moze byc wykonany (nagrzanie elementu), wartosci 0 i 400 to wartosci domyslne
  while (sgp.eCO2 == 400 || sgp.TVOC == 0)
  {
    //Dopoki wyniki przyjmuja domyslne wartosci (400 i 0) pomiar jest kontynuowany, sensor sie nagrzewa
    delay(3000);
    sgp.IAQmeasure();
  }

  eCO2 = sgp.eCO2;
  tVOC = sgp.TVOC;

  //Wyswietlenie wynikow
  Serial.print("Temperatura: ");
  Serial.print(Temperatura);
  Serial.println(" *C");
  Serial.print("Wilgotnosc: ");
  Serial.print(Wilgotnosc);
  Serial.println(" %");
  Serial.print("Cisnienie ");
  Serial.print(Cisnienie);
  Serial.println(" hPa");
  Serial.print("PM2.5 = ");
  Serial.println(PM25);
  Serial.print("PM10 = ");
  Serial.println(PM10);
  Serial.print("TVOC ");
  Serial.print(tVOC);
  Serial.print(" ppb\t");
  Serial.print("eCO2 ");
  Serial.print(eCO2);
  Serial.println(" ppm");

  digitalWrite(Tranzystor_SGP30, LOW);                                         //Wylaczenie sensora SGP30 poprzez tranzystor sterujacy
  WysylanieDanych(Temperatura, Wilgotnosc, Cisnienie, PM25, PM10, eCO2, tVOC); //Wyslanie danych do ThingSpeak

  sds.sleep(); //Usypianie ukladu SDS011
  ++LiczbaUruchomien;
  Serial.println("Uruchomienie numer: " + String(LiczbaUruchomien));
  PowodRestartu();
  esp_sleep_enable_timer_wakeup(CzasUsypiania * PrzelicznikNaSekundy);
  Serial.println("Ide spac co: " + String(CzasUsypiania) +
                 " sekundy");

  Serial.println("IDE SPAC !");
  Serial.flush();
  esp_deep_sleep_start();

  // tutaj kod sie konczy, nic wiecej sie nie wykona
}

void loop()
{
}
