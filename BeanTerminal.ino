// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

#include <Adafruit_Sensor.h>
#include "DHT.h"



//static int DELAYTIME 60000;  // Sleep Time or delay reporting time

#define DHTPIN 0     // what digital pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

boolean newConnect = false;
unsigned long startTime = 0;
int batteryPercentage = 0;
int batteryVoltage = 0;
int temperature = 0;
float h = 0.0;
float t = 0.0;
float f = 0.0;
float hif = 0.0;
float hic = 0.0;
uint8_t scratchBuffer[20] = "Bear Crossing";
float highTemp = -111.0;
float lowTemp = 100.0;
float highHumidity = 0.0;
float lowHumidity = 100.0;

void setup() {
  // Configure Bean to wake up when a client connects
  Bean.enableWakeOnConnect(true);
    
  Serial.begin(); // Bean Serial is always 57600
  startTime = millis();

  dht.begin();
}



void loop() {
    
  getDHT11Data();
    
  // Get Battery Level
  batteryPercentage = Bean.getBatteryLevel();  // Between 0 and 100
  batteryVoltage = Bean.getBatteryVoltage();   // Between 191 and 353 (1.91V-3.53V)
  temperature = Bean.getTemperature();
    
  bool connected = Bean.getConnectionState();
  
  if (connected) {

    if (newConnect) {
      Serial.println("Woke on Connect. Running Bean+Terminal.ino");
      newConnect = false;
    }
        
    if (Serial.available())
    {
      String strInput = "";
      char bInput[64];
      int nLength = 64;
      //int nBytesRead = Serial.readBytesUntil('\n', bInput, nLength);
      int nBytesRead = Serial.readBytes(bInput, nLength);
            
      Bean.setLed(0,0,255); 

      for (int n=0; n<nBytesRead; n++)
      {
        if (bInput[n] != char(13) && bInput[n] != char(10))
        {
          strInput = strInput + bInput[n];
          ///Serial.print(byte(bInput[n]));
          ///Serial.print(" ");
        }
      }
      ///Serial.println( "" );
      ///Serial.print("Received ");
      ///Serial.print(nBytesRead-1);
      ///Serial.print(" bytes:  ");
      ///Serial.println(strInput);
      Bean.setLed(0,0,0); // Turn the Off Bean's LED
            
      // COMMANDS

      // temp
      if (strInput == "temp") {
        sendTemp();
      }
            
      // pins
      if (strInput == "pins") {
        sendPins();
      }
            
      // highs
      if (strInput == "highs") {
        Serial.println();
                Serial.print("Temperatures: H ");
                Serial.print(highTemp, 0);
                Serial.print("℉  L "); 
                Serial.print(lowTemp, 0);
                Serial.println("℉");
              Serial.print("Humidity: H ");
                Serial.print(highHumidity, 0); 
                Serial.print("%  L "); 
                Serial.print(lowHumidity, 0); 
                Serial.println("%");
     }
            
      // sethl  Reset High/Lows
      if (strInput == "sethl") {
                highTemp = f;
                lowTemp = f;
                highHumidity = h;
                lowHumidity = h;
                }
         
            
      // time
      if (strInput == "time") {
        Serial.println();
        Serial.print("Active Time: ");
        Serial.print((millis() - startTime ) / 1000);
        Serial.println(" secs.");
      }
            

    }

  } else { // if connected NOT 
      
    //Bean.sleep(60000);
    
    // Is this where a wake on connect occurs???
    newConnect = true;
    
  }
  Bean.sleep(60000); //Supposidly the bean will run loop() when serial received
}
    
        
void SerialEvent()
{
    
}

void sendTemp()
{
    Serial.println(" "); 
    Serial.println("DHT11:");
    Serial.print("Humidity: ");
    Serial.print(h, 0);
    Serial.print("%   Temperature: ");
    Serial.print(t, 0);
    Serial.print(" ℃ ");
    Serial.print(f, 0);
    Serial.print(" ℉   ");
    Serial.print("Heat Index: ");
    Serial.print(hic, 0);
    Serial.print(" ℃ ");
    Serial.print(hif, 0);
    Serial.println(" ℉");
    Serial.print("Bean Temperature: ");
    Serial.print(temperature);
    Serial.print(" ℃ ");
    Serial.print(temperature*1.8+32, 0);
    Serial.print(" ℉    Battery: ");
    Serial.print(batteryPercentage);
    Serial.print("% - ");
    Serial.print((double) batteryVoltage / 100);
    Serial.println(" Volts");

}

void sendPins()
{
    Serial.println("");
    Serial.print("Digital Pins [0 - 9]: ");
    for (int n=0; n<10; n++)
    {
      Serial.print(" ");
      Serial.print(digitalRead(n));
    }
    Serial.println("");
        
    Serial.print("Analog Pins [A0-A5]: ");
    Serial.print(analogRead(A0));
    Serial.print(" ");
    Serial.print(analogRead(A1));
    Serial.print(" ");
    Serial.print(analogRead(A2));
    Serial.print(" ");
    Serial.print(analogRead(A3));
    Serial.print(" ");
    Serial.print(analogRead(A4));
    Serial.print(" ");
    Serial.print(analogRead(A5));
    Serial.println("");
}

void getDHT11Data()
{
  // Wait a few seconds between measurements.
 // delay(2000);  

    
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  hic = dht.computeHeatIndex(t, h, false);

    // Check to see if new highs or lows
    if (f > highTemp)highTemp = f;
    if (f < lowTemp)lowTemp = f;
    if (h > highHumidity) highHumidity = h;
    if (h < lowHumidity) lowHumidity = h;
    
}