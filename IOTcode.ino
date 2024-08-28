#include <ESP8266WiFi.h>
#include <ThingerESP8266.h>
#include <SoftwareSerial.h>

SoftwareSerial espSerial(D1); // Recieving pin

#define USERNAME "aadhi16" //ThingerIO username
#define DEVICE_ID "microproject" //Device ID defined in ThingerIO
#define DEVICE_CREDENTIAL "lm_Ac6_SkehRBCn!"//Credential defined in ThingerIO

#define SSID "Rehabitat35"// Wi-fi SSID
#define SSID_PASSWORD "GVAS75jm" // Wi-Fi Password

ThingerESP8266 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL); 

float temperature;
int light;
int moisture;
String ledStatus;
String fanStatus;
String motorStatus;

void setup() {
  Serial.begin(9600); // Serial monitor for debugging
  espSerial.begin(9600); // Software serial to match the baud rate of the Arduino Due
  WiFi.begin(SSID, SSID_PASSWORD);

  thing.add_wifi(SSID, SSID_PASSWORD);

   thing["ArduinoDueData"] >> [](pson& out) {
    out["Temperature"] = temperature;
    out["Light"] = light;
    out["Moisture"] = moisture;
    out["LED Status"] = ledStatus;
    out["Fan Status"] = fanStatus;
    out["Motor Status"] = motorStatus;
  }; // Values transmitted by the Arduino Due to NodeMCU are updated to the created things of ThingerIO
}

void loop() {
  if (espSerial.available()) {
    String data = espSerial.readStringUntil('\n'); // Reads data transmitted until a new line is found
    Serial.println("Raw data received: " + data); // Debug print to verify raw data
    
    int firstComma = data.indexOf(','); // Stores the index of the end of first value
    int secondComma = data.indexOf(',', firstComma + 1); // Stores the index of the end of second value
    int thirdComma = data.indexOf(',', secondComma + 1); // Stores the index of the end of third value
    int fourthComma = data.indexOf(',', thirdComma + 1); // Stores the index of the end of fourth value
    int fifthComma = data.indexOf(',', fourthComma + 1); // Stores the index of the end of fifth value

    if (firstComma != -1 && secondComma != -1 && thirdComma != -1 && fourthComma != -1 && fifthComma != -1) {
      temperature = data.substring(0, firstComma).toFloat(); // String parsing and conversion to float for temperature
      light = data.substring(firstComma + 1, secondComma).toInt(); // String parsing and conversion to int for light
      moisture = data.substring(secondComma + 1, thirdComma).toInt(); // String parsing and conversion to int for moisture
      ledStatus = data.substring(thirdComma + 1, fourthComma); // Parsing LED status
      fanStatus = data.substring(fourthComma + 1, fifthComma); // Parsing Fan status
      motorStatus = data.substring(fifthComma + 1); // Parsing Motor status

      Serial.println("Temperature: " + String(temperature)); // Prints temperature on serial monitor
      Serial.println("Light: " + String(light)); // Prints light on serial monitor
      Serial.println("Moisture: " + String(moisture)); // Prints moisture on serial monitor
      Serial.println("LED Status: " + ledStatus); // Prints LED status on serial monitor
      Serial.println("Fan Status: " + fanStatus); // Prints fan status on serial monitor
      Serial.println("Motor Status: " + motorStatus); // Prints motor status on serial monitor
    } else {
      Serial.println("Error."); // If something goes wrong while parsing or all the data is not received it prints error
    }
  }
  
  thing.handle();
}
