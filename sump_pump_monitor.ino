#include <UltrasonicSensor.h>
#include "DHTesp.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <BlinkCode.h>

// WIFI Info
const char* ssid = "Lewandowski";
const char* password = "2434stonewell";
//const char* ssid = "2434-IOT24";
//const char* password = "iotiotiot";

// InfluxDB settings
String reportUrl = "http://192.168.0.1:8086/api/v2/write?bucket=sump_pump_monitor";
//String reportUrl = "http://192.168.0.1:1880/update-sensors";


// Distance sensor
// https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
int ULTRASONIC_TRIGGER_PIN = 19;
int ULTRASONIC_ECHO_PIN = 22;
UltrasonicSensor ultrasonic(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN);

// Temperature and humidity sensor (DHT-11)
// https://desire.giesecke.tk/index.php/2018/01/30/esp32-dht11/
DHTesp dht;
int DHT_PIN = 18;
int DHT_MAX_RETRIES = 10; // Maximum number of times to wait for DHT status

// Vibration sensors
const int VIBRATION_PIN_COUNT = 2;
int VIBRATION_PINS[] = {34,35};

// Track the values to be reported
float periodMaxTemperatureC = 0.00;
float periodMaxHumidity = 0.00;
int   periodMaxDistanceMm = 0;
int   periodMaxVibrations[VIBRATION_PIN_COUNT];

// Blink counts for error codes
const int LED_BUILTIN_PIN = 2;
const int ERR_BLINK_OK             = 1;
const int ERR_BLINK_NO_WIFI        = 2;
const int ERR_BLINK_NO_TEMP        = 3;
const int ERR_BLINK_NO_DISTANCE    = 5;
const int ERR_BLINK_HTTP_ERROR     = 6;
using namespace blinkcode;
BlinkCode blinkCode(LED_BUILTIN_PIN);


void setup() {
  pinMode(LED_BUILTIN_PIN, OUTPUT); // initialize GPIO pin 2 LED_BUILTIN as an output.
  blinkCode.begin();

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Wifi status after begin(" + String(ssid) + ", xxxx): " + wl_status());
  wifiConnect();

  // Setup input pins for vibration sensors 
  for (int vibPin = 0; vibPin < VIBRATION_PIN_COUNT; vibPin++) {
    pinMode(VIBRATION_PINS[vibPin], INPUT);
  }

  //Initialize the dht pin and dht object
  dht.setup(DHT_PIN, DHTesp::DHT22);
  // Wait one second for the sensor to be available
  delay(1000);

  resetPeriodValues();

  Serial.println("Setup complete");
}

void resetPeriodValues() {
  periodMaxTemperatureC = 0.00;
  periodMaxHumidity = 0.00;
  periodMaxDistanceMm = 0;
  for (int vibPin = 0; vibPin < VIBRATION_PIN_COUNT; vibPin++) {
    periodMaxVibrations[vibPin] = 0;
  }

}

int loopIteration = 1;
void loop() {
  iteration(loopIteration);
  loopIteration += 1;
  BlinkCode::service(); // Asynchronous service routine for LED blink codes, should be called periodically

  // Every 3000 iterations (roughly 30 seconds?), record stats in influx
  if (loopIteration > 3000) {
    loopIteration = 1; // Reset the iteration counter

    // Reconnect to wifi if needed (rebooting if can't connect)
    if(WiFi.status() != WL_CONNECTED) {
      Serial.println("Wifi not connected, reconnecting");
      wifiConnect();
    }
    
    //Serial.println("Period max values: Temperature="+String(periodMaxTemperatureC)+", Humidity="+String(periodMaxHumidity)+", Distance="+String(periodMaxDistanceMm));
    String msg = "Period max values: ";
    msg = msg + "Temperature=" + String(periodMaxTemperatureC);
    msg = msg + ", Humidity=" + String(periodMaxHumidity);
    msg = msg + ", Distance=" + String(periodMaxDistanceMm);
    msg = msg + ", Vibration=";
    for (int vibPin = 0; vibPin < VIBRATION_PIN_COUNT; vibPin++) {
      msg = msg + String(vibPin) + "=" + periodMaxVibrations[vibPin];
      if ((vibPin + 1) < VIBRATION_PIN_COUNT) {
        msg += ",";
      }
    }
    int httpRespCode = httpPost();

    // Blink LED once if all values were read and HTTP post succeeded
    if(httpRespCode == 200 && periodMaxTemperatureC != 0.00 && periodMaxHumidity != 0.00 && periodMaxDistanceMm != 0) {
      blinkCode.trigger(ERR_BLINK_OK);
    } else {
      // Blink to indicate an HTTP failure
      if(httpRespCode != 200) {
        blinkCode.trigger(ERR_BLINK_HTTP_ERROR);
      }
    }
    Serial.println(msg);
    Serial.println("--------------------------------------------");
    resetPeriodValues();
  }
}

void iteration(int iteration) {
  //Serial.println("In iteration: "+String(iteration));

  // Every iteration, read the vibration sensors and track their maximum value since the last report
  for (int vibPin = 0; vibPin < VIBRATION_PIN_COUNT; vibPin++) {
    int value = analogRead(VIBRATION_PINS[vibPin]);
    if (value > periodMaxVibrations[vibPin]) {
      periodMaxVibrations[vibPin] = value;
    }
  }
  
  // Every 1000 iterations, output the vibration values
  if (iteration % 1000 == 0) {
    String msg = "Current period max vibration: ";
    for (int vibPin = 0; vibPin < VIBRATION_PIN_COUNT; vibPin++) {
      msg = msg + String(vibPin) + "=" + periodMaxVibrations[vibPin];
      if ((vibPin+1) < VIBRATION_PIN_COUNT) {
        msg += ",";
      }
    }
    Serial.println(msg);
  }


  // Every 1000 iterations, read the temperature and humidity values
  if (iteration % 1000 == 0) {
    int dhtRetryCount = -1;
dhtFlag: TempAndHumidity newValues = dht.getTempAndHumidity(); //Get the Temperature and humidity
    int dhtStatus = dht.getStatus();
    // Wait for the DHT sensor to be ready
    while (dhtStatus != 0 && dhtRetryCount < DHT_MAX_RETRIES) {
      dhtRetryCount = dhtRetryCount + 1;
      Serial.println("DHT not ready to be read: status=" + String(dhtStatus) + ", retry count: " + String(dhtRetryCount));
      delay(300);
      if(dhtRetryCount == DHT_MAX_RETRIES) {
        Serial.println("Unable to read DHT value");
        blinkCode.trigger(ERR_BLINK_NO_TEMP);
      }
      goto dhtFlag;
    }
    float temperature = 0.00;
    float humidity = 0.00;
    if (dhtStatus == 0) {
      temperature = newValues.temperature;
      humidity = newValues.humidity;
    }
    Serial.println("Current temperature:" + String(temperature) + " Humidity:" + String(humidity));
    if (temperature > periodMaxTemperatureC) {
      periodMaxTemperatureC = temperature;
    }
    if (humidity > periodMaxHumidity) {
      periodMaxHumidity = humidity;
    }
  }

  // Every 1000 iterations, read the distance sensor
  if (iteration % 1000 == 0) {
    // Configure the ultrasonic sensor with the current temperature
    int tempCForDistanceSensor = 22; // Default value if unable to read temperature sensor
    if (periodMaxTemperatureC != 0.00) {
      tempCForDistanceSensor = (int)periodMaxTemperatureC;
    }
    ultrasonic.setTemperature(tempCForDistanceSensor);
    // Read the distance sensor value
    int distanceMm = ultrasonic.distanceInMillimeters();
    Serial.printf("Current Distance: %dmm\n", distanceMm);
    if(distanceMm == 0) {
      Serial.println("Unable to read distance value");
      blinkCode.trigger(ERR_BLINK_NO_DISTANCE);
    }
    if (distanceMm > periodMaxDistanceMm) {
      periodMaxDistanceMm = distanceMm;
    }

  }

  delay(10);
}

int httpPost() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String serverPath = reportUrl;

    http.begin(serverPath.c_str());

    // Data to send with HTTP POST
    String httpRequestData = "levelMm value=" + String(periodMaxDistanceMm);
    httpRequestData = httpRequestData + "\n" + "temperature value=" + String(periodMaxTemperatureC);
    httpRequestData = httpRequestData + "\n" + "humidity value=" + String(periodMaxHumidity);
    for (int vibPin = 0; vibPin < VIBRATION_PIN_COUNT; vibPin++) {
      httpRequestData = httpRequestData + "\n" + "vibration,sensorNum=" + String(vibPin) + " value=" + String(periodMaxVibrations[vibPin]);
    }

    // Send HTTP POST request
    Serial.println("Sending HTTP POST to " + String(reportUrl));
    int httpResponseCode = http.POST(httpRequestData);

    Serial.println("HTTP Response code: "+httpResponseCode);

    // Free resources
    http.end();
    return httpResponseCode;
  }
  Serial.println("Cannot post to influx, WiFi Disconnected");
  return -1;
}

void wifiConnect() {
   // Wait for wifi to be connected
  uint32_t notConnectedCounter = 0;
  Serial.println("Wifi connecting " + String(ssid));
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    notConnectedCounter++;
    //Serial.print(".");
    Serial.println(wl_status());
    if (notConnectedCounter == 10) { // Restart wifi if not connected after 5s
      Serial.println("Wifi connecting " + String(ssid));
      WiFi.begin(ssid, password);
    }

    if (notConnectedCounter > 20) { // Reset board if still not connected
      Serial.println("\nWifi status: " + String(WiFi.status()));
      Serial.println("\nResetting due to Wifi not connecting...");
      ESP.restart();
    }
  }
  Serial.print("Wifi connected, IP address: ");
  Serial.println(WiFi.localIP());
  blinkCode.trigger(ERR_BLINK_OK);
}

const char* wl_status() {
  wl_status_t status = WiFi.status();
  switch (status) {
    case WL_NO_SHIELD: return "WL_NO_SHIELD";
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";
  }
  // Unknown status
  return "WL_UNKNOWN_STATUS";
}
