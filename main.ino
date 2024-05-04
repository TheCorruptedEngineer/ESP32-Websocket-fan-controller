// Constants
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <EEPROM.h>
#define EEPROM_SIZE 1

const char* ssid = "-";
const char* password = "-";

String messagebanana;
// Globals 
WebSocketsServer webSocket = WebSocketsServer(80);
int speed = 0; // Global variable to store speed value
int RPM = 0;   // Global variable to store RPM value
int Hz = 0;    // Global variable to store Hz value
volatile int counter_rpm = 0;
const int ledPin = 16;  // 16 corresponds to GPIO16
const int freq = 25000;
const int ledChannel = 0;
const int resolution = 8;
const int tacho_pin = 25;
const int button_pin = 33;
const int enable_pin = 21;
byte enablePinState = LOW;


int counter = 50; 
#include <ESP32Encoder.h> // https://github.com/madhephaestus/ESP32Encoder.git 
ESP32Encoder encoder;
#define CLK 4 // CLK ENCODER 
#define DT 15 // DT ENCODER 

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void IRAM_ATTR rpm_fan() {
  counter_rpm++;
}

void setup() {
  // Start Serial port
  Serial.begin(115200);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }

  EEPROM.begin(EEPROM_SIZE);
  // Connect to access point
  Serial.println("Connecting");
  WiFi.begin(ssid, password);
  while ( WiFi.status() != WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE,BLACK);
  display.setCursor(0, 0);
  // Display static text
  // Print our IP address
  Serial.println("Connected!");
  Serial.print("My IP address: ");
  Serial.println(WiFi.localIP());
  display.println("ip: " + WiFi.localIP().toString());
  display.display(); 

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  // Configure LED PWM functionality
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, counter);

  // Setup interrupts for encoder and fan tachometer
  pinMode(button_pin, INPUT_PULLUP);
  pinMode(tacho_pin, INPUT);
  pinMode(enable_pin, OUTPUT);
  enablePinState = EEPROM.read(0); 
  digitalWrite(enable_pin, enablePinState); // Set the LED state
  attachInterrupt(digitalPinToInterrupt(button_pin), button_press, FALLING);

  encoder.attachHalfQuad ( DT, CLK );
  encoder.setCount ( 0 );


  //disableCore0WDT();
  xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "Task1",     /* Name of task. */
    10000,       /* Stack size of task */
    NULL,        /* Parameter of the task */
    1,           /* Priority of the task */
    NULL,        /* Task handle to keep track of created task */
    0            /* Pin task to core 0 */
  );
  delay(100);
  
  xTaskCreatePinnedToCore(
    Task2code,   /* Task function. */
    "Task2",     /* Name of task. */
    10000,       /* Stack size of task */
    NULL,        /* Parameter of the task */
    1,           /* Priority of the task */
    NULL,        /* Task handle to keep track of created task */
    1            /* Pin task to core 1 */
  );
  delay(100);
}

String response;
void onWebSocketEvent(uint8_t num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length) {

  // Figure out the type of WebSocket event
  switch(type) {

    // Client has disconnected
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connection from ", num);
        Serial.println(ip.toString());
      }
      break;

    // Echo text message back to client
    case WStype_TEXT:
      Serial.printf("[%u] Text: %s\n", num, payload);
      messagebanana = (const char*)payload;

      // Check if received message is "nya~"
      if (messagebanana.equals("nya~")) {
        webSocket.sendTXT(num, "meow");
      }
      else if (messagebanana.equals("off")) {
        webSocket.sendTXT(num, "turning off");
        speed = counter;
        counter = 0;
      }
      else if (messagebanana.equals("on")) {
        response = "turning on, speed = " + String(speed);
        counter = speed;
        webSocket.sendTXT(num, response);
      }
      else if (messagebanana.equals("!enablepinstate")) {
        response = "enablePinState = " + String(enablePinState);
        enablePinState = !enablePinState;
        EEPROM.write(0, enablePinState);
        EEPROM.commit();
        webSocket.sendTXT(num, response);
      }
      else if (messagebanana.startsWith("set speed")) {
        // Extract speed value from the message
        int spaceIndex = messagebanana.indexOf(' ');
        if (spaceIndex != -1) {
          String speedString = messagebanana.substring(spaceIndex + 7);
          Serial.print("Speed string: ");
          Serial.println(speedString);
          Serial.print("Length of speed string: ");
          Serial.println(speedString.length());
          // Convert speed string to an integer
          speed = speedString.toInt();
          if(speed > 255) speed = 255,counter = 255;
          else if (speed < 0) speed = 0,counter = 0;
          else counter = speed;
          Serial.print("Set speed to: ");
          Serial.println(speed);
          // Send a message back to the client indicating the current speed
          webSocket.sendTXT(num, "Speed set to: " + String(speed));
          }
      }
      else if (messagebanana.equals("info") || messagebanana.equals("status")) {
        response = "speed=" + String(speed) + ",RPM=" + String(RPM) + ",Hz=" + String(Hz) + ",counter=" + String(counter);
        webSocket.sendTXT(num, response);
      }
      else {
        // Otherwise, echo the received message back to the client
        webSocket.sendTXT(num, payload);
      }
      break;

    // For everything else: do nothing
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}


void button_press(){
  if (counter == 0) {
      counter = speed;
  }
  else {
    speed = counter;
    counter = 0;}
}

void Task1code( void * pvParameters ) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  for (;;) {
    ledcWrite(ledChannel, counter);
    if (counter == 0){
      digitalWrite(enable_pin,!enablePinState);
    }
    else digitalWrite(enable_pin,enablePinState);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void Task2code( void * pvParameters ) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  for (;;) {
    counter_rpm = 0;
    attachInterrupt(digitalPinToInterrupt(tacho_pin), rpm_fan, CHANGE);
    detachInterrupt(digitalPinToInterrupt(tacho_pin));
    Hz = counter_rpm / 2;
    RPM = Hz * 60 / 2;
    display.setCursor(0, 26); // Set cursor to the next line
    display.println("HZ: " + String(Hz) + "       ");
    Serial.print("Hz: ");
    Serial.println(Hz);
    display.setCursor(0, 16); // Set cursor to the next line
    display.println("RPM: " + String(RPM) + "       ");
    Serial.print("RPM: ");
    Serial.println(RPM);
    display.setCursor(0, 36); // Set cursor to the next line
    display.println("S: " + String(counter) + "        ");
    Serial.print("Position: ");
    long newPosition = encoder.getCount() / 2;
    Serial.println(newPosition);
    //Serial.println(counter);
    display.display(); 
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void loop() {
  webSocket.loop();
  vTaskDelay(500 / portTICK_PERIOD_MS);
}
