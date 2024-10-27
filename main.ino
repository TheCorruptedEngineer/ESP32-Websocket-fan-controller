// Constants
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <EEPROM.h>
#define EEPROM_SIZE 1
#define R1 10000
#define R2 20000
const char* ssid = "-";
const char* password = "-"
String messagebanana;
// Globals 
WebSocketsServer webSocket = WebSocketsServer(80);
int speed = 0; // Global variable to store speed value
int RPM = 0;   // Global variable to store RPM value
int Hz = 0;    // Global variable to store Hz value
volatile int counter_rpm = 0;
const int pwmPin = 32;  // 16 corresponds to GPIO16
const int freq = 25000;
const int ledChannel = 0;
const int resolution = 8;
const int tacho_pin = 25;
const int button_pin = 14;
const int enable_pin = 19;
///const int current_pin = 34;
///float currentA = 0;
byte enablePinState = LOW;

#include <ESP32Servo.h>
ESP32PWM pwm;

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

  // Setup interrupts for encoder and fan tachometer
  pinMode(button_pin, INPUT_PULLUP);
  pinMode(tacho_pin, INPUT);
  pinMode(enable_pin, OUTPUT);
  enablePinState = EEPROM.read(0); 
  digitalWrite(enable_pin, enablePinState); // Set the LED state

  encoder.attachHalfQuad ( DT, CLK );
  encoder.setCount ( 0 );

	ESP32PWM::allocateTimer(3);
  pwm.attachPin(pwmPin, freq, resolution);
  pwm.write(counter);


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
        encoder.setCount(0); // Set encoder count to 0
        counter = 0;
      }
      else if (messagebanana.equals("on")) {
        response = "turning on, speed = " + String(speed);
        encoder.setCount(speed * 2); // Set encoder count to 0
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
          if(speed > 100) speed = 100,counter = 100,encoder.setCount(200);
          else if (speed < 0) speed = 0,counter = 0,encoder.setCount(0);
          else counter = speed,encoder.setCount(speed * 2);
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

void updateCount() {
    long new_count = encoder.getCount() / 2; // Get the new count value

    // Check if new_count is less than 0
    if (new_count < 0) {
        new_count = 0; // Set new_count to 0
        encoder.setCount(0); // Set encoder count to 0
    }

    // Check if new_count is greater than 100
    if (new_count > 100) {
        new_count = 100; // Set new_count to 100
        encoder.setCount(200); // Set encoder count to 100
    }

    counter = new_count; // Update the global count variable with the new value
}

void button_press(){
    if (counter == 0) {
      encoder.setCount(speed * 2);
      counter = speed;
    }
    else {
      speed = counter;
      encoder.setCount(0);
      counter = 0;
    }
    
}

void Task1code( void * pvParameters ) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  for (;;) {
    if (counter == 0){
      digitalWrite(enable_pin,!enablePinState);
    }
    else digitalWrite(enable_pin,enablePinState);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if(button_pin == LOW){
      button_press();
      delay(100);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}}

void Task2code( void * pvParameters ) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  for (;;) {
    counter_rpm = 0;
    attachInterrupt(digitalPinToInterrupt(tacho_pin), rpm_fan, CHANGE);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    detachInterrupt(digitalPinToInterrupt(tacho_pin));
    Hz = counter_rpm / 2;
    RPM = Hz * 60 / 2;
    ///float adc = analogRead(current_pin);
    ///float adc_voltage = adc * (3.3 / 4096.0);
    ///float currentA_voltage = (adc_voltage * (R1+R2)/R2);
    ///currentA = (currentA_voltage - 2.5) / 0.100;
    display.setCursor(0, 26); // Set cursor to the next line
    display.println("HZ: " + String(Hz) + "       ");
    Serial.print("Hz: ");
    Serial.println(Hz);
    display.setCursor(0, 16); // Set cursor to the next line
    display.println("RPM: " + String(RPM) + "       ");
    Serial.print("RPM: ");
    Serial.println(RPM);
    display.setCursor(0, 36); // Set cursor to the next line
    updateCount();
    Serial.print("Position: ");
    display.println("S: " + String(counter) + "        ");
    Serial.println(counter);
    ///Serial.print("Current Value: ");
    ///Serial.println(currentA);
    ///display.setCursor(0, 46); // Set cursor to the next line
    ///display.println("A: " + String(currentA) + "       ");
    pwm.write(map(counter, 0, 100, 0, 255));
    //long newPosition = encoder.getCount() / 2;
    // display.println("S: " + String(newPosition) + "        ");
    //Serial.println(newPosition);
    //pwm.write(newPosition);
    display.display(); 
  }
}

void loop() {
  webSocket.loop();
  vTaskDelay(500 / portTICK_PERIOD_MS);
}