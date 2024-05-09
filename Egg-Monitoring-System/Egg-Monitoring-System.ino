#include <DHT.h>
#include <ESP32Servo.h> 

#define DHTPIN 18       // DHT22 Sensor connected to GPIO18
#define SERVOPIN 19     // Servo signal pin connected to GPIO19
#define ECHO_PIN 22     // Ultrasonic sensor echo pin connected to GPIO22
#define TRIG_PIN 23     // Ultrasonic sensor trigger pin connected to GPIO23
#define RED_LED 26      // Red LED pin connected to GPIO26
#define GREEN_LED 25    // Green LED pin connected to GPIO25
#define BUZZER 27       // Buzzer pin connected to GPIO27

#define DHTTYPE DHT22   // DHT sensor type

#define MIN_TEMP 30       // Minimum temperature threshold (in Celsius)
#define MAX_DISTANCE 10   // Maximum distance threshold (in cm)

DHT dht(DHTPIN, DHTTYPE);   // Initialize DHT object
Servo servo;                // Initialize Servo object

void setup() {
  Serial.begin(115200);                         // Start serial communication
  pinMode(RED_LED, OUTPUT);                     // Set red LED pin as output
  pinMode(GREEN_LED, OUTPUT);                   // Set green LED pin as output
  pinMode(BUZZER, OUTPUT);                      // Set buzzer pin as output
  pinMode(TRIG_PIN, OUTPUT);                    // Set trigger pin as output
  pinMode(ECHO_PIN, INPUT);                     // Set echo pin as input
  
  dht.begin();                                  // Initialize DHT sensor
  servo.attach(SERVOPIN);                       // Attach servo to SERVOPIN
}

void loop() {
  float temperature = dht.readTemperature();    // Read temperature from DHT sensor
  float humidity = dht.readHumidity();          // Read humidity from DHT sensor
  float distance = getDistance();               // Read distance from ultrasonic sensor
  
  Serial.print("Temperature: ");                // Print temperature value
  Serial.print(temperature);
  Serial.print(" °C\tHumidity: ");             // Print humidity value
  Serial.print(humidity);
  Serial.print(" %\tDistance: ");              // Print distance value
  Serial.print(distance);
  Serial.println(" cm");
  
  digitalWrite(RED_LED, LOW);                  // Turn off red LED

  if (distance > MAX_DISTANCE) {                // Check if distance is greater than maximum distance threshold
    digitalWrite(GREEN_LED, HIGH);              // Turn on green LED
    digitalWrite(BUZZER, LOW);                  // Turn off buzzer
    if (servo.attached()) {                      // Check if servo is attached
      servo.detach();                            // Detach servo
    }
  } else {
    digitalWrite(GREEN_LED, LOW);               // Turn off green LED
    digitalWrite(BUZZER, HIGH);                 // Turn on buzzer
    if (!servo.attached()) {                     // Check if servo is not attached
      servo.attach(SERVOPIN);                    // Attach servo
    }
    triggerAlert();                              // Call triggerAlert function
  }
  
  delay(2000);                                   // Delay for 2 seconds
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);                   // Set trigger pin to LOW
  delayMicroseconds(2);                           // Wait for 2 microseconds
  digitalWrite(TRIG_PIN, HIGH);                  // Set trigger pin to HIGH
  delayMicroseconds(10);                          // Wait for 10 microseconds
  digitalWrite(TRIG_PIN, LOW);                   // Set trigger pin to LOW
  long duration = pulseIn(ECHO_PIN, HIGH);        // Measure duration of echo pin
  return duration * 0.034 / 2;                    // Calculate distance in cm
}

void triggerAlert() {
  digitalWrite(GREEN_LED, LOW);                  // Turn off green LED
  digitalWrite(RED_LED, HIGH);                   // Turn on red LED
  digitalWrite(BUZZER, HIGH);                    // Turn on buzzer
  servo.write(180);                               // Rotate servo to 180 degrees
  delay(1000);                                    // Delay for 1 second
}
