// Stepper Motor for door
int PUL = 7;  // (PUL-) of Stepper for door
int DIR = 6;  // (DIR-) of Stepper for door 
int ENA = 5;  // (EN-) of Stepper for door

//Conveyor Belt
const int pwmPin = 11;  // PWM Pin of Conveyor Belt 
const int dirPin = 2;   // DIrection Pin of Conveyor Belt 

// UltraSonic Sensor
const int echo = 8;     // Eho of Ultrasonic 
const int trig = 12;    // Trigger Pin of Ultrasonic

// Delay for Ultrasonic Sensor
unsigned long previousMillis = 0;  // For Counting Delay
const long interval = 50;  // Threshold for Delay

// Signal for Detecting Object in front of the UltraSonic Sensor
int signal = 0;

// Variables for Working door
float count1 = 0;
float count2 = 0;

// Setup for Initial Pin setting
void setup() {
    pinMode(PUL, OUTPUT);     // Stepper for door
    pinMode(DIR, OUTPUT);     // Stepper for door
    pinMode(ENA, OUTPUT);     // Stepper for door
    pinMode(dirPin, OUTPUT);  // Conveyor Belt
    pinMode(pwmPin, OUTPUT);  // Conveyor Belt
    pinMode(trig, OUTPUT);    // UltraSonic
    pinMode(echo, INPUT);     // UltraSonic
    Serial.begin(9600);       // Start Serial communication
}

void loop() {
    static float distance = 0;               // UltraSonic Distance
    unsigned long currentMillis = millis();  // Current Time

    // Delay Upload of Distance from UltraSonic Sensor 
    if (currentMillis - previousMillis >= interval) { // if time is over 50ms
        previousMillis = currentMillis;  // Reset Delay Counting
        distance = measureDistance();    // Upload Distance
    }

    // Stepper for Conveyor Belt 
    if (distance > 30) { // Object X
        signal = LOW; // Signal for No Object
        analogWrite(pwmPin, 127);  // Speed
        digitalWrite(dirPin, LOW); // Forward
    } else { // Object O
        signal = HIGH; // Signal for Object
        analogWrite(pwmPin, 0);  // Stop
        digitalWrite(dirPin, LOW);
    }

    // Stepper for door
    if (signal == HIGH) { // if Object O
        if (count1 == 30) { // Wating for the moment to close the door
            // Close the door
            for (int j = 0; j < 300; j++) {
                    digitalWrite(DIR, LOW);  
                    digitalWrite(ENA, HIGH);  
                    digitalWrite(PUL, HIGH);
                    delayMicroseconds(100);
                    digitalWrite(PUL, LOW);
                    delayMicroseconds(100);
                }
            digitalWrite(ENA, LOW);  // Deactivate motor after movement
        }
        
        count1 += 1; // Counting 30 times
        count2 = 0; // Reset
    }
    else if (signal == LOW) { // if Object X
        // Open
        if (count2 == 1) { // Open Only Once After Object Disappears
            // Open the door
            for (int i = 0; i < 300; i++) {
                    digitalWrite(DIR, HIGH);  
                    digitalWrite(ENA, HIGH);
                    digitalWrite(PUL, HIGH);
                    delayMicroseconds(100);
                    digitalWrite(PUL, LOW);
                    delayMicroseconds(100);
                }
            digitalWrite(ENA, LOW);  // Deactivate motor after movement
        }
        count2 += 1; // Counting Only Once time
        count1 = 0; // Reset
    }
}

// Function for UltraSonic Sensor
float measureDistance() {
    digitalWrite(trig, LOW);  
    delayMicroseconds(2);     
    digitalWrite(trig, HIGH); 
    delayMicroseconds(10);    
    digitalWrite(trig, LOW);  
    float cycletime = pulseIn(echo, HIGH);  
    float distance = ((340.0 * cycletime) / 10000.0) / 2.0; 
    return distance; 
}
