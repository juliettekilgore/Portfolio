// Senior Design PID Code
// Juliette Kilgore

const int pulsePin = 2;       // PUL+, motor speed
const int dirPin = 3;         // DIR+, motor direction
const int alarmPin = 4;       // ALM+, ended up not using this
const int pressurePin = A7;   // Pressure transducer side 1
const int pressurePin2 = A6;  // Pressure transducer side 2

float pulseDelay = 15;   
const float scaleFactor = 59.9; 

unsigned long lastPrintTime = 0; // For non-blocking printing

//PID Begin 
float Kp = 3;
float Ki = 0; //not applicable to this system
float Kd = 0; //not applicable to this system

float err_last = 0; 
float err_dot = 0;   
float error_int = 0; 
float error = 0;
long int ttseclast = 0;


//moving avg constants
const int windowSize = 9;
float pressureHistory[windowSize] = {0};
float pressureHistory2[windowSize] = {0};
float pressureSum = 0;
int pressureIndex = 0;
float pressureSum2 = 0;
int pressureIndex2 = 0;

//pressure trasnducer constants
int pressureReading = 0;
int pressureReading2 = 0;
float pressureVoltage = 0;
float pressureVoltage2 = 0;
float pressurePsi = 0;
float pressurePsi2 = 0;
float speed =0;


void setup() {
  Serial.begin(9600);

  pinMode(pulsePin, OUTPUT); //defining the pin constants above as inputs or outputs
  pinMode(dirPin, OUTPUT);
  pinMode(alarmPin, INPUT);

  digitalWrite(dirPin, LOW); 
}
//not sure if this alarm is needed, putting it in just in case
void loop() {
  float ttsec = millis() / 1000.0;


  if (millis() - lastPrintTime >= 750) { //used to be 1000, chnaged to have faster rcn w/ moving av
    lastPrintTime = millis();
    
    pressureReading = analogRead(pressurePin);
    pressureReading2 = analogRead(pressurePin2);
    pressureVoltage = pressureReading * (5.0 / 1023.0);
    pressureVoltage2 = pressureReading2 * (5.0 / 1023.0);

    //moving av code
    pressureSum -= pressureHistory[pressureIndex];
    pressureHistory[pressureIndex] = pressureVoltage * scaleFactor;
    pressureSum += pressureHistory[pressureIndex];
    pressureIndex = (pressureIndex + 1) % windowSize;

    pressureSum2 -= pressureHistory2[pressureIndex];
    pressureHistory2[pressureIndex] = pressureVoltage2 * scaleFactor;
    pressureSum2 += pressureHistory2[pressureIndex];
    pressureIndex2 = (pressureIndex2 + 1) % windowSize;

    // Calculate moving average
    pressurePsi = pressureSum / windowSize;
    pressurePsi2 = pressureSum2 / windowSize;

    float Ts = ttsec - ttseclast;
    error = 0 - (pressurePsi - pressurePsi2);
    //err_dot = (error - err_last) / Ts;
    //error_int = error_int + error * Ts;
    //if (error_int > 100) error_int = 100; //to prevent intergral error accumulation
    //if (error_int < -100) error_int = -100;

    ttseclast = ttsec;
    err_last = error;

    if (abs(error) >= 0.3) {
      pulseDelay = 15 + abs(Kp*error) + abs(Ki*error_int) + abs(Kd*err_dot); //PID implementation Ki and Kd are 0
    }
    speed = abs(-89 * pulseDelay +2450) *0.001; //equation found experimentally to compare flow rate to speed


    Serial.print("   Pressure before Valve (psi): ");
    Serial.print(pressurePsi2 * 0.1);
    Serial.print("   Differential: ");
    Serial.print((pressurePsi - pressurePsi2)*0.1);
    Serial.print("   Flow Rate L/min: ");
    Serial.println(speed);
  }

  // Run motor
  digitalWrite(pulsePin, HIGH);     
  delayMicroseconds(pulseDelay);    
  digitalWrite(pulsePin, LOW);      
  delayMicroseconds(pulseDelay);   
}

