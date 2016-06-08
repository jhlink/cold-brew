#define PRESSURE 2


void setup() {
  // put your setup code here, to run once:

    Serial.begin(9600);
    pinMode(PRESSURE, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
//  digitalWrite(SOLENOID, LOW);

  delay(1000);
  int readPressure = analogRead(PRESSURE);
  float convertedPressure = convertVoltageToPressure(readPressure);
  Serial.println(convertedPressure);
  delay(3000);
}

float convertVoltageToPressure(int inputValue) {
    //  Convert analogValue to voltage.
    float voltageValue = inputValue * 3.3/1024
    
    //  Convert Vout voltage divider value into Vin value. 
    float result = voltageValue * 3/2.0;

    //  Formula is as follows
    //      Vout = Vs * (0.018  * P + 0.92)  +- (PressureError * TempMultiplier * .018 * Vs)
    //      Vs = 5.0 += 0.25V

    float finalResult = (result - (.92 * 5.0)) / (.018 * 5.0);
    return finalResult;
}

