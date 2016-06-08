#define PUMP 13
#define SOLENOID 14


void setup() {
  // put your setup code here, to run once:

    Serial.begin(9600);
    pinMode(PUMP, OUTPUT);
    pinMode(SOLENOID, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
//  digitalWrite(SOLENOID, LOW);

  delay(1000);
//  digitalWrite(SOLENOID, HIGH);
//  delay(1000);
//  digitalWrite(SOLENOID, LOW);
//  delay(1000);
  digitalWrite(PUMP, HIGH);
  delay(5000);
  digitalWrite(PUMP,LOW);
  delay(5000);
 }
