// Drain Motor H-Bridge
#define HBRIDGE_A 10
#define HBRIDGE_B 20


void setup() {
  // put your setup code here, to run once:

    Serial.begin(9600);
    pinMode(HBRIDGE_A, OUTPUT);
    pinMode(HBRIDGE_B, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(HBRIDGE_A, 0);
  analogWrite(HBRIDGE_B, 200);

  delay(1000);

  analogWrite(HBRIDGE_B, 0);
  
  delay(1000);
 
  analogWrite(HBRIDGE_A, 200);
  
  delay(1000);
  
  analogWrite(HBRIDGE_B, 0);
  
  delay(1000);

}
