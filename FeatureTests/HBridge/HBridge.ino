// Drain Motor H-Bridge
#define HBRIDGE_A 10
#define HBRIDGE_B 20
#define MA 3
#define MB 4

void setup() {
  // put your setup code here, to run once:

    Serial.begin(9600);
    pinMode(HBRIDGE_A, OUTPUT);
    pinMode(HBRIDGE_B, OUTPUT);
    pinMode(MA, INPUT_PULLUP);
    pinMode(MB, INPUT_PULLUP);
}
 
void loop() {
  // put your main code here, to run repeatedly:
//  analogWrite(HBRIDGE_A, 0);
//  analogWrite(HBRIDGE_B, 200);
////
//  delay(1000);
//
//  analogWrite(HBRIDGE_B, 0);
//  
//  delay(1000);
// 
//  analogWrite(HBRIDGE_A, 200);
//  
//  delay(1000);
//  
//  analogWrite(HBRIDGE_B, 0);
//  
//  delay(1000);

//
//    Serial.print("B value: ");
//    Serial.println(digitalRead(MB));

//    delay(1000);

//    Serial.print("A value: ");
//    Serial.println(digitalRead(MB));
//    delay(200);
    while (digitalRead(MA) != LOW) {
        analogWrite(HBRIDGE_A, 200);
    }
    analogWrite(HBRIDGE_A, 0);

    delay(3000);

    while (digitalRead(MB) != LOW) {
        analogWrite(HBRIDGE_B, 200);
    }
    analogWrite(HBRIDGE_B, 0);

    Serial.print("Switch A: ");
    Serial.println(digitalRead(MA));

    Serial.print("Switch B: ");
    Serial.println(digitalRead(MB));
    delay(3000);

}
