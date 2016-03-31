#define BTN A6

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(BTN, INPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);

    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    
}

void loop() {
  // put your main code here, to run repeatedly:
    delay(1000);
    Serial.println("This is " + String(digitalRead(BTN)));
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
}
