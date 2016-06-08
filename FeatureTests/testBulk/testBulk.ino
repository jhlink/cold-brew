// Drain Motor H-Bridge
#define HBRIDGE_A 10
#define HBRIDGE_B 20

#define PUMP 13
#define SOLENOID 14

#define PRESSURE 2
void setup() {
	// put your setup code here, to run once:

	Serial.begin(9600);
	pinMode(HBRIDGE_A, OUTPUT);
	pinMode(HBRIDGE_B, OUTPUT);
	pinMode(PUMP, OUTPUT);
	pinMode(SOLENOID, OUTPUT);
	pinMode(PRESSURE, INPUT);
}

float convertVoltageToPressure(int inputValue) {
	//  Convert analogValue to voltage.
	float voltageValue = inputValue * 3.3/1024;

		//  Convert Vout voltage divider value into Vin value. 
		float result = voltageValue * 3/2.0;

	//  Formula is as follows
	//      Vout = Vs * (0.018  * P + 0.92)  +- (PressureError * TempMultiplier * .018 * Vs)
	//      Vs = 5.0 += 0.25V

	float finalResult = (result - (.92 * 5.0)) / (.018 * 5.0);
	return finalResult;
}

void loop() {
	// put your main code here, to run repeatedly:
	//  digitalWrite(SOLENOID, LOW);

	openSolenoid(4000);
	openPump(5000);
	forwardHbridge(5000);
	reverseHbridge(5000);

	int readPressure = analogRead(PRESSURE);
	float convertedPressure = convertVoltageToPressure(readPressure);
	Serial.println(convertedPressure);
	delay(3000);

}

void openSolenoid(int time) {
	digitalWrite(SOLENOID, HIGH);
	delay(time);
	digitalWrite(SOLENOID, LOW);
	delay(2000);
}

void openPump(int time) {
	digitalWrite(PUMP, HIGH);
	delay(time);
	digitalWrite(PUMP, LOW);
	delay(2000);
}

void forwardHbridge(int time) { 
	digitalWrite(HBRIDGE_B, LOW);
	digitalWrite(HBRIDGE_A, HIGH);
	delay(time);

	digitalWrite(HBRIDGE_A, LOW);
	delay(2000);
}

void reverseHbridge(int time) { 
	digitalWrite(HBRIDGE_A, LOW);
	digitalWrite(HBRIDGE_B, HIGH);
	delay(time);

	digitalWrite(HBRIDGE_B, LOW);
	delay(2000);
}


