#define REL1 7
#define REL2 6
#define REL3 5
#define REL4 4
#define BTN	 3
#define LEDST 2

enum states {idle, vacu, dwll, drin}
class Machine
{
    // Class Member Variables
    // These are initialized at startup
	int state;	//	Default state is idle
    long vacDwlTime;    // milliseconds of off-time
 
  // Constructor - creates a Flasher 
  // and initializes the member variables and state
  public:
  Machine(int startState, long setTime)
  {
    ledPin = pin;
    pinMode(ledPin, OUTPUT);     
      
    OnTime = on;
    OffTime = off;
    
    ledState = LOW; 
    previousMillis = 0;
  }
 
  void Update(unsigned long currentMillis)
  {
    if((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
    {
        ledState = LOW;  // Turn it off
      previousMillis = currentMillis;  // Remember the time
      digitalWrite(ledPin, ledState);  // Update the actual LED
    }
    else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
    {
      ledState = HIGH;  // turn it on
      previousMillis = currentMillis;   // Remember the time
      digitalWrite(ledPin, ledState);     // Update the actual LED
    }
  }
};

void setup() {
	Serial.begin(9600);

	pinMode(REL1, OUTPUT);
	pinMode(REL2, OUTPUT);
	pinMode(REL3, OUTPUT);
	pinMode(REL4, OUTPUT);
}

void loop() {

}


