#include <Button.h>
#include <FiniteStateMachine.h>
#include <Adafruit_NeoPixel.h>

#define VALV_PUMP 7
#define VALV_ATM 5
#define PUMP	4
#define BTN     2
#define PRS_SEN 9
#define NEOPIX  3

//  Tell compiler that functions exist, just implementated later
extern void idled();
extern void vacuum();
extern void dwell();
extern void drain();
extern void vacuumUpdate();
extern void dwellUpdate();

////	Define the states within the SM

// Idle state
State idle_state = State(idled);			

// Vacuum state, with accompanying update function for specific behaviours and controls 
State vacu_state = State(vacuum, vacuumUpdate, NULL);	

// Dwell state, with accompanying update function for extra dwell state behaviours	
State dwll_state = State(dwell, dwellUpdate, NULL);		

// Drain state
State drin_state = State(drain);	

FSM stateMachine = FSM(idle_state);		//	Initial SM with beginning state
Button button = Button(BTN, PULLUP); 	//	Specify button that user utilizes

unsigned long startTime;				//	Used to track times for Vacuum/Dwell timer
unsigned long setTimeLimit = 30000;		//	User defined time limit

// Different states of the machine: Idle, Vacuum, Dwell, and Drain
enum states {IDLED, VACU, DWLL, DRIN};
const byte NUM_OF_STATES = 4; 	//	Number of total states
Adafruit_NeoPixel ledStat = Adafruit_NeoPixel(1, NEOPIX, NEO_GRB + NEO_KHZ800);		//	Neopixel initializiation

void setup() {
	Serial.begin(9600);
	pinMode(VALV_PUMP, OUTPUT);
	pinMode(VALV_ATM, OUTPUT);
	pinMode(PUMP, OUTPUT);
	pinMode(PRS_SEN, INPUT_PULLUP);
}

void loop() {
	//	Keeps track of button presses to direct button states
	//	Value only exists within this function; Value exists between iterations
    static byte buttonPresses = 0; 
    
    if (button.uniquePress()){
		//	Increment buttonPresses and constrain it to [0, NUM_OF_STATES - 1]
        buttonPresses = ++buttonPresses % NUM_OF_STATES; 

		//	Buttons used to manipulate states in SM
        switch (buttonPresses){
            case IDLED:
                Serial.println("IDLE");
                stateMachine.transitionTo(vacu_state);
                break;
    
            case VACU:
                Serial.println("VACU");
                stateMachine.transitionTo(drin_state);
                break;
    
            case DWLL:
                Serial.println("DWLL");
                stateMachine.transitionTo(drin_state);
                break;
    
            case DRIN:
                Serial.println("DRIN");
                stateMachine.transitionTo(idle_state);
                break;
        }
    }
    
	//	Updates the SM for every loop -- APPLICATION CRITICAL
    stateMachine.update();
}

//	Pump is turned off first, pump valve closes, and finally the atm valve
//		closes. LED status is cleared. 
void idled() {
	digitalWrite(PUMP, LOW);
    digitalWrite(VALV_PUMP, LOW);
    digitalWrite(VALV_ATM, LOW);
        
    ledStat.clear();
    ledStat.show();
}

//	The pump valve opens, then the pump turns on. Atm valve is ensured
//		to be closed due to necessary safety redundancy. Displays 
//		Yellow on LED status.
void vacuum() {
	//	Begin timer for total time between Vacuum and Dwell states
    startTime = millis(); 
	digitalWrite(VALV_PUMP, HIGH);
	digitalWrite(PUMP, HIGH);
    digitalWrite(VALV_ATM, LOW);
	   
    ledStat.setPixelColor(0, ledStat.Color(255, 255,0));
    ledStat.show();
}

//	Keeps track of pressure and transitions to Dwell state if 
//		pressure threshold is met. If time set by setTimeLimit
//		is met, transition immediately to the Drain state.
void vacuumUpdate() {
	//	Keep track of time. Kick into Drain state when time is up. 
	//		Ensures timer has priority over pressure.
    if ((millis() - startTime) > setTimeLimit) {
        stateMachine.immediateTransitionTo(drin_state);
    } else if (digitalRead(PRS_SEN) == LOW) {
		//	If the pressure is at or above the calibrated sensor threshold
		//		the pin will read LOW, setting immediate transition to
		//		the Dwell state
        stateMachine.immediateTransitionTo(dwll_state);
    }
}

//	Pump is first turned off, then the valve pump is turned off. 
//		Atm valve is ensured to be closed. Displays Red on LED status.
void dwell() {
	digitalWrite(PUMP, LOW);
	digitalWrite(VALV_PUMP, LOW);
    digitalWrite(VALV_ATM, LOW);
	    
    ledStat.setPixelColor(0, ledStat.Color(255, 0, 0));
    ledStat.show();
}

//	If pressure fall threshold, SM transitions immediately to Vacuum state.
//		Similar timer exists as in vacuumUpdate.
void dwellUpdate() {
	//	Keep track of time. Kick into Drain state when time is up. 
	//		Ensures timer has priority over pressure.
    if ((millis() - startTime) > setTimeLimit) {
        stateMachine.immediateTransitionTo(drin_state);
    } else if (digitalRead(PRS_SEN) == HIGH) {
		//	If pressure is below set pressure threshold, return to 
		//		Vacuum state to repressurize. 
        stateMachine.immediateTransitionTo(vacu_state);
    }
}

//	Pump is turned off first, then the pump valve closes. The atm valve
//		opens, and displays Green as the LED status.
void drain() {  
	// Close pump valve, turn off pump, and open the atmosphere valve
	//	to equalize with external pressure.
	digitalWrite(PUMP, LOW);
	digitalWrite(VALV_PUMP, LOW);
	digitalWrite(VALV_ATM, HIGH);

    ledStat.setPixelColor(0, ledStat.Color(0, 255,0));
    ledStat.show();
}


