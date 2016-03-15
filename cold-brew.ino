#include <FiniteStateMachine.h>
#include <Adafruit_NeoPixel.h>

#define VALV_PUMP 7
#define VALV_ATM 5
#define PUMP	4
#define BTN     2
#define PRS_SEN 10
#define NEOPIX  3

//  Tell compiler that functions exist, just implementated later
extern void idled();
extern void vacuum();
extern void dwell();
extern void drain();
extern void vacuumUpdate();
extern void dwellUpdate();
extern void drainUpdate();
extern void done();

////	Define the states within the SM

// Idle state
State idle_state = State(idled);			

// Vacuum state, with accompanying update function for specific behaviours and controls 
State vacu_state = State(vacuum, vacuumUpdate, NULL);	

// Dwell state, with accompanying update function for extra dwell state behaviours	
State dwll_state = State(dwell, dwellUpdate, NULL);		

// Drain state, with timeout of X seconds
State drin_state = State(drain, drainUpdate, NULL);	

State done_state = State(done);

FSM stateMachine = FSM(idle_state);		//	Initial SM with beginning state

unsigned long startTime;			//	Used to track times for Vacuum/Dwell timer
unsigned long setTimeLimit = 420000;		//	User defined time limit

unsigned long stateStartTime;			//	Used to track times for entering Vacuum/Dwell states
unsigned long stateDebounceLimit = 30000;	//	Used to define state debounce time limit

/*************************** TIMOUT DURATION ***************************/
unsigned long vacuumTimeLimit  = 120000;
unsigned long dwellTimeLimit = 600000;
unsigned long drainTimeout = 60000;


int buttonState;             // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 100;    // the debounce time; increase if the output flickers

// Different states of the machine: Idle, Vacuum, Dwell, and Drain
enum states {IDLED, VACU, DRIN, DONE, DWLL};
const byte NUM_OF_STATES = 4;	//	Number of total states
Adafruit_NeoPixel ledStat = Adafruit_NeoPixel(1, NEOPIX, NEO_GRB + NEO_KHZ800);		//	Neopixel initializiation


/*************************** COLOR FOR MODES  ***************************/
uint32_t idleLEDColor = ledStat.Color(0, 0, 0);			// Clear color
uint32_t vacuumLEDColor = ledStat.Color(255, 255, 0);		// Yellow
uint32_t drainLEDColor = ledStat.Color(255, 0, 0);		// Red
uint32_t doneLEDColor = ledStat.Color(0, 255, 0);			// Green
uint32_t dwellLEDColor = ledStat.Color(238, 130, 238);	// Purple

void setup() {
	Serial.begin(9600);
	pinMode(VALV_PUMP, OUTPUT);
	pinMode(VALV_ATM, OUTPUT);
	pinMode(PUMP, OUTPUT);
	pinMode(PRS_SEN, INPUT_PULLUP);
	pinMode(BTN, INPUT_PULLUP);
	ledStat.begin();
}

void loop() {
	//	Keeps track of button presses to direct button states
	//	Value only exists within this function; Value exists between iterations
    static int buttonPresses = 0; 

    // read the state of the switch into a local variable:
    int reading = digitalRead(BTN);
    
    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState) {
        // reset the debouncing timer
        lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > debounceDelay) {
        // if the button state has changed:
        if (reading != buttonState) {
          buttonState = reading;
    
          // only toggle the LED if the new button state is HIGH
          if (buttonState == LOW) {
            //    Buttons used to manipulate states in SM
            
        
            //  Increment buttonPresses and constrain it to [0, NUM_OF_STATES - 1]
            buttonPresses = ++buttonPresses % NUM_OF_STATES; 
            
            Serial.print("This is what: ");
            Serial.println(buttonPresses);
            switch (buttonPresses){
                case IDLED:
                	Serial.println("IDLE");
               		stateMachine.transitionTo(idle_state);
                	break;
        
                case VACU:
                	Serial.println("VACU");
                   	//  Begin timer for total time between Vacuum and Dwell states
                   	startTime = millis(); 
					stateStartTime = millis();
                    stateMachine.transitionTo(vacu_state);
                    break;
        
//                case DWLL:
//                    Serial.println("DWLL");
//                    stateMachine.transitionTo(drin_state);
//                    break;
        
                case DRIN:
                	Serial.println("DRIN");
					stateStartTime = millis();
                	stateMachine.transitionTo(drin_state);
                	break;

				case DONE:
					Serial.println("DONE");
					stateMachine.transitionTo(done_state);
					break;
            }
          }
        }
    }

    lastButtonState = reading;

    if ((millis() % 1000) == 0) {
        Serial.println(digitalRead(PRS_SEN));

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

	ledStat.setPixelColor(0, idleLEDColor);
	ledStat.show();
}

//	The pump valve opens, then the pump turns on. Atm valve is ensured
//		to be closed due to necessary safety redundancy. Displays 
//		Yellow on LED status.
void vacuum() {
	digitalWrite(VALV_PUMP, HIGH);
	digitalWrite(PUMP, HIGH);
	digitalWrite(VALV_ATM, LOW);
	   
	ledStat.setPixelColor(0, vacuumLEDColor);
	ledStat.show();
}

//	Keeps track of pressure and transitions to Dwell state if 
//		pressure threshold is met. If time set by setTimeLimit
//		is met, transition immediately to the Drain state.
void vacuumUpdate() {
	//	Keep track of time. Kick into Drain state when time is up. 
	//		Ensures timer has priority over pressure.
//	if ((millis() - startTime) > setTimeLimit) {
//        	stateMachine.immediateTransitionTo(drin_state);
//   	} else if ((digitalRead(PRS_SEN) == LOW) && ((millis() - stateStartTime) > stateDebounceLimit)) {
//    	//	If the pressure is at or above the calibrated sensor threshold
//    	//		the pin will read LOW, setting immediate transition to
//    	//		the Dwell state
//        Serial.print("Vacuum Update:  \t");
//        Serial.println((millis() - stateStartTime));
//        stateStartTime = millis();
//    	stateMachine.immediateTransitionTo(dwll_state);
//	}

	if ((millis() - startTime) > vacuumTimeLimit) {
		stateMachine.immediateTransitionTo(dwll_state);
	}
}

//	Pump is first turned off, then the valve pump is turned off. 
//		Atm valve is ensured to be closed. Displays Red on LED status.
void dwell() {
	digitalWrite(PUMP, LOW);
	digitalWrite(VALV_PUMP, LOW);
   	digitalWrite(VALV_ATM, LOW);
	    
	ledStat.setPixelColor(0, dwellLEDColor);
	ledStat.show();
}

//	If pressure fall threshold, SM transitions immediately to Vacuum state.
//		Similar timer exists as in vacuumUpdate.
void dwellUpdate() {
	//	Keep track of time. Kick into Drain state when time is up. 
	//		Ensures timer has priority over pressure.
//	if ((millis() - startTime) > setTimeLimit) {
//		stateMachine.immediateTransitionTo(drin_state);
//	} else if ((digitalRead(PRS_SEN) == HIGH) && ((millis() - stateStartTime) > stateDebounceLimit)) {
//    	//	If pressure is below set pressure threshold, return to 
//    	//		Vacuum state to repressurize. 
//        Serial.print("Dwell Update: \t");
//        Serial.println((millis() - stateStartTime));
//        stateStartTime = millis();
//		stateMachine.immediateTransitionTo(vacu_state);
//	}

	if ((millis() - startTime) > dwellTimeLimit) {
		stateMachine.immediateTransitionTo(drin_state);	
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

	ledStat.setPixelColor(0, drainLEDColor);
	ledStat.show();
}


void drainUpdate() {
	if((millis() - stateStartTime) >  drainTimeout) {
		stateMachine.transitionTo(done_state);
	}
}

void done() {
	digitalWrite(PUMP, LOW);
	digitalWrite(VALV_PUMP, LOW);
	digitalWrite(VALV_ATM, LOW);

	ledStat.setPixelColor(0, doneLEDColor);
	ledStat.show();
}

