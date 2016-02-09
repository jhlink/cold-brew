#include <Button.h>
#include <FiniteStateMachine.h>
#include <Adafruit_NeoPixel.h>

// Import NeoPixel library to initialize pixel 
#define VALV_PUMP 7
#define VALV_ATM 5
#define PUMP	4
#define BTN     2
#define PRS_SEN 9
#define NEOPIX  3

// Different states of the machine: Idle, Vacuum, Dwell, and Drain
enum states {IDLED, VACU, DWLL, DRIN};
const byte NUMBER_OF_SELECATBLE_STATES = 4; 
Adafruit_NeoPixel ledStat = Adafruit_NeoPixel(1, NEOPIX, NEO_GRB + NEO_KHZ800);
unsigned long startTime;
extern void idled();
extern void vacuum();
extern void dwell();
extern void drain();
/** this is the definitions of the states that our program uses */
State idle_state = State(idled);  //no operation
State vacu_state = State(vacuum);  //this state fades the LEDs in
State dwll_state = State(dwell);  //this state flashes the leds FLASH_ITERATIONS times at 1000/FLASH_INTERVAL
State drin_state = State(drain); //show the circular animation

FSM stateMachine = FSM(idle_state); //initialize state machine, start in state: noop

Button button = Button(BTN, PULLUP); //initialize the button (wire between BUTTON_PIN and ground)

void setup() {
	Serial.begin(9600);

	pinMode(VALV_PUMP, OUTPUT);
	pinMode(VALV_ATM, OUTPUT);
	pinMode(PUMP, OUTPUT);

	pinMode(PRS_SEN, INPUT_PULLUP);
}

void loop() {
	// Execute methods due to volatile state changes

      //counter variable, holds number of button presses
    static byte buttonPresses = 0; //only accessible from this function, value is kept between iterations
    
    if (button.uniquePress()){
        //increment buttonPresses and constrain it to [0, NUMBER_OF_SELECATBLE_STATES-1]
        buttonPresses = ++buttonPresses % NUMBER_OF_SELECATBLE_STATES; 
        /*
          manipulate the state machine by external input and control
        */
        //CONTROL THE STATE
        switch (buttonPresses){
            case IDLED:
                Serial.println("IDLE");
                stateMachine.transitionTo(vacu_state); break;
                break;
    
            case VACU:
                Serial.println("VACU");
                stateMachine.transitionTo(drin_state); break; //first press
                break;
    
            case DWLL:
                Serial.println("DWLL");
                stateMachine.transitionTo(drin_state); break; //second press
                break;
    
            case DRIN:
                Serial.println("DRIN");
                stateMachine.transitionTo(idle_state); break; //second press
                break;
        }
    }
    
    //THIS LINE IS CRITICAL
    //do not remove the stateMachine.update() call, it is what makes this program 'tick'
    stateMachine.update();
}

void idled() {
    digitalWrite(VALV_PUMP, LOW);
    digitalWrite(VALV_ATM, LOW);
    digitalWrite(PUMP, LOW);
    
    ledStat.clear();
    ledStat.show();
}

void vacuum() {
	digitalWrite(VALV_PUMP, HIGH);
    digitalWrite(VALV_ATM, LOW);
	digitalWrite(PUMP, HIGH);
   
    ledStat.setPixelColor(0, ledStat.Color(255, 255,0));
    ledStat.show();
}

void dwell() {
	digitalWrite(VALV_PUMP, LOW);
    digitalWrite(VALV_ATM, LOW);
	digitalWrite(PUMP, LOW);
    
    ledStat.setPixelColor(0, ledStat.Color(255, 0, 0));
    ledStat.show();
}

void drain() {  
	digitalWrite(VALV_PUMP, LOW);
    digitalWrite(VALV_ATM, HIGH);
    digitalWrite(PUMP, LOW);

    ledStat.setPixelColor(0, ledStat.Color(0, 255,0));
    ledStat.show();
}


