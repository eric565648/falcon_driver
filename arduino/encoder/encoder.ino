#define ENCODERPINA 3
#define ENCODERPINB 2
#define CPR 1024
#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 2
#define PRINT_POS 0

// variables modified by interrupt handler must be declared as volatile
volatile long encoderPosition=0;
volatile long interruptsReceived=0;

// track direction: 0 = counter-clockwise; 1 = clockwise
short currentDirection=CLOCKWISE;

// track last position so we know whether it's worth printing new output
long previousPosition=0;

void onInterrupt(){

  // read both inputs
  int a = digitalRead(ENCODERPINA);
  int b = digitalRead(ENCODERPINB);

  if(a==b){
    // b is leading a (counter-clockwise)
    encoderPosition--;
    currentDirection = COUNTER_CLOCKWISE;
  }
  else{
    // a is leading b (clockwise)
    encoderPosition++;
    currentDirection = CLOCKWISE;
  }

  encoderPosition = encoderPosition % CPR;
  interruptsReceived++;
}

void setup() {
  
  // inputs
  pinMode(ENCODERPINA,INPUT);
  pinMode(ENCODERPINB,INPUT);

  // interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODERPINA),onInterrupt,RISING);
//  attachInterrupt(digitalPinToInterrupt(ENCODERPINB),onInterrupt,RISING);

  // enable diagnostic output
  Serial.begin(9600);
  Serial.println("\n\n");
  Serial.println("Ready.");

}

void loop() {
  
  if(((encoderPosition - previousPosition)>PRINT_POS) || ((previousPosition-encoderPosition)>PRINT_POS)){
    Serial.print(encoderPosition,DEC);
    Serial.print("\t");
    Serial.print(currentDirection == CLOCKWISE ? "clockwise" : "counter-clockwise");
    Serial.print("\t");
    Serial.println(interruptsReceived, DEC);
    previousPosition = encoderPosition;
  }

}
