/*
 * This software allows for the control of the motors for a gripper equipped with two fingers and for the logging of the suction flow
 * pressure in the tubes connected to the fingertips.
 * 
 * Author: Philippe Nadeau <philippe.nadeau.3@ens.etsmtl.ca>
 * Date of last update :  June 21th 2019
 */

//If this is set to true, then a PI position controller will be used and the command
//will correspond to the desired position. This requires the used of homing switch and the encoder.
//If its set to false then a fixed speed will be used and the command will correspond
//to the amount of milliseconds the speed should be maintained.
const bool PI_CONTROLLER_USED = false;
bool motorsInMovement         = false;
//Used Arduino Pins below

//Pulling this pin high disables the driver.
#define MOTORS_DRIVER_DISABLE 4

//By default, this input signal is LOW and is pulled HIGH when the homing contact is made.
#define FINGER1_HOMING      A2
//The two channels of the quadrature encoder, going forward A precedes B.
//The encoder offers 48 edges per revolution. If we take into account the
//gear ration of 75:1, we get 3600 counts per output revolution.
#define FINGER1_ENCODER_A   11
#define FINGER1_ENCODER_B   2
//Ths pin tells in which direction the driver should propell the motor.
//High = Closes the finger
//Low  = Opens the finger 
#define FINGER1_DRIVER_DIR  7
//Duty-cycle of the motor, determines the speed
#define FINGER1_DRIVER_PWM  9
//Analog reading of the pressure
#define FINGER1_PRESSURE    A3

#define FINGER2_HOMING      A4
#define FINGER2_ENCODER_A   13
#define FINGER2_ENCODER_B   3
#define FINGER2_DRIVER_DIR  8
#define FINGER2_DRIVER_PWM  10
#define FINGER2_PRESSURE    A5

//Sampling period in microseconds used for the control loop
int samplingPeriod = 10; //10mS = 0.01 second

//This variable is incremented/decremented for each encoder count
//depending on the direction of the motor.
volatile unsigned int finger1positionCounts = 0;
volatile unsigned int finger2positionCounts = 0;

//Time at which the control loop was started
unsigned long previousTime          = 0;
volatile int finger1lastError       = 0;
volatile int finger1lastCommand     = 0;
volatile int finger2lastError       = 0;
volatile int finger2lastCommand     = 0;

//This array keeps track of the received bytes
const int commandBufferSize = 14;
char commandBuffer[commandBufferSize] = "";
byte commandBufferIndex = 0;

//These variables keeps track of the position
//the user wants to reach.
int motor1Command = 0;
int motor2Command = 0;
bool motor1Forward = true;
bool motor2Forward = true;

//This variable keeps track of the timestamp
//since the speed command was executed.
unsigned long speedCommandTimestamp  = 0;
//Amplitude of the PWM use for speed control
const int motor1SpeedCommandAmplitude = 127;
const int motor2SpeedCommandAmplitude = 137;

//Quick helper to decompose a 16 bits integer into its MSB and LSB
//and transmit those on Serial. That way, the receiver always get
//the same number of bytes from the sender.
void sendInteger(uint16_t integer){
  byte LSB = (integer & 0x00FF);
  byte MSB = ((integer & 0xFF00) >>8);
  Serial.write(MSB);
  Serial.write(LSB);
}

/* 
 * This function alters the position counter when called by the ISR handler.
 * The channel A has a 90 degrees ADVANCE on channel B (quadrature)
 * By design, this function must be triggered on a edge of channel B.
 */
int encoderChannelBTrigger(int channelA_pin, int channelB_pin, int homing_pin, int counter){
  /*Serial.print(digitalRead(channelA_pin));
  Serial.print(',');
  Serial.print(digitalRead(channelB_pin));
  Serial.print('\n');*/
  
  //If channel B is HIGH, it means a rising edge triggered the interrupt
  if(digitalRead(channelB_pin) == HIGH){
    //If channel A is already HIGH, it means we're moving forward since channel A precedes channel B
    if(digitalRead(channelA_pin) == LOW)
      counter++;
    //If channel A is not yet HIGH, it means we're moving backward
    if(digitalRead(channelA_pin) == HIGH)
      //If the counter goes under zero, it will wrap around the unsigned integer.
      if(counter > 0 )
        counter--;
  }
  
  //If channel B is LOW, it means a falling edge triggered the interrupt
  if(digitalRead(channelB_pin) == LOW){
    //If channel A is already LOW, it means we're moving forward since channel A precedes channel B
    if(digitalRead(channelA_pin) == HIGH)
      counter++;
    //If channel A is not yet LOW, it means we're moving backward
    if(digitalRead(channelA_pin) == LOW)
      //If the counter goes under zero, it will wrap around the unsigned integer.
      if(counter > 0 )
        counter--;
  }

  //When using the PI controller, if the homing switch is HIGH, it means we are home.
  if(PI_CONTROLLER_USED == true && analogRead(homing_pin) > 512)
      counter = 0;

  return counter;
}

/*
 * This function is called on a rising or falling edge of
 * the signal coming out of the Channel B of Motor 1.
 */
void interruptHandlerMotor1ChannelB(){
  //Calls the counting function with the pins associated with motor #1.
  finger1positionCounts = encoderChannelBTrigger(FINGER1_ENCODER_A, FINGER1_ENCODER_B, FINGER1_HOMING, finger1positionCounts);
}

/*
 * This function is called on a rising or falling edge of
 * the signal coming out of the Channel B of Motor 2.
 */
void interruptHandlerMotor2ChannelB(){
  //Calls the counting function with the pins associated with motor #2.
  finger2positionCounts = encoderChannelBTrigger(FINGER2_ENCODER_A, FINGER2_ENCODER_B, FINGER2_HOMING, finger2positionCounts);
}

/*
 * Uses the differential pressure sensor to read the
 * current values and transmit 6 bytes over the Serial interface.
 */
void transmitPressureReadings(){
  /*
  According to the datasheet: Vout = Vs*(0.018*P+0.5)
  Therefore   : P = (Vout - 0.5*Vs)/(0.018*Vs)
  If Vs = 5.0 : P = 11.1111*(Vout-0.25) [kPa]
  If the Vout is within [0,5] then P is within [-27.78 , 27.78]
    which roughly corresponds to datasheet numbers of [-25 , 25]
    
  As the ADC on the Arduino Uno is 10 bits, the resolution is 50/1024 = 0.05 kPa
  As per the sensor's datasheet, there is a maximal error of +/- 1.25 kPa for each reading
  which is 25 times the resolution. This needs to be taken into account when processing the
  data afterward.
  */
  int f1_Vout = analogRead(FINGER1_PRESSURE);
  int f2_Vout = analogRead(FINGER2_PRESSURE);

  //To allow for a faster execution of the function, the raw voltage will be sent
  //instead of the computed pressure value.
  sendInteger(f1_Vout);
  Serial.write(',');
  sendInteger(f2_Vout);
  Serial.print('\n');
}

//Takes a value between -255 and 255 to set the PWM
//The sign sets the direction while the amplitude sets the duty cycle
void setPWMMotor1(int signedDutycycle){
  if(signedDutycycle < 0)
    digitalWrite(FINGER1_DRIVER_DIR, LOW);
  else
    digitalWrite(FINGER1_DRIVER_DIR, HIGH);
    
  if(abs(signedDutycycle) > 255)
    analogWrite(FINGER1_DRIVER_PWM,255);
  else
    analogWrite(FINGER1_DRIVER_PWM,abs(signedDutycycle));
}


//Takes a value between -255 and 255 to set the PWM
//The sign sets the direction while the amplitude sets the duty cycle
void setPWMMotor2(int signedDutycycle){
  if(signedDutycycle < 0)
    digitalWrite(FINGER2_DRIVER_DIR, LOW);
  else
    digitalWrite(FINGER2_DRIVER_DIR, HIGH);
    
  if(abs(signedDutycycle) > 255)
    analogWrite(FINGER2_DRIVER_PWM,255);
  else
    analogWrite(FINGER2_DRIVER_PWM,abs(signedDutycycle));
}

/*
 * Defines a PI controller with anti-windup mechanism.
 * WARNING: PI Gains depends on the motor. You need to find the proper gains for each motor.
 * but these gains will be very similar one to the other.
 */
int PIControllerMotor1(int desiredPosition){
  //You should prefer slow if you dont have a very good reason of going fast.
  float Kp = 0.1; //Fast: 0.4,  Slow: 0.1
  float Ki = 0.31; //Fast: 0.03, Slow: 0.31

  //We calculate and accumulate the position error
  float error = desiredPosition - finger1positionCounts;
  finger1lastError += error;

  //Anti integrator windup mechanism
  if(finger1lastError < -100) finger1lastError = -100;
  if(finger1lastError > 100)  finger1lastError  = 100;
  
  int command = Kp*error + Ki*finger1lastError;

  //If the command is within the deadzone, it will make an annoying noise
  //and wont move anyways.
  if(command > -31 && command < 32) command = 0; 

  finger1lastCommand = command;

  return command;
}

/*
 * Defines a PI controller with anti-windup mechanism.
 * WARNING: PI Gains depends on the motor. You need to find the proper gains for each motor.
 * but these gains will be very similar one to the other.
 */
int PIControllerMotor2(int desiredPosition){
  //You should prefer slow if you dont have a very good reason of going fast.
  float Kp = 0.1; //Fast: 0.4,  Slow: 0.1
  float Ki = 0.31; //Fast: 0.03, Slow: 0.31

  //We calculate and accumulate the position error
  float error = desiredPosition - finger2positionCounts;
  finger2lastError += error;

  //Anti integrator windup mechanism
  if(finger2lastError < -100) finger2lastError = -100;
  if(finger2lastError > 100)  finger2lastError  = 100;
  
  int command = Kp*error + Ki*finger2lastError;

  //If the command is within the deadzone, it will make an annoying noise
  //and wont move anyways.
  if(command > -31 && command < 32) command = 0; 

  finger2lastCommand = command;

  return command;
}

/*
 * This functions sets the PWM duty cycle (0-255) that should be
 * sent to the motor driver so it reaches the desired position.
 */
void controlLoopMotor1(int desiredPosition){
  int command = PIControllerMotor1(desiredPosition);
  setPWMMotor1(command);
}

/*
 * This functions returns the PWM duty cycle (0-255) that should be
 * sent to the motor driver so it reaches the desired position.
 */
byte controlLoopMotor2(int desiredPosition){
  int command = PIControllerMotor2(desiredPosition);
  setPWMMotor2(command);
}

//This function is executed once per sampling period
void executeControlStep(){
  if(PI_CONTROLLER_USED){
    controlLoopMotor1(motor1Command);
    controlLoopMotor2(motor2Command);
  }else{
    unsigned long now = millis();

    //If the motors are not moving and a command is pending, we need to react.
    if(motorsInMovement == false && (motor1Command > 0 || motor2Command > 0) ){
      speedCommandTimestamp = now;
    }
    
    unsigned long timeSinceLastStep  = (now > previousTime) ? (now - previousTime) : 0;
    unsigned long timeToGoMotor1     = motor1Command > 0 ? motor1Command : 0;
    unsigned long timeToGoMotor2     = motor2Command > 0 ? motor2Command : 0;
    
    //We potentially stop early so we dont overshoot by next step
    if(timeToGoMotor1 < timeSinceLastStep){
      setPWMMotor1(0);
      motor1Command = 0;
    }else{
      if(motor1Forward){
        setPWMMotor1(motor1SpeedCommandAmplitude);
      }else{
        setPWMMotor1(-1*motor1SpeedCommandAmplitude);
      }
    }

    if(timeToGoMotor2 < timeSinceLastStep){
      setPWMMotor2(0);
      motor2Command = 0;
    }else{
      if(motor2Forward){
        setPWMMotor2(motor2SpeedCommandAmplitude);
      }else{
        setPWMMotor2(-1*motor2SpeedCommandAmplitude);
      }
    }
    
    //If both motors reached their goal, the control can be stopped.
    if(timeToGoMotor1 < timeSinceLastStep && timeToGoMotor2 < timeSinceLastStep){
      motorsInMovement      = false;
      speedCommandTimestamp = 0;
    }else{
      motor1Command = motor1Command > timeSinceLastStep ? motor1Command - timeSinceLastStep : 0;
      motor2Command = motor2Command > timeSinceLastStep ? motor2Command - timeSinceLastStep : 0;
      motorsInMovement  = true;
    }
    
  }
}

/*
 * This sequence slowly opens the fingers and stop actuating the
 * appropriate motor once the finger has reached the homing position.
 * As the Arduino Uno ADC is 20 bits, analog values are between 0-1023
 * and therefore, the middleground is 512.
 */
void executeHomingSequence(){
  setPWMMotor1(-1*motor1SpeedCommandAmplitude);
  setPWMMotor2(-1*motor2SpeedCommandAmplitude);
  while(analogRead(FINGER1_HOMING) < 512 || analogRead(FINGER2_HOMING) < 512){
    if(analogRead(FINGER1_HOMING) > 512)
      setPWMMotor1(0);
    if(analogRead(FINGER2_HOMING) > 512)
      setPWMMotor2(0);
  }
  //Serial.println("Homing sequence done.");
}

//This function reads the command buffer and execute the position commands.
void processCommand(){
  //Command format verification
  if(commandBuffer[0] == '1'   && commandBuffer[1] == ':'    && isDigit(commandBuffer[2])  &&
     isDigit(commandBuffer[3]) && isDigit(commandBuffer[4])  && isDigit(commandBuffer[5])  &&
     commandBuffer[6] == ' '   && commandBuffer[7] == '2'    && commandBuffer[8] == ':'    &&
     isDigit(commandBuffer[9]) && isDigit(commandBuffer[10]) && isDigit(commandBuffer[11]) && isDigit(commandBuffer[12])){
     //Interpret the position commands
     int ASCII_Zero = 48;
     
     int m1_thousands = (int)commandBuffer[2] - ASCII_Zero;
     int m1_hundreds  = (int)commandBuffer[3] - ASCII_Zero;
     int m1_decades   = (int)commandBuffer[4] - ASCII_Zero;
     int m1_units     = (int)commandBuffer[5] - ASCII_Zero;
     
     int m2_thousands = (int)commandBuffer[9] - ASCII_Zero;
     int m2_hundreds  = (int)commandBuffer[10] - ASCII_Zero;
     int m2_decades   = (int)commandBuffer[11] - ASCII_Zero;
     int m2_units     = (int)commandBuffer[12] - ASCII_Zero;
     
     motor1Command = 1000*m1_thousands + 100*m1_hundreds + 10*m1_decades + m1_units;
     motor2Command = 1000*m2_thousands + 100*m2_hundreds + 10*m2_decades + m2_units;

     if(PI_CONTROLLER_USED == false){
        //If the command is pair, it means forward
        //If the command is odd , it means backward
        if(m1_units % 2 == 0)
          motor1Forward = true;
        else
          motor1Forward = false;
          
        if(m2_units % 2 == 0)
          motor2Forward = true;
        else
          motor2Forward = false;
     }    
  } 
}


void setup() {
  //Initialize the Serial interface
  Serial.begin(115200);
  
  //Configure input/output pins, all pins are INPUT by default
  pinMode(FINGER1_HOMING, INPUT_PULLUP);
  pinMode(FINGER2_HOMING, INPUT_PULLUP);
  pinMode(FINGER1_DRIVER_DIR, OUTPUT);
  pinMode(FINGER2_DRIVER_DIR, OUTPUT);
  pinMode(FINGER1_DRIVER_PWM, OUTPUT);
  pinMode(FINGER2_DRIVER_PWM, OUTPUT);
  
  //Attach Interrupts, on Arduino Uno only digital pins #2 and #3 can be used with hardware interrupts
  attachInterrupt(digitalPinToInterrupt(FINGER1_ENCODER_B), interruptHandlerMotor1ChannelB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FINGER2_ENCODER_B), interruptHandlerMotor2ChannelB, CHANGE);

  //If the PI position control is used, then a homing sequence must be executed to zero out the position.
  //If the speed control is used instead then the samplingPeriod must be as small as possible.
  if(PI_CONTROLLER_USED){
     //Initially, we dont know the position of the fingers and need to do the homing.
    executeHomingSequence();
    samplingPeriod = 10;//10mS
  }else{
    samplingPeriod = 5; //5mS
  }

  
  //Start timestamp
  previousTime = millis();
}


void loop() {
  //If the fingers are in the process of moving, then we dont want to introduce
  //any latencies by processing commands.
  if (!motorsInMovement && Serial.available() > 0){
    //Accumulate serial readings until a command is fully received.
    //A command is defined by this pattern where the position MUST be on 4 bytes:
    //1:0000 2:2048\n
    
    char incomingByte = Serial.read();
    
    //If the buffer is not full yet, accumulate.
    if(commandBufferIndex < commandBufferSize-1){
      commandBuffer[commandBufferIndex] = incomingByte;
      commandBufferIndex++;
    }else{
      //The buffer is full, we can process the command and empty the buffer.
      processCommand();
      commandBufferIndex = 0;
      for(int i = 0; i < commandBufferSize; i++)
        commandBuffer[i] = 0;
    }
  }
  
  //This loop is executed periodically to do the position control
  if(millis() - previousTime > samplingPeriod){
    //Execute one step of our closed-loop control system
    executeControlStep();
    
    //The serial interface is configured to transfer at 11.52 kilobits per second.
    //This transmit 12*8=96 bits and does 2 ADC readings. Should take approx. 96/11.520 mS + 2*50uS = 8.43ms
    
    sendInteger(finger1positionCounts);
    Serial.write(',');
    sendInteger(finger2positionCounts);
    Serial.write(',');
    transmitPressureReadings();
    
    previousTime = millis();
  }

}
