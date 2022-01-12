/*
 * Nicholas Kawwas,
 * Jackey Weng,
 * Karine Chatta, 
 * Shuang Luo,
 * Salma Alawani
 * Group 4: Autonomous Hovercraft Using Vision Sensors
 */

//Define Constants
#define BAUDRATE 115200
#define SERIAL_DELAY 90
#define IMG_SENSOR_YDIM 32

//Creating Finite State Machine to Determine Current and Next Moves in Different Cases 
//Enumeration of All Possible States
enum HC_state {
  START,
  ADJUST_HC,
  ADJUST_HC_RIGHT,
  ADJUST_HC_LEFT,
  GO_STRAIGHT,
  SLOW_DOWN,
  TURN_LEFT_ONE,
  TURN_LEFT_TWO,
  TURN_LEFT_THREE,
  TURN_RIGHT_ONE,
  TURN_RIGHT_TWO,
  TURN_RIGHT_THREE,
  WORKS,
  PAUSE,
  STUCK
};

//Defined Global Variables
// Data to Send Back to CoppeliaSim
int liftState, thrustState;
float thrustThrottle, servoPosition;

// Next State Logic
uint8_t currState, nextState;
float stateTime;

// Calculate Time per State
unsigned long interval, previousTime;
bool turnRight = true;

//Store Image Array from Vision Sensor
unsigned long img[IMG_SENSOR_YDIM];

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(BAUDRATE);
  Serial.setTimeout(1);

  //Initialize Default Values
  liftState = 1;
  thrustState = 1;
  thrustThrottle = 1;
  servoPosition = 0;

  currState = START;
  nextState = START;

  stateTime = 0;
  previousTime = 0;
  interval = 0;
}

void serialWait() {
  //Turn LED on - High Voltage Level
  digitalWrite(LED_BUILTIN, HIGH);

  //Read Image on Serial Bus
  while (Serial.peek() != 's') {
    //Read to Discard i Then Store the Image String as Array
    if (Serial.peek() == 'i') {
      char t = Serial.read();
      delayMicroseconds(6 * SERIAL_DELAY);
      for (int j = IMG_SENSOR_YDIM - 1; j >= 0; j--) {
        char longarr[8];
        for (int i = 0; i < 8; i++) {
          longarr[i] = Serial.read();
          if (Serial.available() < 4) {
            delayMicroseconds(SERIAL_DELAY);
          }
        }
        unsigned long hexval = strtoul((const char * ) longarr, 0, 16);
        img[j] = hexval;
      }

      //Read to Discard @ Then Print Out Image to Check Properly Received    
      if (Serial.peek() == '@') {
        char t = Serial.read(); //discard the @
        Serial.write(0x23);
        Serial.write(0x23);
        Serial.write("\r\n");
        Serial.write("Printing image...");
        Serial.write("\r\n");
        for (int b = 0; b < IMG_SENSOR_YDIM; b++) {
          for (int c = 0; c < 32; c++) {
            Serial.print(bitRead(img[b], c));
            Serial.write(" ");
          }
          Serial.write("\r\n");
        }
        Serial.write(0x40);
        Serial.write(0x40);
        Serial.write("\r\n");
      }
    }

    //Read to Discard s, Turn LED Off (Low Voltage) Then Exit Serial Wait     
    if (Serial.peek() == 's') {
      char t = Serial.read();
      digitalWrite(LED_BUILTIN, LOW);
      return;
    }

    if (Serial.available() > 2) {
      char t = Serial.read();
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}

//Empties Serial Port
void serialEmpty() {
  while (Serial.available())
    char t = Serial.read();
}

//Reads Hex Value from Image Array Given X and Y Coordinates
short readImage(short imgX, short imgY) {
  return bitRead(img[imgY], imgX);
}

//Gets Relative Distance From the Wall to Hovercraft
// Read Pixels from the Middle of the Image from Top to Bottom
int getDistance() {
  for (int i = 0; i < 31; i++)
    if (readImage(15, i) == 1)
      return i;
}

int getLeftWall() {
  for (int i = 0; i < 31; i++)
    if (readImage(3, i) == 1)
      return i;
}

int getRightWall() {
  for (int i = 0; i < 31; i++)
    if (readImage(28, i) == 1)
      return i;
}

/*
 * Vision Sensor - Front Wall:
 * 17 ~ No Wall
 * 11 ~ 1.00 m
 * 10 ~ 0.80 m
 * 9 ~ 0.75 m
 * 8 ~ 0.64 m
 * 7 ~ 0.53 m
 * 6 ~ 0.50 m
 * 5 ~ 0.43 m
 * 4 ~ 0.40 m
 * 3 ~ 0.38 m
 * 2 ~ 0.35 m
 * 1 ~ 0.33 m
 * 0 ~ 0.30 m
 * 
 * Vision Sensor - Side Walls:
 * Side == Front -> No Wall on Side
 *    7 ~ Centered
 *  < 7 ~ Close to Specific Side Wall
 *  > 7 ~ Far from Specific Side Wall
 */

void loop() {
  //Read Serial Port until Sensor Data is Sent
  //Then Read Image Data if Sent From CoppeliaSim
  serialWait();

  //Receive and Interpret the Data String from CoppeliaSim
  float simTime = Serial.parseFloat();

  //Time since Arduino is Last Reset
  long timeNow = millis();

  //Calculate Sensor Data With Image from Front Vision Sensor
  //Determine Whether There is a Front Wall Using Pixels
  //Compare Pixel Value to Floor Value (17)
  int distanceF = getDistance();
  int resultF = distanceF < 17 ? 1 : 0;

  //Determine Whether There is a Left Wall Using Pixels
  //Compare Pixel Value to Front Wall Value
  int distanceL = getLeftWall();
  int resultL = distanceL != distanceF ? 1 : 0;

  //Determine Whether There is a Right Wall Using Pixels
  //Compare Pixel Value to Front Wall Value
  int distanceR = getRightWall();
  int resultR = distanceR != distanceF ? 1 : 0;

  //Update State to Next State 
  currState = nextState;

  //Sensor Data Analysis and Control Algorithm Here:
  //Finite State Machine & Next State Calculation
  finite_state_machine(distanceF, resultF, distanceR, resultR, distanceL, resultL, timeNow);

  //Empty Serial Port for Next Image
  serialEmpty();

  //Send Data to CoppeliaSim
  // Format {simTime, timeNow, liftState, thrustState, thrustThrottle, servoPosition, currState} 
  Serial.print(simTime);
  Serial.write(",");
  Serial.print(timeNow / 1000);
  Serial.write(",");
  Serial.print(liftState);
  Serial.write(",");
  Serial.print(thrustState);
  Serial.write(",");
  Serial.print(thrustThrottle);
  Serial.write(",");
  Serial.print(servoPosition);
  Serial.write(",");
  Serial.print(currState);
  Serial.write(",");
  Serial.print(distanceF);
  Serial.write(",");
  Serial.print(distanceL);
  Serial.write(",");
  Serial.print(distanceR);
  Serial.write("\r\n");
}

//Determine Current and Next Move Based On Sensor Data - What Hovercraft Should Do in Different States
void finite_state_machine(int distanceF, int detectF, int distanceR, int detectR, int distanceL, int detectL, int timeNow) {
  switch (currState) {
    
  //Starting State
  case START:
    HC_action(1, 1, -PI/4);
    nextState = ADJUST_HC;

    previousTime = timeNow;
    set_interval(5000);
    break;

  //Course Correction to Keep Hovercraft Moving Straight
  case ADJUST_HC:
    HC_action(1, 1, -0.47);

    //When Front Wall is Detected
    if (timeNow - previousTime >= interval || (timeNow - previousTime >= 3*interval/4 && distanceF <= 2)) {
      //Enter Turn Right State 
      if (turnRight && distanceF <= 8) {
        HC_action(1, 0.9, -PI/2);
        previousTime = timeNow;
        set_interval(2000);
        //HC_stop();
        nextState = TURN_RIGHT_ONE;
      //Enter Turn Left State
      } else if (!turnRight && distanceF <= 5) {
        HC_action(1, 0.9, PI/3);
        nextState = TURN_LEFT_ONE;
      }
    }
    //Place Evenly in Center
     else if (detectR == 1 && detectL == 1) {
      //Adjust Hovercraft to Center - Too Close to Right
      if (distanceR > distanceL) {
        HC_action(1, 1, -0.62);
      }
      //Adjust Hovercraft to Center - Too Close to Left
      else if (distanceL > distanceR)
        HC_action(1, 1, -0.10);
     }

     else if(distanceR == 17) {
      HC_action(1, 1, -0.55);
     }

     else if(distanceL == 17) {
      HC_action(1, 1, 0.30);
     }

     /*
     //Too Much to the Left
     else if(detectR == 0 && detectL == 1) {
      HC_action(1, 1, -0.5);
     }
     //Too Much to the Right 
     else if(detectR == 1 && detectL == 0) {
      HC_action(1, 1, -0.3);
     }
     //Too Close to the Right
     else if(distanceL < distanceF && distanceF <= distanceR) {
      HC_action(1, 1, -0.8);
     }
    //Too Close to the Left
     else if(distanceL > distanceF && distanceF >= distanceR) {
      HC_action(1, 1, 0);
     }*/
     
    //Direct Thrust To Go Over Over Bumps
    else if (distanceF == 18)
      HC_action(1, 1, 0);
    
    break;

    //First Right Turn
  case TURN_RIGHT_ONE:
    HC_action(1, 1, -PI / 2);

    //When Front Wall Detected At End of Intersection and No Right Wall,
    //Move On to Next Turn Right State of Moving Straight Before Turning Again
    if ((distanceF >= 7 || distanceL == 0) && timeNow - previousTime >= interval) {
      previousTime = timeNow;
      HC_action(1, 1, -PI / 10);
      nextState = TURN_RIGHT_TWO;
    }
    break;

    //Move Forward After First Turn
  case TURN_RIGHT_TWO:
    HC_action(1, 1, -0.20);

    //When Front Wall is Less Than 50cm
    //Start Turning Again //&& detectR == 0
    if (distanceF <= 4) {
      HC_action(1, 1, -PI / 2);
      nextState = TURN_RIGHT_THREE;
    }
    break;

    //Finish Right Turn
  case TURN_RIGHT_THREE:
    HC_action(1, 0.9, -PI / 2);

    //When No Walls Are Detected
    //Start Adjusting the Hovercraft Again to Continue Straight
    //1- No Front Wall and Side Walls (Straightaway)
    //2- No Front Wall and Only Right Wall (Before End of Turn)
    //3- Allow For Enough Time to Elapse
    if(distanceR - distanceL >= 9 || distanceF >= 8) {
      HC_stop();
      turnRight = false;
      previousTime = timeNow;
      set_timer(timeNow);
      set_interval(7000);
      nextState = ADJUST_HC;
    }
    break;

//First Left Turn
  case TURN_LEFT_ONE:
    HC_action(1, 1, PI/4.4);
    //When Front Wall Detected At End of Intersection,
    //Move On to Next Turn Left State of Moving Straight Before Turning Again
     if(distanceF >= 10 || distanceL >= 17) {
      HC_stop();
       set_timer(timeNow);
      set_interval(9000);
      turnRight = true;
       nextState = ADJUST_HC;
    }
     else if(distanceF >= 7) {
      HC_stop();
      //previousTime = timeNow;
      //set_interval(1500);
      nextState = TURN_LEFT_TWO;
    }
    break;

    //Move Forward After First Turn
  case TURN_LEFT_TWO:
    HC_action(1, 1, -0.47);

    //When Front Wall is Less Than 42cm
    //Start Turning Again
    if (distanceF <= 4) {
      HC_action(1, 1, PI / 3);
      nextState = TURN_LEFT_THREE;
    }
    break;

    //Finish Left Turn
  case TURN_LEFT_THREE:
    HC_action(1, 0.9, PI/4.4);

    //When Right Wall is Detected and Front Wall is Further Away
    //Start Adjusting the Hovercraft Again to Continue Straight
    if (distanceF >= 10 || (distanceL == 17 && distanceF >= 4)) {
      previousTime = timeNow;
      set_timer(timeNow);
      set_interval(6000);
      turnRight = true;
      HC_stop();
      nextState = ADJUST_HC;
    }
    break;
  }
}

//Give the Hovercraft an Action with the Parameter for Throttle and Angle 
//e.g. Straight, turn left, and turn right 
void HC_action(int lift, float throttle, float angle) {
  liftState = lift;
  thrustState = 1;
  thrustThrottle = throttle;
  servoPosition = angle;
}

//Turn Off Hovercraft Lift and Thrust Fans
void HC_stop() {
  liftState = 0;
  thrustState = 0;
  thrustThrottle = 1.0;
  servoPosition = 0;
}

//Set Previous Time to Current Time to Mark End of Turn and Start of Adjustment Period
void set_timer(int timeNow) {
  previousTime = timeNow;
}

//Set Time Interval to Adjust Hovercraft
void set_interval(float t) {
  interval = t;
}
