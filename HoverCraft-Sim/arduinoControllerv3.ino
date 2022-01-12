//Setup Basic Components
int liftState, thrustState;
float thrustThrottle, servoPosition;

//Finite State Machine Object
  //Unstuck/Start - 0
  //Going Straight - 1
  //Turn Left - 2
  //Turn Right - 3
  //Stuck - 4
struct State {
  int currState, prevState;
  float stateTime;
};

//Setup State Machine
State FSM;

void setup() {
  //Initialize Default Values
  liftState = 1;
  thrustState = 1;
  thrustThrottle = 1.0;
  servoPosition = 0.0;

  //Initialize FSM to Default
  FSM = {0, 1, 0};
  
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200); 
  Serial.setTimeout(2);
}

//Wait for Sensor Data
void serialWait() {
  while(Serial.peek() != 's') {
    char t = Serial.read();
    // Turn On LED, High
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1);
    // Turn Off LED, Low                      
    digitalWrite(LED_BUILTIN, LOW);
    delay(1);
  }
  //Discard the 's' char
  char t = Serial.read();
}

//Move Straight
void goStraight(float angle) {
  //Keep the Thrust Fan On and Servo Straight (0)
  liftState = 1;
  thrustState = 1;
  thrustThrottle = 1.0;
  servoPosition = angle; // PI/8 to Counteract Torque From Lift Fan

  //Set Current State to Straight
  FSM.currState = 1;
}

//Turn Left
void startTurnLeft(float angle){
  //Keep the Thrust Fan On, Slow Down During Turn, and Servo Turns Right to Move Left (Pi/2)
  thrustState = 1;
  thrustThrottle = 0.5;
  servoPosition = angle; //PI/2;

  //Set Current State to Left
  FSM.currState = 2;
}

//Turn Right
void startTurnRight(float angle){
  //Keep the Thrust Fan On, Slow Down During Turn, and Servo Turns Left to Move Right (-Pi/2)
  thrustState = 1;
  thrustThrottle = 0.5;
  servoPosition = angle; //-1*PI/2

  //Set Current State to Right
  FSM.currState = 3;
}


void setStateTime(int time) {
  //Set Time to Start of State
  if(FSM.currState == FSM.prevState && FSM.stateTime != 0)
      FSM.stateTime = time;
  //Time Reset: Not Stuck (-1)
  else
      FSM.stateTime = -1;
}

//Hovercraft is Stuck - Get UnStuck
boolean isStuck(int time) {
  if(FSM.currState != FSM.prevState || FSM.stateTime <= 0)
    return false;

  //Same State for Over 5 Seconds, Considered Stuck
  if(time - FSM.stateTime >= 5) 
    return true;

  return false;
}

void getUnstuck(boolean tryRight) {
  //Try to Turn Left Or Right
  if(tryRight)
    startTurnRight(-1/4*PI);
  else
    startTurnLeft(-1/4*PI);
  
  //Once Unstuck Change Both
  FSM.prevState = 4;
  FSM.currState = 0;
}

void loop() {  
  //Read Serial Port until Sensor Data Sent
  serialWait();
  
  //Receive and Interpret the data String from CoppeliaSim
  float simTime = Serial.parseFloat();
  int resultF = Serial.parseInt();
  float distanceF = Serial.parseFloat();
  int resultR = Serial.parseInt();
  float distanceR = Serial.parseFloat();

  //Time 
  int timeNow = millis()/1000;

  liftState = 1;
  thrustState = 0;
  thrustThrottle = 0;
  servoPosition = 0;
  
  //Sets State Time
  setStateTime(timeNow);
/*
  //Check If Hovercraft is Stuck
  if(isStuck(timeNow)) {
    //Compare Distance Front and Right
    if(distanceF > distanceR)
      goStraight();
    //Turn Left if Distance Right is < 0.2m
    else if(distanceR < 0.2)
      getUnstuck(false);
    //Otherwise Turn Right
    else
      getUnstuck(true);
  } */

  //Case 1 - No Wall In Front
  if(resultF == 0) {
    goStraight(0);
  }
  //Cases 2, 3 - Wall In Front
  else {
    //Relatively Far - Hasn't Reached Intersection
    if(distanceF >= 0.6) {
      //Wall On Right
      if(resultR == 1 && distanceR < 0.25) {
        startTurnLeft(1/2*PI);
      } else if(resultR == 0 || distanceR > 0.3) {
        startTurnRight(-1/2*PI);
      } else {
        goStraight(PI/4);
      }
    //Start Turn
    } else if(distanceF < 0.4) {
      //No Wall Or Wall Detected Further Away - Turn Right
      if(resultR == 0 || distanceR > 0.2) {
        startTurnRight(-1/2*PI);
      //Wall Present In Front and Right too Close - Turn Left
      } else if(distanceF < 0.3 && distanceR < 0.2) {
        startTurnLeft(1/2*PI);
      //Otherwise Continue Straight
      } else {
        startTurnLeft(1/2*PI);
      }
    }
  }
  
/*
  //No Wall Detected in Front - Can Continue Straight or Turn
  if(resultF == 0) {
    goStraight();
    
  //Wall Detected in Front - Must Turn to Find Opening
  } else {
    //Distance In Front < 0.05 m
    if(distanceF <= 0.05) {
      //TODO: Probably Stuck - Rotate Fan Either Left or Right to Get Unstuck
      //getUnstuck()
      
      //No Wall on Right - Rotate Right
      if(resultR == 0) {
        startTurnRight(-1/4*PI);
      //Wall on Right and Too Close - Rotate Left
      } else if(resultR == 1 && distanceR <= 0.1) {
        startTurnLeft(1/2*PI);
      //Wall on Right but Further Away - Rotate Right
      } else if(resultR == 1 && distanceR >= 0.25) {
        startTurnRight(-1*PI);
      //Wall on Right Detected - Try Left
      } else {
        startTurnLeft(1/4*PI);
      }   
    //Distance In Front < 0.3 m - Rotate
    } else if(distanceF <= 0.3) {
      //Slow down thrust fan
      //thrustState = 1;
      
      if(FSM.prevState > 1)
        goStraight();
      //No Wall on Right - Rotating Right
      else if(resultR == 0) {
        startTurnRight(-1/2*PI);
      //Wall on Right and Too Close - Rotate Left
      } else if(resultR == 1 && distanceR <= 0.1) {
        startTurnLeft(1/2*PI);
      //Wall on Right but Further Away - Rotate Right
      } else if(resultR == 1 && distanceR >= 0.25) {
        startTurnRight(-1*PI);
      //Wall on Right Detected - Try Left
      } else {
        startTurnLeft(1/4*PI);
      }  

      //thrustThrottle = 0;
   //Distance In Front < 0.5 m - Start Rotating, Approaching Wall
    } else if(distanceF <= 0.5) {
      //Turn off thrust fan
      //thrustState = 0;
      //thrustThrottle = 0;
      if(FSM.prevState > 1)
        goStraight();
      //No Wall on Right - Rotating Right
      else if(resultR == 0) {
        startTurnRight(-1/4*PI);
      //Wall on Right and Too Close - Rotate Left
      } else if(resultR == 1 && distanceR <= 0.1) {
        startTurnLeft(1/4*PI);
      //Wall on Right but Further Away - Rotate Right
      } else if(resultR == 1 && distanceR >= 0.25) {
        startTurnRight(-1/4*PI);
      //Wall on Right Detected - Try Left
      } else {
        startTurnLeft(1/4*PI);
      }  
    //Distance In Front > 0.5 - Keep Going Straight
    } else {
      goStraight();
    }
  }
  */
  
  //Set Previous State to Current State
  FSM.prevState = FSM.currState;
  
  //Send Data to CoppeliaSim
  // Format {simTime, timeNow, liftState, thrustState, thrustThrottle, servoPosition} 
  Serial.print(simTime);
  Serial.write(",");
  Serial.print(timeNow);
  Serial.write(",");
  Serial.print(liftState);
  Serial.write(",");
  Serial.print(thrustState);
  Serial.write(",");
  Serial.print(thrustThrottle);
  Serial.write(",");
  Serial.print(servoPosition);
  Serial.write("\r\n");
}
