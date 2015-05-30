#include <Servo.h>
#include "ServoPins.h"
#include "JointLimits.h"
#include "StatusCodes.h"

int status;
Servo gripper, wristRoll, wristPanL, wristPanR, elbowRoll, elbowPanL, elbowPanR, shoulderRoll, shoulderPanL, shoulderPanR;

void setup(){
  Serial.begin(9600);
  Serial.println("Started...");
  attachServos();
  //homePos();
  status = READY;
}

void loop(){
  if(status == READY){
      while(Serial.available()){
        int m = Serial.parseInt();
        char c = Serial.read();
        int t = Serial.parseInt();
        c = Serial.read();
        commandServo(DOFS[m],t);
      }
  }
}
      
////////////////////////////////////////////////////
void commandServo(int m, int p) {
  status = BUSY;
  switch (m) {

    case GRIPPER_PIN:
          if(p == 0){
            openGripper();
          } else if(p == 1){
            closeGripper();
          } else if (p >= GRIPPER_OPEN && p <= GRIPPER_CLOSED) {
            gripper.write(p);
          }
          break;


    case WRIST_ROLL_PIN:
      if (p >= WRIST_ROLL_MIN && p <= WRIST_ROLL_MAX) {
        wristRoll.write(p);
      }
      break;


    case WRIST_PAN_PINL:
      if (p >= WRIST_PAN_MIN && p <= WRIST_PAN_MAX) {
        wristPanL.write(p);
        wristPanR.write(180 - p);
      }
      break;


    case ELBOW_ROLL_PIN:
      if (p >= ELBOW_ROLL_MIN && p <= ELBOW_ROLL_MAX) {
        elbowRoll.write(p);
      }
      break;


    case ELBOW_PAN_PINL:
      if (p >= ELBOW_PAN_MIN && p <= ELBOW_PAN_MAX) {
        elbowPanL.write(p);
        elbowPanR.write(180 - p);
      }
      break;


    case SHOULDER_ROLL_PIN:
      if (p >= SHOULDER_ROLL_MIN && p <= SHOULDER_ROLL_MAX) {
        shoulderRoll.write(p);
      }
      break;
  
  case SHOULDER_PAN_PINL:
      if (p >= SHOULDER_PAN_MIN && p <= SHOULDER_PAN_MAX) {
        shoulderPanL.write(p);
        shoulderPanR.write(180-p);
      }
      break;

  default:
      Serial.print("No servo on pin ");
      Serial.println(m);
      break;
  }
  status = READY;
}

void openGripper() {
  gripper.write(GRIPPER_OPEN);
}

void closeGripper() {
  gripper.write(GRIPPER_CLOSED + 30);
}


void attachServos() {
  gripper.attach(GRIPPER_PIN);
  wristRoll.attach(WRIST_ROLL_PIN);
  wristPanL.attach(WRIST_PAN_PINL);
  wristPanR.attach(WRIST_PAN_PINR);
  elbowRoll.attach(ELBOW_ROLL_PIN);
  elbowPanL.attach(ELBOW_PAN_PINL);
  elbowPanR.attach(ELBOW_PAN_PINR);
  shoulderRoll.attach(SHOULDER_ROLL_PIN);
  shoulderPanL.attach(SHOULDER_PAN_PINL);
  shoulderPanR.attach(SHOULDER_PAN_PINR);
}

void homePos(){
  openGripper();
  commandServo(DOFS[0], 90);
  commandServo(DOFS[1], 90);
  commandServo(DOFS[2], 120);
  commandServo(DOFS[3], 70);
  commandServo(DOFS[4], 90);
  commandServo(DOFS[5], 90);
}
