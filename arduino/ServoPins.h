// blue
const int GRIPPER_PIN = 2;

// green
const int WRIST_ROLL_PIN = 3;

// yellow, yellow
const int WRIST_PAN_PINL = 4;
const int WRIST_PAN_PINR = 5;

// orange
const int ELBOW_ROLL_PIN = 6;

// blue, blue
const int ELBOW_PAN_PINL = 7;
const int ELBOW_PAN_PINR = 8;

// green
const int SHOULDER_ROLL_PIN = 9;

// yellow, yellow
const int SHOULDER_PAN_PINL = 10;
const int SHOULDER_PAN_PINR = 11;

const int N = 7;

const int DOFS[N] = {SHOULDER_PAN_PINL, SHOULDER_ROLL_PIN,
                     ELBOW_PAN_PINL, ELBOW_ROLL_PIN,
                     WRIST_PAN_PINL, WRIST_ROLL_PIN,
                     GRIPPER_PIN};
