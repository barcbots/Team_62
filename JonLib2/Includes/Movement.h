#pragma systemFile
#warning "JonLib2: Movement"

void setLeftWheelSpeed (int speed = 127);
void setRightWheelSpeed (int speed = 127);

int leftEncoderCurve (int target);
int rightEncoderCurve (int target);

void setWheelSpeed (int leftWheelSpeed = 127, int rightWheelSpeed = 127) {
	setLeftWheelSpeed(leftWheelSpeed);
	setRightWheelSpeed(rightWheelSpeed);
}

void setWheelSpeed (int speed = 127) {
	setWheelSpeed(speed, speed);
}

void spin (int speed = 127) {
	setWheelSpeed(-speed, speed);
}

void timeDrive(int time, int leftPower = 127, int rightPower = 127) {
	setWheelSpeed(leftPower, rightPower);
	delay(time);
}

void timeDriveStop(int time, int leftPower = 127, int rightPower  = 127) {
	timeDrive(time, leftPower, rightPower);
	setWheelSpeed(0,0);
}

void tankDrive(int leftPower, int rightPower, int deadbands) {
	setWheelSpeed(
		abs(leftPower)<deadbands?0:leftPower,
		abs(rightPower)<deadbands?0:rightPower
	);
}

typedef struct {
	pid left;
	pid right;
	tSensors leftEncoder;
	tSensors rightEncoder;
	tSensors gyroscope;
	float gyroP;
	int maxSpeed;
} drivebase;

void initPIDDrivebase (drivebase *controller, tSensors leftEncoder, tSensors rightEncoder, float kP,  float kI, float kD, int threshold = 10, int integralLimit = -1, int slewRate = 10) {
	initPIDController(controller->left,  kP, kI, kD, threshold, integralLimit, slewRate);
	initPIDController(controller->right, kP, kI, kD, threshold,  integralLimit, slewRate);
	controller->leftEncoder = leftEncoder;
	controller->rightEncoder = rightEncoder;
	controller->gyroP = 0.0;
}

void initPIDDrivebase (drivebase *controller, tSensors leftEncoder, tSensors rightEncoder, tSensors gyroscope, float kP,  float kI, float kD, int threshold = 10, int integralLimit = -1, int slewRate = 10, float gyroP = 0.0) {
	initPIDDrivebase(controller,  leftEncoder,  rightEncoder,  kP,  kI, kD, threshold, integralLimit, slewRate);
	controller->gyroscope = gyroscope;
	controller->gyroP =  gyroP;
}

int getGyroCrossTrackError(drivebase *controller, int target) {
	return (target - SensorValue[controller->gyroscope])*controller->gyroP;
}

bool drivebasePIDAuto(drivebase *controller, bool useGyro) {
	pid *left = controller->left;
	pid *right = controller->right;

	clearIntegral(left);
	clearIntegral(right);

	int gyroTarget;
	if(controller->gyroP!=0) {
		gyroTarget = SensorValue[controller->gyroscope];
		writeDebugStream("GYRO USED ");
	}

	writeDebugStream("INTENDED: (%d, %d) ", left->target, right->target);

	long lastUpdate = nPgmTime;
	int lastLeftError = -1, lastRightError = -1;

	do {
		if(useGyro) {
			int gyroSway = getGyroCrossTrackError(controller, gyroTarget);
			setWheelSpeed(
				limit(updatePIDController(left, controller->leftEncoder)-gyroSway, controller->maxSpeed),
				limit(updatePIDController(right,  controller->rightEncoder)+gyroSway, controller->maxSpeed)
			);
		} else {
			setWheelSpeed(
				limit(updatePIDController(left, controller->leftEncoder), controller->maxSpeed),
				limit(updatePIDController(right, controller->rightEncoder), controller->maxSpeed)
			);
		}

		if(abs(left->error)<(abs(lastLeftError)-3)){
			lastUpdate = nPgmTime;
		}

		if(abs(right->error)<(abs(lastRightError)-3)){
			lastUpdate = nPgmTime;
		}

		lastLeftError = left->error;
		lastRightError = right->error;

		if((nPgmTime-lastUpdate)>MOVE_TIMEOUT) {
			setWheelSpeed(0);
			writeDebugStream("TIMEOUT ");
			writeDebugStreamLine("ACT: (%d,%d)", SensorValue[controller->leftEncoder],SensorValue[controller->rightEncoder]);
			return false;
		}

		if(abs(left->error)>=left->threshold || abs(right->error)>=right->threshold)
			clearTimer(T4);

		delay(25);

	} while(time1[T4]<150);

	setWheelSpeed(0);
	writeDebugStreamLine("ACT: (%d,%d)", SensorValue[controller->leftEncoder],SensorValue[controller->rightEncoder]);
	return true;
}

//todo - make these timeout + return false
void addDrivebaseTargetPID(drivebase *controller,  int leftTarget, int rightTarget, int maxSpeed = 127) {
	controller->maxSpeed = maxSpeed;
	addTarget(controller->left, leftEncoderCurve(leftTarget));
	addTarget(controller->right, rightEncoderCurve(rightTarget));
}

void addDrivebaseTargetPID(drivebase *controller, int target) {
	addDrivebaseTargetPID(controller, target, target);
}

bool addDrivebaseTargetPIDAuto(drivebase *controller, int leftTarget, int rightTarget, int maxSpeed = 127) {
	addDrivebaseTargetPID(controller, leftTarget, rightTarget, maxSpeed);
	return drivebasePIDAuto(controller, leftTarget==rightTarget);
}

bool addDrivebaseTargetPIDAuto(drivebase *controller, int target) {
	return addDrivebaseTargetPIDAuto(controller, target, target);
}

void setDrivebaseTargetPID(drivebase *controller,  int leftTarget, int rightTarget, int maxSpeed = 127) {
	controller->maxSpeed = maxSpeed;
	setTarget(controller->left, leftEncoderCurve(leftTarget));
	setTarget(controller->right, rightEncoderCurve(rightTarget));
}

void setDrivebaseTargetPID(drivebase *controller, int target) {
	setDrivebaseTargetPID(controller, target, target);
}

bool setDrivebaseTargetPIDAuto(drivebase *controller, int leftTarget, int rightTarget, int maxSpeed = 127) {
	setDrivebaseTargetPID(controller, leftTarget, rightTarget, maxSpeed);
	return drivebasePIDAuto(controller, false);
}

bool setDrivebaseTargetPIDAuto(drivebase *controller, int target) {
	return setDrivebaseTargetPIDAuto(controller, target, target);
}
