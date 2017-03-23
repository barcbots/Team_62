#pragma systemFile

typedef struct {
	pid controller;
	tSensors sensor;
} gyroscope;

void initPIDGyroscope (gyroscope *gyroController, tSensors sensor, float kP,  float kI, float kD, int threshold = 10, int integralLimit = -1, int slewRate = 10) {
	pid *controller = gyroController->controller;
	initPIDController(controller, kP, kI, kD, threshold, integralLimit, slewRate);
	gyroController->sensor = sensor;
}

bool leftSwingTurnGyroPID (gyroscope *gyroController) {
	pid *controller = gyroController->controller;

	long lastUpdate = nPgmTime;

	do {
		setLeftWheelSpeed(updatePIDController(controller, gyroController->sensor));

		if(abs(controller->error)>controller->threshold*THRESHOLD_COEFF)
			lastUpdate = nPgmTime;

		if((nPgmTime-lastUpdate)>MOVE_TIMEOUT) {
			setWheelSpeed(0);
			return false;
		}

		delay(25);
	} while(abs(controller->error)>controller->threshold);

	setWheelSpeed(0);
	return true;
}

bool rightSwingTurnGyroPID (gyroscope *gyroController) {
	pid *controller = gyroController->controller;

	long lastUpdate = nPgmTime;

	do {
		setRightWheelSpeed(updatePIDController(controller, gyroController->sensor));

		if(abs(controller->error)>controller->threshold*THRESHOLD_COEFF)
			lastUpdate = nPgmTime;

		if((nPgmTime-lastUpdate)>MOVE_TIMEOUT) {
			setWheelSpeed(0);
			return false;
		}

		delay(25);
	} while(abs(controller->error)>controller->threshold);

	setWheelSpeed(0);
	return true;
}

bool pointTurnGyroPID (gyroscope *gyroController) {
	pid *controller = gyroController->controller;

	long lastUpdate = nPgmTime;

	do {
		spin(updatePIDController(controller, gyroController->sensor));

		if(abs(controller->error)<=(abs(controller->lastError)-20))
			lastUpdate = nPgmTime;

		if((nPgmTime-lastUpdate)>MOVE_TIMEOUT) {
			setWheelSpeed(0);
			writeDebugStreamLine("TIMEOUT");
			return false;
		}

		if(abs(controller->error)>controller->threshold)
			clearTimer(T4);

		delay(25);
	} while(time1[T4]<100);

	setWheelSpeed(0);
	return true;
}

void setGyroTargetPID (gyroscope *gyroController, float target) {
	pid *controller = gyroController->controller;
	clearIntegral(controller);
	controller->target = target;
}

void setGyroTargetPIDAutoPointTurn (gyroscope *gyroController, float target) {
	setGyroTargetPID(gyroController, target);
	pointTurnGyroPID(gyroController);
}

void setGyroTargetPIDAutoRightSwingTurn (gyroscope *gyroController, float target) {
	setGyroTargetPID(gyroController, target);
	rightSwingTurnGyroPID(gyroController);
}

void setGyroTargetPIDAutoLeftSwingTurn (gyroscope *gyroController, float target) {
	setGyroTargetPID(gyroController, target);
	leftSwingTurnGyroPID(gyroController);
}

void addGyroTargetPID (gyroscope *gyroController, float target) {
	pid *controller = gyroController->controller;
	clearIntegral(controller);
	controller->target = controller->target + target;
}

void addGyroTargetPIDAutoPointTurn (gyroscope *gyroController, float target) {
	addGyroTargetPID(gyroController, target);
	pointTurnGyroPID(gyroController);
}

void addGyroTargetPIDAutoRightSwingTurn (gyroscope *gyroController, float target) {
	addGyroTargetPID(gyroController, target);
	rightSwingTurnGyroPID(gyroController);
}

void addGyroTargetPIDAutoLeftSwingTurn (gyroscope *gyroController, float target) {
	addGyroTargetPID(gyroController, target);
	leftSwingTurnGyroPID(gyroController);
}
