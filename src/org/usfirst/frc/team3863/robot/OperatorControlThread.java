package org.usfirst.frc.team3863.robot;

import java.io.File;
import java.io.PrintWriter;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.NetworkButton;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OperatorControlThread extends Thread implements CompList {

	Robot instance;
	


	long currentFrame = 0; // Used to keep track of time
	static long liftTarget = 0; // The target setpoint for the lift mechanism
	boolean transmissionState = false; // Sets if the transmission is active or not
	boolean transmissionIsSwitchable; // Is the transmission button in the middle of a state change?
	boolean forkState = false; // Sets the state of the pincer forks
	boolean forkIsSwitchable; // Is the fork button in the middle of a state change?
	int joystickMode = 0; // Drive mode for the joysticks
	boolean liftLocked = true; // Do we want to lock the lift from being moved manually?
	boolean feederRunning = false;

	// Angle setpoint for the gyro during auton
	static double currentAngle;
	// Angle setpoint for the gyro during angle
	double targetPosition = 0;

	NetworkTable nt = NetworkTable.getTable("RoboInfo");
	
	





	public OperatorControlThread(Robot robot) {
		instance = robot;
	}

	@Override
	public void run() {

		
		
		
		// Get the previous position of the encoder stored in memory
		readLastPosition();

		// Enable the compressor
		compressor.start();

		// Zero the winch, ensuring we have accurate position.
		calibrateWinch();

		liftPIDcontroller.disable();
		


		

		// Main Loop
		while (instance.isOperatorControl() && instance.isEnabled()) {
			cleanMove();
			//compressor.stop();
			

			if (rightJoy.getRawButton(GYRO_ZERO_B)) {
				targetPosition = intGyro.getAngle();
			}

			// Change the lift target on joystick input only when the limit switches are not active
			if (rightJoy.getRawButton(LIFT_RAISE_B)) {
				liftTarget += 140;
				liftLocked = false;
			} else if (rightJoy.getRawButton(LIFT_LOWER_B)) {
				liftTarget -= 140;
				liftLocked = false;
			} else if (liftLocked == false) {
				liftTarget = -winchMotor3.getEncPosition();
				liftLocked = true;
			}

			// Run the PID algorithm for the lift motors
			liftPID(liftTarget);

			// If the trigger is no t
			if (!rightJoy.getTrigger()) {
				transmissionIsSwitchable = true;
			}
			if (rightJoy.getTrigger() && transmissionIsSwitchable) {
				transmissionIsSwitchable = false;
				transmissionState = !transmissionState;
			}

			if (rightJoy.getRawButton(7)) {
				liftTarget = 0;
			}
			if (rightJoy.getRawButton(9)) {
				liftTarget = 3850;
			}
			if (rightJoy.getRawButton(11)) {
				liftTarget = 6700;
				System.out.println("Top Kek m8");
			}

			clawRelease(rightJoy.getRawButton(LIFT_NEUTRAL_B));

			if (transmissionState) {
				transmissionSwitch.set(DoubleSolenoid.Value.kForward);
			} else {
				transmissionSwitch.set(DoubleSolenoid.Value.kReverse);
			}
 
			if (!rightJoy.getRawButton(CLAW_TOGGLE_B)) {
				forkIsSwitchable = true;
			}
			if (rightJoy.getRawButton(CLAW_TOGGLE_B) && forkIsSwitchable) {
				forkIsSwitchable = false;
				forkState = !forkState;
				claw(forkState);
			}

			

			if (rightJoy.getRawButton(8)) {
				calibrateWinch();
			}

			if (currentFrame % 100 == 0) {
				writeCurrentPosition();
			}
			
			if(rightJoy.getRawButton(10) && feederRunning == false)
			{
				feederPickup();
			
			}

			updateDashboardInfo();

		}
	}
	
	public void feederPickup()
	{
		feederRunning = true;
		clawRelease(true);
		Timer.delay(0.1);
		Auton.AutonLift(4600);
		Timer.delay(0.1);
		claw(false);
		Timer.delay(0.2);
		Auton.AutonLift(2500);
		Timer.delay(0.1);
		claw(true);
		Timer.delay(0.3);
		clawRelease(true);
		Timer.delay(0.1);
		Auton.AutonLift(3700);
		Timer.delay(0.1);
		clawRelease(false);
		Timer.delay(0.1);
		Auton.AutonLift(4400);
		feederRunning = false;
	}

	public void updateDashboardInfo() {
		SmartDashboard.putNumber("Gyro", intGyro.getAngle());
		currentFrame++;
		SmartDashboard.putNumber("frame", currentFrame);
		// PID loop tuning
		SmartDashboard.putData("test", liftPIDcontroller);
		SmartDashboard.putNumber("Angle", currentAngle);
		SmartDashboard.putNumber("hat", rightJoy.getPOV());

		SmartDashboard.putNumber("set", liftTarget);

		SmartDashboard.putNumber("Ultrasonic", liftUltra.getVoltage());

		if (transmissionState) {
			SmartDashboard.putString("TransmissionState", "Low Speed");
		} else {
			SmartDashboard.putString("TransmissionState", "High Speed");
		}

		nt.putNumber("LiftPosition", winchMotor3.getEncPosition());
		nt.putBoolean("GearState", !transmissionState);
		nt.putNumber("BatteryLevel", pdb.getVoltage());
	}

	public void cleanMove() {
		double speed = (1 - rightJoy.getThrottle()) / 2;

		if (rightJoy.getPOV() == 0) {
			driveStraightPERFECTLY(speed, targetPosition);
		} else if (rightJoy.getPOV() == 180) {
			driveStraightPERFECTLY(-speed, targetPosition);
		} else if (rightJoy.getPOV() == 270) {
			driveStrafePERFECTLY(-speed, targetPosition);
		} else if (rightJoy.getPOV() == 90) {
			driveStrafePERFECTLY(speed, targetPosition);
		} else {
			singleJoyDrive();
		}

	}

	/**
	 * Close or open the claw
	 * 
	 * @param enable
	 */
	public static void claw(boolean enable) {
		clawRelease(false);
		if (enable) {
			pincerSwitch.set(DoubleSolenoid.Value.kForward);
		} else {
			pincerSwitch.set(DoubleSolenoid.Value.kReverse);
		}
	}

	/**
	 * Enable or disable the release valve for the claw
	 * 
	 * @param enable
	 */
	public static void clawRelease(boolean enable) {
		if (enable) {
			releaseValve.set(DoubleSolenoid.Value.kReverse);
		} else {
			releaseValve.set(DoubleSolenoid.Value.kForward);
		}
	}

	/**
	 * Drive mode using two joysticks, left for steering, right for throttle
	 */
	public void dualJoyDrive() {
		double thr = (1 - rightJoy.getThrottle()) / 2;

		double x = -leftJoy.getX();
		if (Math.abs(x) < 0.08) {
			x = 0;
		}
		double y = leftJoy.getY();
		if (Math.abs(y) < 0.08) {
			y = 0;
		}
		double t = rightJoy.getTwist() * 0.75;
		if (Math.abs(t) < 0.05) {
			t = 0;
		}

		double backLeftSpeed = -(-y + x + t) * thr;
		double frontLeftSpeed = -(-y - x + t) * thr;
		double backRightSpeed = -(y + x + t) * thr;
		double frontRightSpeed = -(y - x + t) * thr;

		moveStraight(backLeftSpeed, backRightSpeed, frontLeftSpeed, frontRightSpeed);
	}

	/**
	 * Drive using the same throttle as the joystick
	 */
	public void singleJoyDrive() {
		double thr = (1 - rightJoy.getThrottle()) / 2;

		double x = -rightJoy.getX();
		if (Math.abs(x) < 0.08) {
			x = 0;
		}
		double y = rightJoy.getY();
		if (Math.abs(y) < 0.05) {
			y = 0;
		}
		double t = rightJoy.getTwist() * 0.75;
		if (Math.abs(t) < 0.05) {
			t = 0;
		}

		double backLeftSpeed = -(-y + x + t) * thr;
		double frontLeftSpeed = -(-y - x + t) * thr;
		double backRightSpeed = -(y + x + t) * thr;
		double frontRightSpeed = -(y - x + t) * thr;

		moveStraight(backLeftSpeed, backRightSpeed, frontLeftSpeed, frontRightSpeed);
	}

	/**
	 * Helper function to set all four motor powers in a single command
	 * 
	 * @param BL
	 *            - Back Left motor power
	 * @param BR
	 *            - Back Right motor power
	 * @param FL
	 *            - Front Left motor power
	 * @param FR
	 *            - Front Right motor power
	 */
	public static void setAllMotors(double BL, double BR, double FL, double FR) {
		backLeftMotor.set(BL);
		backRightMotor.set(BR);
		frontLeftMotor.set(FL);
		frontRightMotor.set(FR);
	}

	/**
	 * the setpoint based PID loop for the lift motors
	 * 
	 * @param setpoint
	 *            - Setpoint in ticks to set the lift. 0 is full down, ~6000 is full up
	 */
	public static void liftPID(double setpoint) {
		double error = -winchMotor3.getEncPosition() - setpoint;
		double speed = error * -0.00105;
		SmartDashboard.putNumber("speed", speed);
		SmartDashboard.putNumber("enc", winchMotor3.getEncPosition());
		if (Math.abs(speed) < 0.25) {
			speed = 0;
		}
		winch(speed);

	}

	public void LiftPIDnew(double setpoint) {
		liftPIDcontroller.setSetpoint(setpoint);

	}

	/**
	 * Move the wheels at the same velocity
	 * 
	 * @param BL
	 *            - Back Left motor power
	 * @param BR
	 *            - Back Right motor power
	 * @param FL
	 *            - Front Left motor power
	 * @param FR
	 *            - Front Right motor power
	 */

	public static void moveStraight(double BL, double BR, double FL, double FR) {
		double cFR = straightMove(FR, frontRightMotor.getEncVelocity());
		double cBR = straightMove(BR, backRightMotor.getEncVelocity());
		double cFL = straightMove(FL, frontLeftMotor.getEncVelocity());
		double cBL = straightMove(BL, backLeftMotor.getEncVelocity());
		SmartDashboard.putNumber("FL_velocity", frontLeftMotor.getEncVelocity());
		SmartDashboard.putNumber("FR_velocity", frontRightMotor.getEncVelocity());
		SmartDashboard.putNumber("BL_velocity", backLeftMotor.getEncVelocity());
		SmartDashboard.putNumber("BR_velocity", backRightMotor.getEncVelocity());
		setAllMotors(cBL, cBR, cFL, cFR);
	}

	/**
	 * Modify a target speed to match the velocity
	 * 
	 * @param speed
	 *            - the input target speed
	 * @param velocity
	 *            - the encoder velocity
	 * @return - the speed
	 */
	public static double straightMove(double speed, double velocity) {
		if (speed == 0) {
			return 0;
		}
		double target = Math.log(speed) * -10;
		// error = target - velocity; // Velocity is inverted, so add instead of subtract
		// System.out.println(target+" "+velocity+" "+error);
		SmartDashboard.putNumber("target", target);
		return speed; // - (error/1500)*0.5;

	}

	/**
	 * Run the winch motors unless it is at a limit switch
	 * 
	 * @param power
	 *            - the output power of both winches
	 */
	public static void winch(double power) {
		if (winchMotor6.isFwdLimitSwitchClosed() && power > 0) {
			power = 0;
		}
		if (winchMotor6.isRevLimitSwitchClosed() && power < 0) {
			power = 0;
		}
		winchMotor6.set(power);
		winchMotor3.set(-power);
	}

	/**
	 * calibrate the winch encoder
	 */
	public static void calibrateWinch() {
		claw(false);
		while (!winchMotor6.isRevLimitSwitchClosed()) {
			winch(-0.6);
		}
		winchMotor3.setPosition(0);
		liftTarget = 0;

		writeCurrentPosition();
	}

	/**
	 * Read the last encoder position from the flash
	 */
	public void readLastPosition() {
		try {

			if (new File("/robotStorage/liftPos").isFile()) {
				List<String> data = Files.readAllLines(Paths.get("/robotStorage/liftPos"), Charset.defaultCharset());
				int newPos = Integer.parseInt(data.get(0).trim());
				System.out.println(newPos);
				winchMotor3.setPosition(newPos);
			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Write the encoder position to the flash
	 */
	public static void writeCurrentPosition() {
		PrintWriter writer;
		try {
			writer = new PrintWriter("/robotStorage/liftPos", "UTF-8");
			writer.println(-winchMotor3.getEncPosition());
			writer.write("            ");
			writer.close();
		} catch (Exception e) {
			e.printStackTrace();
		}

	}

	/**
	 * 
	 */
	public void pickup() {
		pincerSwitch.set(DoubleSolenoid.Value.kForward);
		forkState = !forkState;
		double time = 0;
		while (time < 400) {
			Timer.delay(0.001);
			liftPID(liftTarget);
			time += 1;
			if (time == 300) {
				liftTarget += 600;
			}
			if (time == 100) {
				releaseValve.set(DoubleSolenoid.Value.kReverse);
			}
		}
		releaseValve.set(DoubleSolenoid.Value.kForward);

	}

	public static void turnToAngle(double destination) {
		double error = destination - intGyro.getAngle();
		double speed = error * -0.0081;
		moveStraight(speed, speed, speed, speed);
	}
	
	/*
	public static double distanceIR(int robotDirection) {
		int curveState = 2; //curveState 2 = logarithmic region, curveState 1 = linear region
		//robotDirection 1 = forward, robotDirection 2 = backwards
		double rawDist = an3.getVoltage();
		if (Math.abs(rawDist - 3) < 0.01) {
			if (robotDirection == 1) {
				curveState = 2;
			} else if (robotDirection == 2) {
				curveState = 1;
			}
		}
		
		if (rawDist < 0.6) {
			return -1;
		} else if (curveState == 1) {
			//linear regression
		} else if (curveState == 2) {
			double distanceMidRange = -9.77145 * math.log(0.226765 * rawDist);
		}
		
	}
	*/

	public static void driveStraightPERFECTLY(double speed, double targetAngle) {
		double error1 = targetAngle - intGyro.getAngle();
		double correction1 = error1 * 0.01;
		double error2 = targetAngle - intGyro.getAngle();
		double correction2 = error2 * 0.01;
		double error3 = targetAngle - intGyro.getAngle();
		double correction3 = error3 * 0.01;
		double error4 = targetAngle - intGyro.getAngle();
		double correction4 = error4 * 0.01;
		if (speed == 0) {
			correction1 = 0;
		}
		if (correction1 > 0.5) {
			correction1 = 0.5;
		}
		if (correction1 < -0.5) {
			correction1 = -0.5;
		}
		if (speed == 0) {
			correction2 = 0;
		}
		if (correction2 > 0.5) {
			correction2 = 0.5;
		}
		if (correction2 < -0.5) {
			correction2 = -0.5;
		}
		if (speed == 0) {
			correction3 = 0;
		}
		if (correction3 > 0.5) {
			correction3 = 0.5;
		}
		if (correction3 < -0.5) {
			correction3 = -0.5;
		}
		if (speed == 0) {
			correction4 = 0;
		}
		if (correction4 > 0.5) {
			correction4 = 0.5;
		}
		if (correction4 < -0.5) {
			correction4 = -0.5;
		}

		moveStraight(-speed - correction1, speed - correction2, -speed - correction3, speed - correction4);

	}

	public static void driveStrafePERFECTLY(double speed, double targetAngle) {
		double error1 = targetAngle - intGyro.getAngle();
		double correction1 = error1 * 0.01;
		double error2 = targetAngle - intGyro.getAngle();
		double correction2 = error2 * 0.01;
		double error3 = targetAngle - intGyro.getAngle();
		double correction3 = error3 * 0.01;
		double error4 = targetAngle - intGyro.getAngle();
		double correction4 = error4 * 0.01;
		if (speed == 0) {
			correction1 = 0;
		}
		if (correction1 > 0.5) {
			correction1 = 0.5;
		}
		if (correction1 < -0.5) {
			correction1 = -0.5;
		}
		if (speed == 0) {
			correction2 = 0;
		}
		if (correction2 > 0.5) {
			correction2 = 0.5;
		}
		if (correction2 < -0.5) {
			correction2 = -0.5;
		}
		if (speed == 0) {
			correction3 = 0;
		}
		if (correction3 > 0.5) {
			correction3 = 0.5;
		}
		if (correction3 < -0.5) {
			correction3 = -0.5;
		}
		if (speed == 0) {
			correction4 = 0;
		}
		if (correction4 > 0.5) {
			correction4 = 0.5;
		}
		if (correction4 < -0.5) {
			correction4 = -0.5;
		}

		SmartDashboard.putNumber("speed", speed);
		SmartDashboard.putNumber("correction4", correction4);

		moveStraight(speed + correction1, speed + correction2, -speed - correction3, -speed - correction4);
	}

	
	
}
