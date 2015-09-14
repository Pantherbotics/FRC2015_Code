package org.usfirst.frc.team3863.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Auton implements CompList {

	/**
	 * First stupidly simple auton Drive forward for 2.6 second
	 */

	public static void auton1() {
		AutonStraight(0.5, 60);
		Timer.delay(0.2);
		autonTurnRelative(90);
	}

	public static void auton2() {
		// compressor.start();
		SmartDashboard.putNumber("Begin auton1", 1);

		// TODO: delete this for Saturday
		// TODO: lessen the delays
		//intGyro.initGyro();
		intGyro.reset();

		transmissionSwitch.set(DoubleSolenoid.Value.kForward);

		//AutonStrafe(0.5, 200);

		// Timer.delay(2);
		OperatorControlThread.clawRelease(false);
		AutonLift(3000);
		AutonStraight(0.8, 15);
		Timer.delay(0.1);
		OperatorControlThread.claw(true);
		Timer.delay(0.1);
		AutonLift(7700);
		//Timer.delay(0.1);
		SmartDashboard.putNumber("WinchLoop", 5); // winchMotor6.set(0); // winchMotor3.set(0); Timer.delay(2); intGyro.reset(); autonTurnRelative(-90); Timer.delay(2); AutonStraight(-0.5, -4); Timer.delay(2); AutonStrafe(0.5, 100);
		autonTurnRelative(-87);
		//Timer.delay(0.1);
		AutonStraight(-0.9, -10);
		//Timer.delay(0.1);
		AutonStrafe(0.8, 22);
		//Timer.delay(0.1);
		OperatorControlThread.clawRelease(true);
		Timer.delay(0.1);
		OperatorControlThread.claw(true);
		// SmartDashboard.putNumber("Begin auton1", 2);
		// Timer.delay(2);
		// SmartDashboard.putNumber("Begin auton1", 3);
		//Timer.delay(0.1);
		AutonStraight(0.2, 13);
		//Timer.delay(0.1);
		OperatorControlThread.clawRelease(true);
		//Timer.delay(0.1);
		AutonLift(3200);
		//Timer.delay(0.1);
		OperatorControlThread.claw(false);
		Timer.delay(0.25);
		OperatorControlThread.calibrateWinch();
		//Timer.delay(0.1);
		AutonLift(800);
		//Timer.delay(0.1);
		OperatorControlThread.claw(true);
		Timer.delay(0.25);
		OperatorControlThread.clawRelease(true);
		//Timer.delay(0.1);
		AutonLift(3200);
		//Timer.delay(0.1);
		OperatorControlThread.claw(true);
		//Timer.delay(0.1);
		intGyro.reset();
		autonTurnRelative(-90);
		//Timer.delay(0.1);
		AutonStraight(1, 62);
		AutonStraight(0.5, 10);
		//Timer.delay(0.7);
		// AutonStraight(0.5, 100);
		// Timer.delay(2);
		autonTurnRelative(90);
		// Timer.delay(2);
		// AutonTurn(90);

		transmissionSwitch.set(DoubleSolenoid.Value.kReverse);
	}

	public static void auton3() {
		intGyro.reset();
		AutonLift(2800);
		AutonStraight(0.5, 15);
		OperatorControlThread.claw(true);
		Timer.delay(0.5);
		AutonLift(4500);
		AutonStraight(-0.5, -72);
		autonTurnRelative(90);

		// AutonStraight(0.5, 60);
		// Timer.delay(2);
		// autonTurnRelative(90);
	}
	public static void auton4(){
		// compressor.start();
				SmartDashboard.putNumber("Begin auton1", 1);

				// TODO: delete this for Saturday
				// TODO: lessen the delays
				//intGyro.initGyro();
				intGyro.reset();

				transmissionSwitch.set(DoubleSolenoid.Value.kForward);

				//AutonStrafe(0.5, 200);

				// Timer.delay(2);
				OperatorControlThread.clawRelease(false);
				OperatorControlThread.claw(true);
				Timer.delay(0.2);
				//AutonStraight(0.8, 15);				
				AutonLift(5000);
				//Timer.delay(0.1);
				SmartDashboard.putNumber("WinchLoop", 5); // winchMotor6.set(0); // winchMotor3.set(0); Timer.delay(2); intGyro.reset(); autonTurnRelative(-90); Timer.delay(2); AutonStraight(-0.5, -4); Timer.delay(2); AutonStrafe(0.5, 100);
				autonTurnRelative(-55);
				//Timer.delay(0.1);
				//AutonStraight(-0.9, -10);
				//Timer.delay(0.1);
				AutonStrafe(0.8, 24);
				//Timer.delay(0.1);
				//OperatorControlThread.clawRelease(true);
				//Timer.delay(0.2);
				//OperatorControlThread.claw(true);
				// SmartDashboard.putNumber("Begin auton1", 2);
				// Timer.delay(2);
				// SmartDashboard.putNumber("Begin auton1", 3);
				//Timer.delay(0.1);
				AutonStraight(0.2, 20);
				//Timer.delay(0.1);
				OperatorControlThread.clawRelease(true);
				//Timer.delay(0.1);
				AutonLift(3200);
				//Timer.delay(0.1);
				OperatorControlThread.claw(false);
				Timer.delay(0.25);
				//OperatorControlThread.calibrateWinch();
				//Timer.delay(0.1);
				AutonLift(400);
				//Timer.delay(0.1);
				OperatorControlThread.claw(true);
				Timer.delay(0.5);
				OperatorControlThread.clawRelease(true);
				//Timer.delay(0.1);
				AutonLift(3200);
				//Timer.delay(0.1);
				OperatorControlThread.claw(true);
				//Timer.delay(0.1);
				intGyro.reset();
				autonTurnRelative(-90);
				//Timer.delay(0.1);
				AutonStraight(1, 62);
				AutonStraight(0.5, 10);
				//Timer.delay(0.7);
				// AutonStraight(0.5, 100);
				// Timer.delay(2);
				autonTurnRelative(90);
				// Timer.delay(2);
				// AutonTurn(90);

				transmissionSwitch.set(DoubleSolenoid.Value.kReverse);
	}
	
	public static void auton5() {
		
		OperatorControlThread.claw(true);
		AutonLift(3000);
		Timer.delay(0.5);
		AutonStraight(0.5, 10);
		Timer.delay(0.5);
		AutonLift(1300);
		Timer.delay(0.5);
		AutonStraight(-0.5, -56);
		Timer.delay(0.2);
		autonTurnRelative(90);
	}
	public static void auton6(){
		AutonStraight(0.2, 18);
		Timer.delay(0.2);
		AutonLift(1600);
		Timer.delay(0.2);
		OperatorControlThread.claw(true);
		Timer.delay(0.2);
		//AutonLift(3200);
		AutonStraight(-0.5, -85);
		Timer.delay(0.2);
		autonTurnRelative(90);
		
	}
	public static void auton7(){
		
		
	}

	public static void AutonStrafe(double speed, double distance) {
		// SmartDashboard.putNumber("initGyro", 1);
		// intGyro.initGyro();
		// SmartDashboard.putNumber("initGyro", 2);

		frontRightMotor.setPosition(0);
		backRightMotor.setPosition(0);
		frontLeftMotor.setPosition(0);
		backLeftMotor.setPosition(0);

		Timer.delay(0.1);

		// double StartingPosBL = backLeftMotor.getEncPosition();
		// double StartingPosBR = backRightMotor.getEncPosition();
		// double StartingPosFL = frontLeftMotor.getEncPosition();
		// double StartingPosFR = frontRightMotor.getEncPosition();

		// SmartDashboard.putNumber("StartingPosBL", StartingPosBL);

		double currentAngle = intGyro.getAngle();
		double EncoderTicks = (distance * 12500) / (8 * 3.1415926535 * .9);
		SmartDashboard.putNumber("Encoder", EncoderTicks);
		while ((Math.abs(backLeftMotor.getEncPosition()) + Math.abs(backRightMotor.getEncPosition()) + Math.abs(frontLeftMotor.getEncPosition()) + Math.abs(frontRightMotor.getEncPosition())) < Math.abs(4 * EncoderTicks)) {
			SmartDashboard.putNumber("BLLoop Position", backLeftMotor.getEncPosition());
			SmartDashboard.putNumber("BRLoop Position", backRightMotor.getEncPosition());
			SmartDashboard.putNumber("FLLoop Position", frontLeftMotor.getEncPosition());
			SmartDashboard.putNumber("FRLoop Position", frontRightMotor.getEncPosition());
			OperatorControlThread.driveStrafePERFECTLY(speed, currentAngle);
		}
		SmartDashboard.putNumber("Ending Position", backLeftMotor.getEncPosition());
		OperatorControlThread.moveStraight(0, 0, 0, 0);
	}

	public static void AutonStraight(double speed, double distance) {
		// intGyro.initGyro();
		frontRightMotor.setPosition(0);
		backRightMotor.setPosition(0);
		frontLeftMotor.setPosition(0);
		backLeftMotor.setPosition(0);
		double currentAngle = intGyro.getAngle();
		double EncoderTicks = (distance * 12500) / (8 * 3.1415926535);
		SmartDashboard.putNumber("LoopPos", 1);
		Timer.delay(0.1);
		while ((Math.abs(backLeftMotor.getEncPosition()) + Math.abs(backRightMotor.getEncPosition()) + Math.abs(frontLeftMotor.getEncPosition()) + Math.abs(frontRightMotor.getEncPosition())) < Math.abs(4 * EncoderTicks)) {
			SmartDashboard.putNumber("BLLoop Position", backLeftMotor.getEncPosition());
			SmartDashboard.putNumber("BRLoop Position", backRightMotor.getEncPosition());
			SmartDashboard.putNumber("FLLoop Position", frontLeftMotor.getEncPosition());
			SmartDashboard.putNumber("FRLoop Position", frontRightMotor.getEncPosition());
			SmartDashboard.putNumber("LoopPos", 2);
			OperatorControlThread.driveStraightPERFECTLY(speed, currentAngle);
		}
		SmartDashboard.putNumber("Ending Position", backLeftMotor.getEncPosition());
		SmartDashboard.putNumber("LoopPos", 3);
		OperatorControlThread.moveStraight(0, 0, 0, 0);
	}

	public static void autonTurnRelative(double position) {
		position *= 0.9;
		final double gain = 0.5;
		final double initial = intGyro.getAngle();
		final double wheelSpeed = -position / Math.abs(position) * gain;

		while (Math.abs(initial - intGyro.getAngle()) < Math.abs(position)) {
			SmartDashboard.putNumber("Gyro Position", intGyro.getAngle());
			OperatorControlThread.moveStraight(wheelSpeed, wheelSpeed, wheelSpeed, wheelSpeed);
			double myAngle = intGyro.getAngle();
			SmartDashboard.putNumber("myAngle", myAngle);
		}
		OperatorControlThread.moveStraight(0, 0, 0, 0);
	}

	public static void AutonLift(double target) {
		SmartDashboard.putNumber("WinchLoop", 1);
		double CurrentLiftSpot = Math.abs(winchMotor3.getEncPosition());
		while (CurrentLiftSpot < (target - 300) || CurrentLiftSpot > (target + 300)) {
			SmartDashboard.putNumber("WinchEncoder", winchMotor3.getEncPosition());
			SmartDashboard.putNumber("WinchLoop", 2);
			OperatorControlThread.liftPID(target);
			CurrentLiftSpot = Math.abs(winchMotor3.getEncPosition());
		}
		SmartDashboard.putNumber("WinchLoop", 3);
		winchMotor3.set(0);
		winchMotor6.set(0);
		SmartDashboard.putNumber("WinchLoop", 4);
	}
	

}
