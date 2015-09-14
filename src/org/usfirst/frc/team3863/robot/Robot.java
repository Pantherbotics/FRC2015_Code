package org.usfirst.frc.team3863.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SerialPort;

public class Robot extends SampleRobot implements CompList {

	Thread operatorControl;
	int autonMode = -1; // Selector for the program to run during auton mode
	
	
	DigitalInput sw1,sw2,sw3,sw4; //The for DIO for the auton select switch

	/**
	 * Constructor method, instantiate the motors and other classes
	 */
	public Robot() {

		//ystem.out.println("wow");		//intGyro.initGyro();
		OperatorControlThread.currentAngle = intGyro.getAngle();

		liftPIDcontroller.setOutputRange(-0.75, 0.75);
		liftPIDcontroller.setInputRange(-25, 6500);
		
		sw1 = new DigitalInput(0);
		sw2 = new DigitalInput(1);
		sw3 = new DigitalInput(2);
		
        autonMode = getAutonMode();
        SmartDashboard.putNumber("Auton Mode", autonMode);
		// Kill the compressor on startup
		compressor.stop();

		transmissionSwitch.set(DoubleSolenoid.Value.kForward);
		
		// Enable the compressor
		compressor.start();
	
		
		//readLastPosition();		// Restore the lift position at startup (calibration will override this)

	}
	
	public void robotInit()
	{
		SmartDashboard.putNumber("RobotInitDebug", 1);
		//intGyro.initGyro();
		SmartDashboard.putNumber("RobotInitDebug", 2);
		OperatorControlThread.clawRelease(true);
		SmartDashboard.putNumber("RobotInitDebug", 3);
	}

	public void autonomous() {

		if (operatorControl != null) {
			try {
				operatorControl.join();
			} catch (InterruptedException e) {
				
			}
			operatorControl = null;
		}
		
		
		autonMode = getAutonMode();
        SmartDashboard.putNumber("Auton Mode", autonMode);
        
		OperatorControlThread.calibrateWinch();
		autonMode = -1;
		switch (autonMode) {
		case 1:
			Auton.auton1();
			break;
		case 2:
			Auton.auton2();
			break;
		case 3:
			Auton.auton3();
			break;
		case 4:
			Auton.auton4();
			break;
		case 5:
			Auton.auton5();
			break;
		case 6:
			Auton.auton6();
			break;
	    }
	}

	public void disabled() {
		//writeCurrentPosition();	// Save the lift position when disabled
		
		intGyro.reset();
		double myAngle;
		
		frontRightMotor.set(0);
		frontLeftMotor.set(0);
		backRightMotor.set(0);
		backLeftMotor.set(0);
		winchMotor3.set(0);
		winchMotor6.set(0);

		if (operatorControl != null) {
			try {
				operatorControl.join();
			} catch (InterruptedException e) {

			}
			operatorControl = null;
		}

		while (isDisabled()) {
			SmartDashboard.putNumber("DisabledDebug", 1);
			//intGyro.initGyro();
			//System.out.println("Gyro Initialized");
			SmartDashboard.putNumber("DisabledDebug", 2);
			OperatorControlThread.clawRelease(true);
			//System.out.println("Claw Pressure Released");
			SmartDashboard.putNumber("DisabledDebug", 3);

			myAngle = intGyro.getAngle();
			SmartDashboard.putNumber("myAngle", myAngle);

			byte toSend[] = {3,0,0,0};
			serialMXP.writeString("Hi");
			Timer.delay(0.2);
			if(serialMXP.getBytesReceived() != 0)
			SmartDashboard.putString("SERIAL", serialMXP.readString());
		}
	}

	/**
	 * Called once during Driver Control
	 */
	public void operatorControl() {
		if (operatorControl == null) {
			operatorControl = new OperatorControlThread(this);
		}
		operatorControl.start();
	}

	/**
	 * Runs during test mode
	 */
	public void test() {
		
	}
	
	
	
	public int getAutonMode() {		
		boolean a = sw1.get();
		boolean b = sw2.get();
		boolean c = sw3.get();
		double val = an3.getVoltage();
		int swState = 0;
		SmartDashboard.putNumber("analog 3", val);
		
		if(val < 0.5)
		{
			swState = 1;
			return swState;
		}
		else if(val > 0.5 && val <3.5)
		{
			swState = 2;
			return swState;
		}
		else if(val > 3.5)
		{
			switch (a +" "+b +" "+c){
			case "true true true":
				swState = 3;
				return swState;
				
			case "true true false":
				swState = 4;
				return swState;
				
			case "true false false":
				swState = 5;
				return swState;
				
			case "false true false":
				swState = 6;
				return swState;
				
			case "false false false":
				swState = 7;
				return swState;
			}
				

			
		}
		
		return -1;
	}
	//0..6
	/*
	public void setLEDmode(int mode) {
		short bmode = (short)mode;
		short flags = 0;
		short index = 0;
		short length = 0;
		byte[] toSend = {(byte)3, (byte)flags, (byte)index, (byte)length };
		serialMXP.write(toSend, 4);
	}*/
		
	
	

}


