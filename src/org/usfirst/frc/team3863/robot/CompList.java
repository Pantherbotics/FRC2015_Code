package org.usfirst.frc.team3863.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface CompList {
	// Soft setpoints for the lift
	final int liftTop = 6492;
	final int liftBottom = -20;

	// Button constants for the controls
	final int LIFT_RAISE_B = 5;
	final int LIFT_LOWER_B = 3;
	final int LIFT_NEUTRAL_B = 2;
	final int GYRO_ZERO_B = 6;
	final int CLAW_TOGGLE_B = 4;

	// Drive Motors
	final CANTalon backLeftMotor = new CANTalon(13);
	final CANTalon backRightMotor = new CANTalon(10);
	final CANTalon frontLeftMotor = new CANTalon(12);
	final CANTalon frontRightMotor = new CANTalon(11);

	// Winch Motors
	final CANTalon winchMotor6 = new CANTalon(15);
	final CANTalon winchMotor3 = new CANTalon(14);

	// Joysticks
	final Joystick rightJoy = new Joystick(0);
	final Joystick leftJoy = new Joystick(1);

	// Pneumatics Control Module
	final Compressor compressor = new Compressor();
	final DoubleSolenoid transmissionSwitch = new DoubleSolenoid(0, 1);
	final DoubleSolenoid pincerSwitch = new DoubleSolenoid(2, 3);
	final DoubleSolenoid releaseValve = new DoubleSolenoid(4, 5);

	// Ultrasonic sensor
	final AnalogInput liftUltra = new AnalogInput(1);

	final Gyro intGyro = new Gyro(0);
	
	final AnalogInput an3 = new AnalogInput(3);


	final PIDSource liftInput = new PIDSource() {

		@Override
		public double pidGet() {
			// TODO Auto-generated method stub
			SmartDashboard.putNumber("Encval2:", winchMotor3.getEncPosition());
			return -winchMotor3.getEncPosition();
		}
	};

	final PIDOutput liftOutput = new PIDOutput() {

		@Override
		public void pidWrite(double output) {
			// TODO Auto-generated method stub
			OperatorControlThread.winch(output);
			SmartDashboard.putString("MYTest:", "" + output);
		}
	};

	final PIDController liftPIDcontroller = new PIDController(0.0015, 0.0001, 0, liftInput, liftOutput);
	
	final PowerDistributionPanel pdb = new PowerDistributionPanel();
	
	final SerialPort serialMXP = new SerialPort(9600, SerialPort.Port.kMXP);


}
