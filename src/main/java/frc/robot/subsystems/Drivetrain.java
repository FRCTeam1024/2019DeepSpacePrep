/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends Subsystem {
	private TalonSRX frontLeft  = new TalonSRX(RobotMap.FRONT_LEFT_MOTOR_PORT);
	private TalonSRX middleLeft = new TalonSRX(RobotMap.MIDDLE_LEFT_MOTOR_PORT);
	private TalonSRX rearLeft = new TalonSRX(RobotMap.REAR_LEFT_MOTOR_PORT);
	private TalonSRX frontRight = new TalonSRX(RobotMap.FRONT_RIGHT_MOTOR_PORT);
	private TalonSRX middleRight = new TalonSRX(RobotMap.MIDDLE_RIGHT_MOTOR_PORT);
	private TalonSRX rearRight = new TalonSRX(RobotMap.REAR_RIGHT_MOTOR_PORT);
	
	private Solenoid shifter = new Solenoid(RobotMap.SHIFTER_PORT);
	//private AHRS navx;
	private AnalogGyro navx;
	
	public double rotateToAngleRate;
	
//	public double pidGet;
	
	//Remove these and any references when set properly
	public double turnkP = Constants.TURN_KP;
	public double turnkI = Constants.TURN_KI;
	public double turnkD = Constants.TURN_KD;
	public double turnkF = Constants.TURN_KF;
	
	public Encoder encoder = new Encoder(RobotMap.DRIVE_ENCODER_CHANNEL_A, RobotMap.DRIVE_ENCODER_CHANNEL_B, false, EncodingType.k4X);
	
	public PIDController posPID;
	public PIDController turnPID;
	public PIDController trimPID;
	
	public Drivetrain() {
		frontRight.setInverted(false); //might take this out
		rearRight.setInverted(false);
		frontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		
		setFollower(rearLeft, frontLeft);
		setFollower(rearRight, frontRight);
		setFollower(middleLeft, frontLeft);
		setFollower(middleRight, frontRight);
		
		//navx = new AHRS(RobotMap.NAVX_PORT);
		navx = new AnalogGyro(0);
		//navx = new AHRS(SerialPort.Port.kUSB);
		navx.setPIDSourceType(PIDSourceType.kDisplacement);
		navx.reset();
		
		turnPID = new PIDController(Constants.TURN_KP, Constants.TURN_KI, Constants.TURN_KD, navx, output->{});
        turnPID.setInputRange(Constants.MIN_ROTATION_ANGLE, Constants.MAX_ROTATION_ANGLE);
        turnPID.setContinuous(true);
        turnPID.setOutputRange(Constants.TURN_PID_MIN_OUTPUT, Constants.TURN_PID_MAX_OUTPUT); //probably will be much less
        turnPID.setAbsoluteTolerance(Constants.TURN_PID_ABSOLUTE_TOLERANCE);
        //turnPID.setOutputRange(30, maximumOutput);
        //turnPID.setPercentTolerance(2.0/360.0);
        
        trimPID = new PIDController(Constants.TRIM_KP, Constants.TRIM_KI, Constants.TRIM_KD, navx, output->{});
        trimPID.setInputRange(Constants.MIN_ROTATION_ANGLE, Constants.MAX_ROTATION_ANGLE);
        trimPID.setContinuous(true);
        trimPID.setOutputRange(Constants.TRIM_PID_MIN_OUTPUT, Constants.TRIM_PID_MAX_OUTPUT); //probably will be much less
        //trimPID.setAbsoluteTolerance(0.5);
        
        
        encoder.setPIDSourceType(PIDSourceType.kDisplacement);
        encoder.setDistancePerPulse(Constants.DRIVETRAIN_ENCODER_DISTANCE_PER_PULSE);
        encoder.setReverseDirection(true);
        
        posPID = new PIDController(Constants.POS_KP, Constants.POS_KI, Constants.POS_KD, encoder, output->{});
	
        posPID.setOutputRange(Constants.POS_PID_MIN_OUTPUT, Constants.POS_PID_MAX_OUTPUT);
        
        //turnPID.setPercentTolerance(1.0);
        
        
	}
	
	public void setCoast() {
		frontLeft.setNeutralMode(NeutralMode.Coast);
		middleLeft.setNeutralMode(NeutralMode.Coast);
		rearLeft.setNeutralMode(NeutralMode.Coast);
		frontRight.setNeutralMode(NeutralMode.Coast);
		middleRight.setNeutralMode(NeutralMode.Coast);
		rearRight.setNeutralMode(NeutralMode.Coast);
	}
	
	public void setBrake() {
		frontLeft.setNeutralMode(NeutralMode.Brake);
		middleLeft.setNeutralMode(NeutralMode.Brake);
		rearLeft.setNeutralMode(NeutralMode.Brake);
		frontRight.setNeutralMode(NeutralMode.Brake);
		middleRight.setNeutralMode(NeutralMode.Brake);
		rearRight.setNeutralMode(NeutralMode.Brake);
	}
	
	public void shiftLow() {
		shifter.set(false);
		System.out.println("Shifting Low");
	}
	
	public void shiftHigh() {
		shifter.set(true);
		System.out.println("Shifting High");
	}
	
	/**
	 * @returns true if the robot is moving.
	 */
	public boolean isMoving() {
		return Math.abs(frontLeft.getMotorOutputPercent()) > 0.05 || 
			   Math.abs(frontRight.getMotorOutputPercent()) > 0.05;
	}
	
	/**
	 * @returns The heading from the navx (in degrees).
	 */
	public double getHeading() {
		/*if (RobotState.isAutonomous()) {
			if (!navx.isConnected()) {
				Scheduler.getInstance().disable();
			}
		}*/
		return navx.getAngle();
	}
	
	/**
	 * Drives the motors toward the current angle setpoint
	 * Assumes that there is a setpoint set and that the pid has been enabled
	 */
	public void pidTurn() {
		double minPowerToMove = 0.2; // just a guess
		double netPower = turnPID.get();
		if(netPower < minPowerToMove && netPower > 0.0) {
			netPower = minPowerToMove;
		} else if (netPower > -minPowerToMove && netPower < 0.0) {
			netPower = -minPowerToMove;	
		}
		// drive(-(turnPID.get()), turnPID.get());
		drive(-(netPower), netPower);
	}
	
	public void turn(double power){
		drive(-(power), power);
	}

	public void pidTurnLeftOneSide(double oppositeSidePower) {
		double minPowerToMove = 0.5; // just a guess
		double netPower = turnPID.get();
		if(netPower < minPowerToMove && netPower > 0.0) {
			netPower = minPowerToMove;
		} else if (netPower > -minPowerToMove && netPower < 0.0) {
			netPower = -minPowerToMove;	
		}
		// drive(-(turnPID.get()), turnPID.get());
		//drive(-(netPower), netPower);
		drive(oppositeSidePower, netPower);
	}
	
	public void pidTurnRightOneSide(double oppositeSidePower) {
		double minPowerToMove = 0.5; // just a guess
		double netPower = turnPID.get();
		if(netPower < minPowerToMove && netPower > 0.0) {
			netPower = minPowerToMove;
		} else if (netPower > -minPowerToMove && netPower < 0.0) {
			netPower = -minPowerToMove;	
		}
		// drive(-(turnPID.get()), turnPID.get());
		drive(-(netPower), oppositeSidePower);
	}
	
	/**
	 * Drives the motors toward the current distance setpoint
	 * Maintains the current angle in order to drive straight
	 */
	public void pidDriveForwardStraight() {
		//drive(posPID.get() + turnPID.get(), posPID.get() + -1*turnPID.get());
		drive(-posPID.get() - trimPID.get(), -posPID.get() + trimPID.get());
	}
	
	public void pidDriveForwardStraightBias(double modifier) {
		//drive(posPID.get() + turnPID.get(), posPID.get() + -1*turnPID.get());
		drive(-posPID.get() - trimPID.get() + modifier, -posPID.get() + trimPID.get() + modifier);
	}
	
	/**
	 * Drives the motors toward the current distance setpoint
	 * Maintains the current angle in order to drive straight
	 * (This is a separate function because the gyro is backwards)
	 */
	public void pidDriveBackwardStraight() {
		drive(-posPID.get() - trimPID.get(), -posPID.get() + trimPID.get());
	}

	/**
	 * Drives the motors based on a percent
	 * @param leftPower value from -1.0 to 1.0
	 * @param rightPower value from -1.0 to 1.0
	 */
	public void drive(double leftPower, double rightPower) {
		frontLeft.set(ControlMode.PercentOutput, -leftPower);
		frontRight.set(ControlMode.PercentOutput, rightPower);
	}
	
	public void setFollower(TalonSRX slave, TalonSRX master) {
		slave.set(ControlMode.Follower, master.getDeviceID());
	}
	
	public void stop() {
		frontLeft.set(ControlMode.PercentOutput, 0.0);
		frontRight.set(ControlMode.PercentOutput, 0.0);
	}
	
	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoysticks());
	}

	public void resetGyro() {
		navx.reset();
		Timer.delay(0.2);
		//navx.zeroYaw();
	}
	
	public double getRawRightEncoder() {
		return encoder.get();
	}
	
	
	public double getOpticalDistanceInches() {
	/*	getWheelRotation() = distance / (Math.PI * RobotMap.WHEEL_DIAMETER) * RobotMap.ENCODER_COUNTS_PER_REVOLUTION;
		return (((getRawMagneticEncoder() / RobotMap.ENCODER_RATIO_TO_WHEEL * Math.PI * RobotMap.WHEEL_DIAMETER) / 
				RobotMap.OPTICAL_ENCODER_COUNTS_PER_REVOLUTION)/ 4) * 3 * 3;*/
		return encoder.getDistance(); //must have a specified distance per pulse set
	}
	
	public double getTicks(double distanceInInches) {
//		return (distance * 9 * RobotMap.ENCODER_COUNTS_PER_REVOLUTION * RobotMap.ENCODER_RATIO_TO_WHEEL * 
//				Math.PI * RobotMap.WHEEL_DIAMETER) / 12;
		
		int ticksPerInch = 71;
		return -1*(ticksPerInch * distanceInInches);
	}
	
	public void resetMagneticEncoder() {
		frontRight.setSelectedSensorPosition(0, 0, 0);
	}
	
	public void resetOpticalEncoder() {
		encoder.reset();
	}
	
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("Gyro Angle", getHeading());
		//System.out.println("Gyro: " + getHeading());
    	SmartDashboard.putNumber("Optical Encoder Distance (IN)", getOpticalDistanceInches());
    	SmartDashboard.putNumber("Encoder Raw", encoder.getRaw());
    	SmartDashboard.putNumber("posPID.get()", posPID.get());
    	SmartDashboard.putNumber("turnPID.get()", turnPID.get());
    	SmartDashboard.putBoolean("onTarget", turnPID.onTarget());
	}

	public boolean pidDriveUntil(double initialSpeed) {
		if (posPID.get() < 0.0) {
			return true;
		} else {
			return false;
		}
		
	}
	
	public void setRamp(double secondsUntilFull) {
		frontLeft.configOpenloopRamp(secondsUntilFull, 10);
		middleLeft.configOpenloopRamp(secondsUntilFull, 10);
		rearLeft.configOpenloopRamp(secondsUntilFull, 10);
		frontRight.configOpenloopRamp(secondsUntilFull, 10);
		middleRight.configOpenloopRamp(secondsUntilFull, 10);
		rearRight.configOpenloopRamp(secondsUntilFull, 10);
	}
	
	public void changePosPID(double posP, double posI, double posD) {
		posPID.setPID(posP, posI, posD);
	}
	
	public boolean getShiftState() {
		return shifter.get();
	}
}