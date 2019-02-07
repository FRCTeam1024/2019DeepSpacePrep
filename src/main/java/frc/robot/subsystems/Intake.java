package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.IntakeWithJoystick;

/**
 *
 */
public class Intake extends Subsystem {
  public TalonSRX leftIntake = new TalonSRX(RobotMap.LEFT_INTAKE_MOTOR_PORT);
    public TalonSRX rightIntake = new TalonSRX(RobotMap.RIGHT_INTAKE_MOTOR_PORT);
    
    private Solenoid leftIntakeSlide = new Solenoid(RobotMap.INTAKE_SLIDE_PORT);
    private Solenoid leftIntakePos = new Solenoid(RobotMap.INTAKE_POS_PORT);
    
    private DigitalInput cubeDetector = new DigitalInput(RobotMap.LEFT_BUMP_CUBE_DETECTOR_PORT);
    //private DigitalInput rightBumpDetector = new DigitalInput(RobotMap.RIGHT_BUMP_CUBE_DETECTOR_PORT);
    
    public Relay cubeLight = new Relay(0);
    // public DigitalOutput cube2 = new DigitalOutput(4);
    private boolean intakeInState = true;
    private boolean intakeWideState = true;
    
    public Intake () {
    }
    
    public void setCubeLight() {
    	// if (cubeDetecterState() != true) {
    	// 	cube2.set(true);
    	// } else {
    	// 	cube2.set(false);
    	// }
    }
    public void initDefaultCommand() {
    	setDefaultCommand(new IntakeWithJoystick());
    }
  
    public void intakeSpeed(double Power) {
		leftIntake.set(ControlMode.PercentOutput, -Power);
		rightIntake.set(ControlMode.PercentOutput, Power);
	}
    
    public void intakeSpeed(double leftPower, double rightPower) {
    	leftIntake.set(ControlMode.PercentOutput, -leftPower);
    	rightIntake.set(ControlMode.PercentOutput, rightPower);
    }
	
    public void intakeStop() {
    	leftIntake.set(ControlMode.PercentOutput, 0.0);
		rightIntake.set(ControlMode.PercentOutput, 0.0);
    }
    
    public void slideOut() {
    	leftIntakeSlide.set(true);
    	intakeInState = false;
    }
    
    public void slideIn() {
    	posIn();
    	leftIntakeSlide.set(false);
    	Robot.lift.clamp(false);
    	intakeInState = true;
    }
    
    public void posOut() {
    	if (!intakeInState) {
    		leftIntakePos.set(true);
    		intakeWideState = false;
    	}
    }
    
    public void posIn() {
    	leftIntakePos.set(false);
    	intakeWideState = true;
    }

	public boolean cubeDetecterState() {
		if (cubeDetector.get()) {
			return true;
		} else {
			return false;
		}
	}
	
	public void outputToSmartDashboard() {
		SmartDashboard.putBoolean("BreakBeam", cubeDetecterState());
		SmartDashboard.putNumber("LeftIntake Current: ", leftIntake.getOutputCurrent());
		SmartDashboard.putNumber("RightIntake Current: ", rightIntake.getOutputCurrent());
	}
	
	public boolean intakeWideState() {
		return intakeWideState;
	}
	
	public boolean intakeInState() {
		return intakeInState;
	}
}
