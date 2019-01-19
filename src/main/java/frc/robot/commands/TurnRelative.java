package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnRelative extends Command {
	double targetAngle;
	double degreeTolerance;
	int onTargetCount = 0;
    public TurnRelative(double targetAngle) {
    	requires(Robot.drivetrain);
    	this.targetAngle = targetAngle;
    	degreeTolerance = 3.0;
    }
    
    public TurnRelative(double targetAngle, double degreeTolerance) {
    	requires(Robot.drivetrain);
    	this.targetAngle = targetAngle;
    	this.degreeTolerance = degreeTolerance;
    }

    protected void initialize() {
    	Robot.drivetrain.resetGyro();
    	double currentAngle = Robot.drivetrain.getHeading();
    	//Robot.drivetrain.turnPID.setPercentTolerance(3.0/360.0);
    	Robot.drivetrain.turnPID.setSetpoint(targetAngle);
    	Robot.drivetrain.turnPID.enable();
    	Robot.drivetrain.shiftLow();
    }

    protected void execute() {
    	Robot.drivetrain.pidTurn();
    	SmartDashboard.putNumber("onTargetCount", onTargetCount);
    }
    
    private void log(Object msg) {
    	System.out.println(msg);
    }
    
    private boolean isOnTarget() {
    	//return Math.abs(Robot.drivetrain.getHeading() - targetAngle) < degreeTolerance;  //if the robot is within 1 degrees of the target, stop
    	
    	if (Math.abs(Robot.drivetrain.getHeading() - targetAngle) < 1.0) {
    		//if (Robot.drivetrain.turnPID.get() < 0.1 && Robot.drivetrain.turnPID.get() > -0.1) {
    			//return true;
    		//}
    		return true;
    	}
    	
    	
    	return false;
    }
        
    protected boolean isFinished() {
    	SmartDashboard.putNumber("OnTargetCount", onTargetCount);
    	if (isOnTarget()) {
    		onTargetCount++;
    	} else {
    		onTargetCount = 0;
    	}
    	
    	if(onTargetCount > 1) {
    		log("finished turn at " + Robot.drivetrain.getHeading() + "angle");
    		return true;
    	} else {
    		return false;
    	}
    }

    protected void end() {
    	Robot.drivetrain.stop();
    	Robot.drivetrain.turnPID.disable();
    	onTargetCount = 0;
    }

    protected void interrupted() {
    	Robot.drivetrain.stop();
    	Robot.drivetrain.turnPID.disable();
    	onTargetCount = 0;
    }
}