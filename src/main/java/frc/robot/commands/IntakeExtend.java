package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeExtend extends Command {
	
    public IntakeExtend() {
    	requires(Robot.intake);
    }

    protected void initialize() {
    	Robot.intake.slideOut();
    }
    
    protected void execute() {
    }

    protected boolean isFinished() {
        return true;
    }

    protected void end() {
    }

    protected void interrupted() {
    }
}