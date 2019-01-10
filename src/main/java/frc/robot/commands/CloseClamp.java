package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CloseClamp extends Command {
    public CloseClamp() {
    	requires(Robot.lift);
    }

    protected void initialize() {
    	Robot.lift.clamp(false);
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