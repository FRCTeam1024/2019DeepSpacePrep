/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TurnToTarget extends Command {

  boolean isFinished = false;

  public TurnToTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double targetX = 160;
    double difX1 = Math.abs(targetX - Robot.getCenterX1());
    double difX2 = Math.abs(targetX - Robot.getCenterX2());
    System.out.println("Num Objects : " + Robot.getNumImageObjects());
    if(Robot.getNumImageObjects() < 2) {
      Robot.drivetrain.turn(0.50);
    } else {
      System.out.println("Difx1: " + difX1);
      System.out.println("Difx2: " + difX2);
      System.out.println("Center X 1: " + Robot.getCenterX1());
      System.out.println("Center X 2: " + Robot.getCenterX2());
      if(Math.abs(difX1 - difX2) < 10) {
        System.out.println("FINISHING TURN TO TARGET");
        Robot.drivetrain.turn(0);
        isFinished = true;
      } else {
        Robot.drivetrain.turn(0.50);
      }
      
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
