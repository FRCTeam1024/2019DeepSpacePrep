/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.logging.*;

public class LogTest extends Command {

  private boolean isFinished = false;

  public LogTest() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  private void log(String subsystem, String msg) {
    HelixEvents.getInstance().addEvent(subsystem, msg);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    log("LOGTEST", "initializing...");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    log("LOGTEST", "executing...");
    isFinished = true;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    log("LOGTEST", "testing isFinished...");
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    log("LOGTEST", "ending cleanup...");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    log("LOGTEST", "interrupted!!!");
  }
}
