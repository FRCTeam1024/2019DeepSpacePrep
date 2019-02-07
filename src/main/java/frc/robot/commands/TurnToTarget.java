/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.OI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.buttons.Trigger.ButtonScheduler;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TurnToTarget extends Command {

  boolean isFinished = false;
  private NetworkTable limelightTable;
  private NetworkTableEntry ta;

  public TurnToTarget() {
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    /* if(Robot.oi.lJoy.getTriggerPressed()){
      isFinished = true;
    }*/

    ta = limelightTable.getEntry("ta");
    double area = ta.getDouble(0.0);

    if (Robot.getLimeLightX() >= .5) { // target is right of center, so turn right
      // this turns right?
      Robot.drivetrain.drive(-0.40, -0.20);
    } else if (Robot.getLimeLightX() <= -.5) { // target is left of center, so turn left
      // this turns left?
      Robot.drivetrain.drive(-0.20, -0.40);
    } else if(Robot.getLimeLightX() < .5 && Robot.getLimeLightX() > -.5) {
      // centered, so drive straight
      // we lose sight of both targets when area is around 8% (for now, this will probably change)
      // on test bot, that's when bumpers are about 6 inches from touching target
      // so drive 6 inches more
      if(area < 13.0) {
        Robot.drivetrain.drive(-0.30, -0.30);
      } else {
        isFinished = true;
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
