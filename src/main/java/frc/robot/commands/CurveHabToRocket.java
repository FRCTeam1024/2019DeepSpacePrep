/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.logging.*;

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

// S-curves from position 2, in front of the Hab, but not on it, to the rocket
public class CurveHabToRocket extends Command {

  boolean isFinished = false;
  private int currentStep = 1;
  private int encoderStart = 0;

  public CurveHabToRocket() {
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivetrain.resetGyro();
  }

  private void log(String msg) {
    HelixEvents.getInstance().addEvent("CurveHabToRocket", msg);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if(currentStep == 1) {
      if(Robot.drivetrain.getHeading() < -85 ) {
        // turning left turns the angle negative, so our target is -90
        // stop turning at 85 degrees to allow for some drift to 90?
        log("FIRST CURVE DONE");
        currentStep = 2;
        Robot.drivetrain.resetOpticalEncoder();
      }
    } else if(currentStep == 2) {
        log("STEP 2, encoder = " + Robot.drivetrain.getRawRightEncoder());
        if(Robot.drivetrain.getRawRightEncoder() > 37 ) {
          // encoder value went about 42 for 3 feet
          log("END STEP 2, setting to STEP 3");
          currentStep = 3;
        }
    } else if(currentStep == 3 ) {
      if(Robot.drivetrain.getHeading() > -40) {
        log("END STEP 3");
        isFinished = true;
      }
    }

    if (currentStep == 1) { // first curve section
      // curve left
      Robot.drivetrain.drive(-0.20, -0.60);
    } else if (currentStep == 2) { 
      // first curve done; drive straight a bit
      // encoder value went about 42 for 3 feet
      Robot.drivetrain.drive(-0.40, -0.40);
    } else if(currentStep == 3) { 
      // Curve right
      Robot.drivetrain.drive(-0.60, -0.20);
    }  
    
    // this turns left?
    // Robot.drivetrain.drive(-0.20, -0.40);

    // this turns right?
    // Robot.drivetrain.drive(-0.40, -0.20);
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
