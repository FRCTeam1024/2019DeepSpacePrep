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

public class TurnToTarget extends Command {

  boolean isFinished = false;

  public TurnToTarget() {
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   /* if(Robot.oi.lJoy.getTriggerPressed()){
      isFinished = true;
    }*/


    double targetX = 160;
    
   double difX1 = targetX - Robot.getCenterX1();
   double difX2 = targetX - Robot.getCenterX2();
    System.out.println("Num Objects : " + Robot.getNumImageObjects());
    if(Robot.getNumImageObjects() < 2) {
      if(Robot.getdifX1() < 0) { // single target on right side, so we want to turn right
        // Robot.drivetrain.turn(0.15);
        Robot.drivetrain.drive(-0.40, -0.20);
      }
      else if(Robot.getdifX1() > 0) {//  single target on left side, so we want to turn left
        Robot.drivetrain.drive(-0.20, -0.40);
        //Robot.drivetrain.turn(-0.15);
      }
    // } else if(Robot.getr1Width() - Robot.getr2Width() <= 5 ) {
    } else if(Robot.getNumImageObjects() == 2) {
      if(Robot.getdifX1() > 0 && Robot.getdifX2() < 0) {
        Robot.drivetrain.drive(-0.10, -0.10);
          //Robot.drivetrain.turn(0.0);
        isFinished = true;
      }
      else if(Robot.getdifX1() < 0 && Robot.getdifX2() < 0){
        Robot.drivetrain.drive(-0.40, -0.20);
      }
      else if(Robot.getdifX1() > 0 && Robot.getdifX2() > 0){
        Robot.drivetrain.drive(-0.20, -0.40);
      }
      }
      


    }
     /* System.out.println("Difx1: " + difX1);
      System.out.println("Difx2: " + difX2);
      System.out.println("Center X 1: " + Robot.getCenterX1());
      System.out.println("Center X 2: " + Robot.getCenterX2());
      if(Math.abs(difX1 - difX2) < 10) {
        System.out.println("FINISHING TURN TO TARGET");
        Robot.drivetrain.turn(0);
        isFinished = true;
      } else {
        Robot.drivetrain.turn(0.50);
      }*/
    
    
  

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
