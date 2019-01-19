/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.Robot;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.command.Command;

public class MoveLiftWithJoysticks extends Command {
  public MoveLiftWithJoysticks() {
    requires(Robot.lift);
  }

  protected void initialize() {
  }

  protected void execute() {
    if(Robot.lift.getCommandedOutput() > 0.0) {
      if (Robot.intake.intakeWideState() == false) {
        Robot.lift.clamp(false);
      }
          
      if (Robot.lift.getLiftEncoderValue() < 25000 /*&& !Robot.oi.getOverrideButton()*/) {
          Robot.lift.configMaxOutputs(1.0);
      } else {
        Robot.lift.configMaxOutputs(0.25);
      }
    } else if(Robot.lift.getCommandedOutput() < 0.0) {
      if (Robot.intake.intakeWideState() == false) {
        Robot.lift.clamp(false);
      }
      if (Robot.lift.getLiftEncoderValue() > 3000 /*&& !Robot.oi.getOverrideButton()*/) {
        Robot.lift.configMaxOutputs(1.0);
      } else {
        Robot.lift.configMaxOutputs(0.25);
      }
    } else {
      Robot.lift.configMaxOutputs(1.0);
    }
    Robot.lift.moveCarriage(Robot.oi.logi.getRawAxis(Constants.LIFT_STICK_AXIS));
  }

  protected boolean isFinished() {
      return false;
  }

  protected void end() {
  }

  protected void interrupted() {
  }
}
