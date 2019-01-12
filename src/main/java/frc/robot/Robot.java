/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Intake;

import frc.robot.grip.GripTest;

public class Robot extends TimedRobot {
	public static Drivetrain drivetrain = new Drivetrain();
	public static Lift lift = new Lift();
	public static Intake intake = new Intake();
	public static PowerDistributionPanel pdp = new PowerDistributionPanel();
	public static OI oi;
	
	Command m_autonomousCommand;
	SendableChooser<String> autoChooser = new SendableChooser<String>();

	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	
	private VisionThread visionThread;
	private double centerX = 0.0;
	private RobotDrive drive;
	
	private final Object imgLock = new Object();

	@Override
	public void robotInit() {
		oi = new OI();
		
		try {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			//camera.setResolution(160, 120);
			// camera.setExposureManual(50);
			// camera.setBrightness(50);
			// camera.setWhiteBalanceManual(255);
			
			// camera.setFPS(30);

			camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
			visionThread = new VisionThread(camera, new GripTest(), pipeline -> {

				if (!pipeline.filterContoursOutput().isEmpty()) {
					Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
					synchronized (imgLock) {
						centerX = r.x + (r.width / 2);
					}
				}

			});
			visionThread.start();

			// visionThread above worked with GripTest to detect 1 object;
			// but it was also detecting 'junk' objects, i.e. other than the yellow square
			// we wanted, so we changed the Pipeline in GRIP to detect a blob, which is
			// below, but not yet working because we don't know yet how to process the 
			// blob pipeline output

			// visionThread = new VisionThread(camera, new GripBlobPipeline(), pipeline -> {

			// 	if (!pipeline.findBlobsOutput().empty()){
			// 		MatOfKeyPoint mat = pipeline.findBlobsOutput();
			// 		mat.size();
			// 		Rect r = Imgproc.boundingRect(pipeline.findBlobsOutput());
			// 		synchronized (imgLock) {
			// 			centerX = r.x + (r.width / 2);
			// 		}
			// 	}

			// });

		} catch(Exception e) {
			e.printStackTrace();
		}
		
	
	}
	
	@Override
	public void disabledInit() {
		//drivetrain.setCoast();
		//intake.setCubeLight();
		//intake.posIn();
		//intake.slideIn();
		//lift.clamp(false);
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		//intake.setCubeLight();
		//drivetrain.outputToSmartDashboard();
		//lift.outputToSmartDashboard();
		//intake.outputToSmartDashboard();
	}
	
	@Override
	public void autonomousInit() {
		
		
		
		//Robot.drivetrain.resetOpticalEncoder();
		//Robot.drivetrain.resetGyro();
		
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}
	
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		//intake.setCubeLight();
		//drivetrain.outputToSmartDashboard();
		//lift.outputToSmartDashboard();
		//intake.outputToSmartDashboard();

		// display values from camera
		double centerX;
		synchronized (imgLock) {
			centerX = this.centerX;
		}
		SmartDashboard.putNumber("Image Center X", centerX);
		// double turn = centerX - (IMG_WIDTH / 2);
		// drive.arcadeDrive(-0.6, turn * 0.005);
	}

	@Override
	public void teleopInit() {
		//drivetrain.setBrake();
		//lift.disengageAirBag();
		//intake.setCubeLight();
		// This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to continue until interrupted by another command, remove this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		//intake.cubeLight.set(Relay.Value.kForward);
	}
	
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		//intake.setCubeLight();
		//drivetrain.outputToSmartDashboard();
		//lift.outputToSmartDashboard();
		//intake.outputToSmartDashboard();
		//intake.cubeLight.set(Relay.Value.kForward);
	}
	
	@Override
	public void testPeriodic() {
		
  }
}