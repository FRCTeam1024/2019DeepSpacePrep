/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.KeyPoint;
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
import frc.robot.commands.*;

import frc.robot.grip.*;

public class Robot extends TimedRobot {
	public static Drivetrain drivetrain = new Drivetrain();
	public static Lift lift = new Lift();
	public static Intake intake = new Intake();
	public static PowerDistributionPanel pdp = new PowerDistributionPanel();
	public static OI oi;
	
	Command m_autonomousCommand;
	Command turnTargetCommand;
	SendableChooser<String> autoChooser = new SendableChooser<String>();

	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	
	private VisionThread visionThread;
	private static double centerX1 = 0.0;
	private static double centerX2 = 0.0;
	private static double numCameraObjects = 0.0;
	private RobotDrive drive;
	
	private final Object imgLock = new Object();

	@Override
	public void robotInit() {
		oi = new OI();

		m_autonomousCommand = new SimpleTurn();
		
		try {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
			//camera.setResolution(160, 120);
			// camera.setExposureManual(50);
			// camera.setBrightness(50);
			// camera.setWhiteBalanceManual(255);
			
			// camera.setFPS(30);

			camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

			visionThread = new VisionThread(camera, new GreenLEDReflectiveTape(), pipeline -> {

				System.out.println("in pipeline");
				numCameraObjects = 0.0;
				if (!pipeline.filterContoursOutput().isEmpty()) {
					List<MatOfPoint> contours = pipeline.filterContoursOutput();
					numCameraObjects = contours.size();
					System.out.println("in pipeline, NOT EMPTY,num objects : " + numCameraObjects);
					Rect r1 = Imgproc.boundingRect(contours.get(0));
					synchronized (imgLock) {
						centerX1 = r1.x + (r1.width / 2);
					}
					if(contours.size() > 1) {
						Rect r2 = Imgproc.boundingRect(contours.get(1));
						synchronized (imgLock) {
							centerX2 = r2.x + (r2.width / 2);
						}
					}
					
				} else {
					System.out.println("in pipeline EMPTY ");
					centerX1 = 0;
					centerX2 = 0;
				}
				
			});
			visionThread.start();

			// visionThread above worked with GripTest to detect 1 object;
			// but it was also detecting 'junk' objects, i.e. other than the yellow square
			// we wanted, so we changed the Pipeline in GRIP to detect a blob, which is
			// below, but not yet working because we don't know yet how to process the 
			// blob pipeline output
/*
			visionThread = new VisionThread(camera, new GripBlobPipeline(), pipeline -> {

				if (!pipeline.findBlobsOutput().empty()){
					MatOfKeyPoint mat = pipeline.findBlobsOutput();
					List<KeyPoint> keypoints = mat.toList();
					synchronized (imgLock) {
						// centerX1 = r.x + (r.width / 2);
					}
				}

			});
*/
		} catch(Exception e) {
			e.printStackTrace();
		}
		
	
	}
	
	public static double getCenterX1(){
		return centerX1;
	}

	public static double getCenterX2() {
		return centerX2;
	}

	public static double getNumImageObjects() {
		return numCameraObjects;
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
		double centerX1;
		synchronized (imgLock) {
			centerX1 = this.centerX1;
		}
		SmartDashboard.putNumber("Num Image Objects", numCameraObjects);
		SmartDashboard.putNumber("Image 1 Center X", centerX1);
		SmartDashboard.putNumber("Image 2 Center X", centerX2);
		// double turn = centerX1 - (IMG_WIDTH / 2);
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

		turnTargetCommand = new TurnToTarget();
	}
	
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		//intake.setCubeLight();
		//drivetrain.outputToSmartDashboard();
		//lift.outputToSmartDashboard();
		//intake.outputToSmartDashboard();
		//intake.cubeLight.set(Relay.Value.kForward);

		//turnTargetCommand.start();
		double centerX1;
		synchronized (imgLock) {
			centerX1 = this.centerX1;
		}
		
	}
	
	@Override
	public void testPeriodic() {
		
  }
}