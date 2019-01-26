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
import org.opencv.core.Size;
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
	Command driveTargetCommand;
	SendableChooser<String> autoChooser = new SendableChooser<String>();

	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	
	private VisionThread visionThread;
	private static double centerX1 = 0.0;
	private static double centerX2 = 0.0;
	private static double centerY1 = 0.0;
	private static double centerY2 = 0.0;
	private static double r1X;
	private static double r2X;
	private static double r1Y;
	private static double r2Y;
	private static Size r1Size;
	private static int r1Width;
	private static int r2Width;
	private static int r1Height;
	private static int r2Height;

	private static double numCameraObjects = 0.0;
	private RobotDrive drive;
	
	private static final Object imgLock = new Object();

	@Override
	public void robotInit() {
		oi = new OI();
		m_autonomousCommand = new TurnToTarget();
		
		try {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);

			//camera.setResolution(160, 120);
			camera.setExposureManual(15);
			camera.setBrightness(50);
			camera.setWhiteBalanceManual(255);

			camera.setFPS(30);

			camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
			outputCameraToSmartDashboard();
			visionThread = new VisionThread(camera, new ReflectiveTapeTest(), pipeline -> {

				// System.out.println("in pipeline");
				numCameraObjects = 0.0;
				if (!pipeline.filterContoursOutput().isEmpty()) {
					List<MatOfPoint> contours = pipeline.filterContoursOutput();
					numCameraObjects = contours.size();
					System.out.println("in pipeline, NOT EMPTY,num objects : " + numCameraObjects);
					// System.out.println(contours.get(0));
					Rect r1 = Imgproc.boundingRect(contours.get(0));			
					// System.out.println("r1 : " + r1);
					synchronized (imgLock) {
						r1Width = r1.width;
						centerX1 = r1.x + (r1Width / 2);
						r1X = r1.x;
						r1Y = r1.y;
						r1Height = r1.height;
						r1Size = r1.size();
						centerY1 = r1.y + (r1.height / 2);
						r1Size = r1.size();
						// System.out.println("r1 size:" + r1Size);
					}
					if(contours.size() > 1) {
						Rect r2 = Imgproc.boundingRect(contours.get(1));
						// System.out.println("r2 : " + r2);

						synchronized (imgLock) {
							r2Width = r2.width;
							centerX2 = r2.x + (r2.width / 2);
							r2X = r2.x;
							r2Y = r2.y;
							r2Height = r2.height;
							centerY2 = r2.y + (r2.height / 2);

						}

						// make sure to assign the left-most image to centerX1
						if(centerX2 < centerX1) {
							synchronized (imgLock) {
								double tempSave = centerX1;
								centerX1 = centerX2;
								centerX2 = tempSave;
								tempSave = r1X;
								r1X = r2X;
								r2X = tempSave;
								tempSave = r1Y;
								r1Y = r2Y;
								r2Y = tempSave;
								int tempSaveInt = r1Width;
								r1Width = r2Width;
								r2Width = tempSaveInt;
								tempSaveInt = r1Height;
								r1Height = r2Height;
								r2Height = tempSaveInt;

								// System.out.println("CenterX2 < CenterX1");
							}
						}
					} else { // only 1 object 
						synchronized (imgLock) {
							centerX2 = 0;
							r2Width = 0;
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
			outputCameraToSmartDashboard();

		} catch(Exception e) {
			e.printStackTrace();
		}
		
	
	}
	
	public static synchronized double getCenterX1(){
		synchronized (imgLock) {
		return centerX1;
		}
	}

	public static synchronized double getCenterX2() {
		synchronized (imgLock) {
		return centerX2;
		}
	}

	public static synchronized double getCenterY1() {
		synchronized (imgLock) {
		return centerY1;
		}
	}
	public static synchronized double getCenterY2() {
		synchronized (imgLock) {
		return centerY2;
		}
	}
	public static double getNumImageObjects() {
		return numCameraObjects;
	}
	public static double getEncoderValue() {
		return drivetrain.encoder.getDistance();
	}
	public static double getr1Width() {
		return r1Width;
	}
	public static double getr2Width(){
		return r2Width;
	}
	public static double getR1Area() {
		return (r1Height *r1Width);
	}
	public static double getR2Area(){
		return (r2Height*r2Width);
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

		outputCameraToSmartDashboard();
		// double turn = centerX1 - (IMG_WIDTH / 2);
		// drive.arcadeDrive(-0.6, turn * 0.005);
	}

	private void outputCameraToSmartDashboard() {
		// display values from camera
		double centerX1;
		synchronized (imgLock) {
			centerX1 = this.centerX1;
		}
		SmartDashboard.putNumber("Num Image Objects", numCameraObjects);
		SmartDashboard.putNumber("Image 1 Center X", centerX1);
		SmartDashboard.putNumber("Image 2 Center X", centerX2);
		// SmartDashboard.putNumber("Image 1 Center Y", centerY1);
		// SmartDashboard.putNumber("Image 2 Center Y", centerY2);
		// SmartDashboard.putNumber("r1.x", r1X);
		// SmartDashboard.putNumber("r2.x", r2X);
		// SmartDashboard.putNumber("r1.y", r1Y);
		// SmartDashboard.putNumber("r2.y", r2Y);
		SmartDashboard.putNumber("r1Width", r1Width);
		SmartDashboard.putNumber("r2Width", r2Width);
		//SmartDashboard.putString("size", "size(" + r1Size.width + ", " + r1Size.height + ")");
		SmartDashboard.putNumber("Encoder Value:", getEncoderValue());
		SmartDashboard.putNumber("R1 Area", getR1Area());
		SmartDashboard.putNumber("R2 Area", getR2Area());
		SmartDashboard.putNumber("Angle", drivetrain.getHeading());
		SmartDashboard.putData("Reset Gyro", new resetGyro());
		SmartDashboard.putData("Turn To Target", new TurnToTarget());

	}
	@Override
	public void teleopInit() {
		
		// drivetrain.setBrake();
		// lift.disengageAirBag();
		// intake.setCubeLight();
		// This makes sure that the autonomous stops running when teleop starts running.
		// If you want the autonomous to continue until interrupted by another command,
		// remove this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		//intake.cubeLight.set(Relay.Value.kForward);

		turnTargetCommand = new TurnToTarget();
		driveTargetCommand = new DriveToTargetStraight(0.15, 0.15);
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
		outputCameraToSmartDashboard();
		
	}
	
	@Override
	public void testPeriodic() {
		
  }
}