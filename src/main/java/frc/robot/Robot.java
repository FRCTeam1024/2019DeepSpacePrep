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
import edu.wpi.first.wpilibj.I2C;

import java.io.*;
import java.util.logging.*;

import edu.wpi.first.wpilibj.Filesystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.logging.*;
import frc.robot.grip.*;

public class Robot extends TimedRobot {
	public static Drivetrain drivetrain = new Drivetrain();
	public static Sensors sensors = new Sensors();
	public static Lift lift = new Lift();
	public static Intake intake = new Intake();
	public static PowerDistributionPanel pdp = new PowerDistributionPanel();
	public static OI oi;
	
	Command m_autonomousCommand;
	Command turnTargetCommand;
	Command driveTargetCommand;
	Command driveCommand;
	
	SendableChooser<String> autoChooser = new SendableChooser<String>();

	// define the logger for this class. This should be done for every class
    private static LogWrapper mLog;

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

	private static double LimeX;

	private static double numCameraObjects = 0.0;
	private RobotDrive drive;
	
	private static final Object imgLock = new Object();

	private static int cameraMode = 0;

	@Override
	public void robotInit() {
		oi = new OI();
		m_autonomousCommand = new TurnToTarget();
		NetworkTableEntry tx;
		NetworkTableEntry ty;
		NetworkTableEntry ta;
		Robot.sensors.startColorSensor();
		initLogging();
		
		try {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
			//NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		


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
					mLog.info("in pipeline, NOT EMPTY,num objects : " + numCameraObjects);
					//System.out.println("in pipeline, NOT EMPTY,num objects : " + numCameraObjects);
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
					//System.out.println("in pipeline EMPTY ");
					centerX1 = 0;
					centerX2 = 0;
				}
				
			});
			visionThread.start();
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

	private void initLogging() {
		HelixEvents.getInstance().startLogging();
		// try {
        //     File dir = Filesystem.getDeployDirectory();
        //     String logFile = "NOTFOUND";
        //     if (dir.isDirectory() && dir.exists()) {
        //         logFile = dir.getAbsolutePath() + "/logging.properties";
        //     } else {
		// 		System.out.println("Directory not found.");
		// 	}
        //     System.out.println("**********  logConfig: " + logFile + "  *********************");
        //     // BufferedReader br = new BufferedReader(new FileReader(logFile));
        //     // String line = null;
        //     // while ((line = br.readLine()) != null) {
        //     //     System.out.println(line);
        //     // }
        //     // br.close();
        //     FileInputStream configFile = new FileInputStream(logFile);
        //     LogManager.getLogManager().readConfiguration(configFile);
        // } catch (IOException ex) {
        //     System.out.println("WARNING: Could not open configuration file");
        //     System.out.println("WARNING: Logging not configured (console output only)");
        // }
		// mLog = new LogWrapper(Robot.class.getName());
        // try {
        //     mLog.info("robotInit: ---------------------------------------------------");
        //     // mControlBoard = ControlBoard.getInstance();
        //     // mDriveSys = DriveSys.getInstance();
        //     // mNavXsys = NavXSys.getInstance();
        //     // mDriveDistCmd = new DriveDistCmd(5);
        //     mLog.info("robotInit: Completed   ---------------------------------------");
        // } catch (Exception ex) {
        //     mLog.severe(ex, "Robot.robotInit:  exception: " + ex.getMessage());
        // }
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
	public static double getdifX1(){
		double targetX = 160;
		double difX1 = 0;
		if(getNumImageObjects() == 1){
		difX1 = targetX - Robot.getCenterX1();
		}
		return difX1;
	}
	public static double getdifX2(){
		double targetX = 160;
		double difX2 = 0;
		if(getNumImageObjects() > 1){
		 difX2 = targetX - Robot.getCenterX2();
		}
		return difX2;
		
	}
	public static int switchCameraMode(){
		if(cameraMode == 0){
			cameraMode = 1;
		}else if(cameraMode == 1){
			cameraMode = 0;
		}
		return cameraMode;
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
		//SmartDashboard.putNumber("Num Image Objects", numCameraObjects);
		//SmartDashboard.putNumber("Image 1 Center X", centerX1);
		//SmartDashboard.putNumber("Image 2 Center X", centerX2);
		// SmartDashboard.putNumber("Image 1 Center Y", centerY1);
		// SmartDashboard.putNumber("Image 2 Center Y", centerY2);
		// SmartDashboard.putNumber("r1.x", r1X);
		// SmartDashboard.putNumber("r2.x", r2X);
		// SmartDashboard.putNumber("r1.y", r1Y);
		// SmartDashboard.putNumber("r2.y", r2Y);
		//SmartDashboard.putNumber("r1Width", r1Width);
		//SmartDashboard.putNumber("r2Width", r2Width);
		//SmartDashboard.putString("size", "size(" + r1Size.width + ", " + r1Size.height + ")");
		//SmartDashboard.putNumber("Encoder Value:", getEncoderValue());
		//SmartDashboard.putNumber("R1 Area", getR1Area());
		//SmartDashboard.putNumber("R2 Area", getR2Area());
		SmartDashboard.putNumber("Angle", drivetrain.getHeading());
		SmartDashboard.putData("Reset Gyro", new resetGyro());
		//SmartDashboard.putNumber("difx1", getdifX1());
		//SmartDashboard.putNumber("difx2", getdifX2());
		SmartDashboard.putData("CurveToTarget", new TurnToTarget());
		SmartDashboard.putData("Change Camera Mode", new SwitchCameraMode());
		SmartDashboard.putData("TurnToCenterLimelight", new TurnToCenterLimelight());
		//Robot.sensors.printValue();

		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx;
		NetworkTableEntry ty;
		NetworkTableEntry ta;
		
		tx = table.getEntry("tx");
		ty = table.getEntry("ty");
		ta = table.getEntry("ta");
			//read values periodically
			LimeX = tx.getDouble(0.0);
			double y = ty.getDouble(0.0);
			double area = ta.getDouble(0.0);

			//post to smart dashboard periodically
			SmartDashboard.putNumber("LimelightX", LimeX);
			SmartDashboard.putNumber("LimelightY", y);
			SmartDashboard.putNumber("LimelightArea", area);
	}
	
	public static double getLimeLightX(){
		return LimeX;
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
		
		
		//driveTargetCommand = new DriveToTargetStraight(0.15, 0.15);
		

	}
	
	@Override
	public void teleopPeriodic() {

		if(Robot.drivetrain.lookForGroundTape() == true){
		//LED = GREEN
		// System.out.println("LED ON");
		}
		else{
		//LED = OFF
		// System.out.println("LED OFF");
		}
		Scheduler.getInstance().run();
		outputCameraToSmartDashboard();

		
	
		//intake.setCubeLight();
		drivetrain.outputToSmartDashboard();
		//lift.outputToSmartDashboard();
		//intake.outputToSmartDashboard();
		//intake.cubeLight.set(Relay.Value.kForward);

		//turnTargetCommand.start();
		if(Robot.oi.lJoy.getTriggerPressed()){
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("visionProcessingOn").setNumber(0);
		}
		if(Robot.oi.rJoy.getTriggerPressed()){
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("driverCamOn").setNumber(1);
		}
		outputCameraToSmartDashboard();
	
	}
	
	@Override
	public void testPeriodic() {
		
  }
}