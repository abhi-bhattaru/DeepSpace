/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//test
package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private int kfrontleft = 0;
  private int kfrontRight = 1;
  private int krearleft = 2;
  private int krearRight = 3;

  private int kJoystickPort = 0;

  Victor frontLeft;
  Victor frontRight;
  Victor rearLeft;
  Victor rearRight;

  MecanumDrive chassis;

  Joystick stick;


  private static final int kUltrasonicPort1 = 0;
	private AnalogInput m_ultrasonic1 = new AnalogInput(kUltrasonicPort1);
  private static final double kValueToInches = 0.049;

  ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  Accelerometer accel= new BuiltInAccelerometer(Accelerometer.Range.k4G);
  

  Relay light;
  
	double currentDistance1;
	double[] accVal = new double[3];

  Thread m_visionThread;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    frontLeft = new Victor(kfrontleft);
    frontRight = new Victor(kfrontRight);
    rearLeft = new Victor(krearleft);
    rearRight = new Victor(krearRight);

    frontLeft.setInverted(true);
    rearRight.setInverted(true);

    chassis = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    stick = new Joystick(kJoystickPort);

    light = new Relay(0);
    light.set(Value.kOn);

    m_visionThread = new Thread(() -> {
      // Get the UsbCamera from CameraServer
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      // Set the resolution
      camera.setResolution(240, 180);

      // Get a CvSink. This will capture Mats from the camera
      CvSink cvSink = CameraServer.getInstance().getVideo();
      // Setup a CvSource. This will send images back to the Dashboard
      CvSource outputStream
          = CameraServer.getInstance().putVideo("Rectangle", 320, 240);

      // Mats are very memory expensive. Lets reuse this Mat.
      Mat mat = new Mat();

      // This cannot be 'true'. The program will never exit if it is. This
      // lets the robot stop this thread when restarting robot code or
      // deploying.
      while (!Thread.interrupted()) {
        // Tell the CvSink to grab a frame from the camera and put it
        // in the source mat.  If there is an error notify the output.
        if (cvSink.grabFrame(mat) == 0) {
          // Send the output the error.
          outputStream.notifyError(cvSink.getError());
          // skip the rest of the current iteration
          continue;
        }
        // Put a rectangle on the image
        Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400),
            new Scalar(255, 255, 255), 5);
        // Give the output stream a new image to display
        outputStream.putFrame(mat);
      }
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double x = stick.getX();
    double y = -stick.getY();
    double z = stick.getZ();

    getAccel();
    getHeading();
    getDistance();

    chassis.driveCartesian(y, x, z);
    light.set(Value.kForward);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  	//updates the value of the sonar sensor
	public void getDistance() {
		this.currentDistance1 = m_ultrasonic1.getValue()*this.kValueToInches;
		SmartDashboard.putNumber("distance right", currentDistance1);
	}
	

	//updates the value of the gyro
	public double getHeading() {
		SmartDashboard.putNumber("gyroAngle", gyro.getAngle());
		return gyro.getAngle();
	}
	
	//updates the value of the accelerometer Ignore for now
	public double[] getAccel() {
		accVal[0] = accel.getX()*9.81;
		accVal[1] = accel.getY()*9.81;
		accVal[2] = accel.getZ()*9.81;
		SmartDashboard.putNumber("x accel", accVal[0]);
		SmartDashboard.putNumber("y accel", accVal[1]);
		SmartDashboard.putNumber("z accel", accVal[2]);
		return accVal;
	}
}
