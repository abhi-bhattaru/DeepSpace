/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;


import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;

import edu.wpi.first.wpilibj.DoubleSolenoid;

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

  Joystick stick;
  Thread m_visionThread;


  private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	
	private VisionThread visionThread;
  private double centerX = 0.0;
  private double centerX1 = 0.0;
  private double centerX2 = 0.0;
	
  private final Object imgLock = new Object();

  DoubleSolenoid rearLift = new DoubleSolenoid(0, 0, 1);
  DoubleSolenoid frontLeftLift = new DoubleSolenoid(0, 2, 3); // todo: how to index second PCM node id?
  DoubleSolenoid frontRightLift = new DoubleSolenoid(0, 4, 5);
  
  Compressor compressor = new Compressor(0);


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);


    stick = new Joystick(0);


    compressor.start();
    compressor.setClosedLoopControl(true);

    rearLift.set(DoubleSolenoid.Value.kReverse);
    frontLeftLift.set(DoubleSolenoid.Value.kReverse);
    frontRightLift.set(DoubleSolenoid.Value.kReverse);

    
  }
  
  public void selenoidTest()
  {
    /**
   * This code is for operating the pneumatic encoders/cylinders
   */

   // initialize compressors
   Compressor c = new Compressor(0);

   c.setClosedLoopControl(true);
   c.setClosedLoopControl(false);

  // initialize selenoids
   DoubleSolenoid solenoid1 = new DoubleSolenoid(0, 1, 2);
   DoubleSolenoid solenoid2 = new DoubleSolenoid(1, 1, 2); // todo: how to index second PCM node id?

  }
  
  public void visionTest()
  {
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    /*visionThread = new VisionThread(camera, new TapePipeline(), pipeline -> {
        if (!pipeline.filterContoursOutput().isEmpty()) {
            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
            synchronized (imgLock) {
                centerX = r.x + (r.width / 2);
            }
        }
    });
    visionThread.start();
    */
    visionThread = new VisionThread(camera, new TapePipeline(), pipeline -> {
      if(pipeline.filterContoursOutput().isEmpty()) {
        synchronized (imgLock) {
        centerX = 160;
        }
      }
        if (!pipeline.filterContoursOutput().isEmpty()) {
            Rect r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
            if(pipeline.filterContoursOutput().size() >= 2) {
              Rect r2 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
              synchronized(imgLock) {
                centerX1 = r1.x + (r1.width / 2);
                centerX2 = r2.x + (r2.width/2);
                centerX = (centerX1 + centerX2)/2;
              }
              SmartDashboard.putNumber("centerX", centerX);
              SmartDashboard.putNumber("centerX1", centerX1);
              SmartDashboard.putNumber("centerX2", centerX2);
            } else {
                synchronized (imgLock) {
                  centerX = 0;
                }
            }
        }
    });
    visionThread.start();;
    
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
        double centerX;
        synchronized (imgLock) {
            centerX = this.centerX;
        }
        double turn = centerX - (IMG_WIDTH / 2);
        SmartDashboard.putNumber("turnAmount", turn);
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

    
    double y = stick.getY();

    if (y < 0)
    {
      frontLeftLift.set(DoubleSolenoid.Value.kReverse);
      rearLift.set(DoubleSolenoid.Value.kReverse);
      frontRightLift.set(DoubleSolenoid.Value.kReverse);
    }
    else if (y > 0)
    {
      frontLeftLift.set(DoubleSolenoid.Value.kForward);
      rearLift.set(DoubleSolenoid.Value.kForward);
      frontRightLift.set(DoubleSolenoid.Value.kForward);

    }
      

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
