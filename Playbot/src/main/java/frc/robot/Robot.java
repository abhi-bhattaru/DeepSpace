/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;

import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Relay;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.vision.VisionThread;

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

  //Vision
  Thread m_visionThread;
  private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;	
	private VisionThread visionThread;
  private double centerX = 0.0;
  private double centerX1 = 0.0;
  private double centerX2 = 0.0;	
  private final Object imgLock = new Object();

  
  	//ports
	final int leftDrivePwmPort = 0;
	final int rightDrivePwmPort = 1;
  final int gripperUpDownPwmPort = 2;
	final int intakePortL = 4;
  final int intakePortR = 5;

	//driveTrain
	Victor leftMotor = new Victor(leftDrivePwmPort);
  Victor rightMotor = new Victor(rightDrivePwmPort);
  Victor gripperUpDownMotor = new Victor(gripperUpDownPwmPort);

  DifferentialDrive chassis;
	JoystickLocations porting = new JoystickLocations();
	XboxController xbox = new XboxController(porting.joystickPort);
  DriveTrain dtr;

  Victor intakeLeft = new Victor(intakePortL);
	Victor intakeRight = new Victor(intakePortR);
  
  int currentHeightSelection;
  Encoder encoder1 = new Encoder(1,2);

  double intakeSpeed=1.0;
  double outtakeSpeed=1.0;
  
  SpeedControllerGroup intake;

  Relay light;
  
  boolean isDockingMode;
  final double isOnCenterThresholdInches = .1; //Arbitrary value will need to be calibrated
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    
    rightMotor.setInverted(true);

    chassis = new DifferentialDrive(leftMotor, rightMotor);
		chassis.setExpiration(.1);
		chassis.setSafetyEnabled(false);
    dtr = new DriveTrain(chassis, xbox, porting);

    intakeLeft.setInverted(true);
		intakeRight.setSafetyEnabled(false);
    intakeLeft.setSafetyEnabled(false);    
    intake = new SpeedControllerGroup(intakeLeft, intakeRight);

    light = new Relay(0);
    light.set(Value.kOn);

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
    visionThread.start();
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
    dtr.chassis.setSafetyEnabled(true);
    dtr.changeDrive();
    dtr.updateAxes();

    currentHeightSelection = 
    if(xbox.getBumper(Hand.kRight))
    {
      lineAlignment();
    }
    else
    {
      manualDriveConditions();
    }

    ChangeHeight();
  }

  public void ChangeHeight(){

      if (currentHeightSelection == -1)
      {
        return;
      }
      else if (currentHeightSelection == 0)
      {
        //todo: go down
        //todo: read limit switch if pressed set currentHeightSelection = 0 and rotaryencoder = 0
        currentHeightSelection = 0;
        encoder1.reset();
      }
      HashMap<int, decimal> liftHeights = new HashMap<int, decimal>();

      // define static heights
      liftHeights.put(1, 2.0);
      liftHeights.put(2, 7.5);
      liftHeights.put(3, 14.6);
      liftHeights.put(4, 18.33);


      // get current heights
      decimal targetHeight = liftHeights.get(currentHeightSelection);
      decimal currentHeight = encoder1.GetDistance(); // todo: read from encoder

      // apply movement
      decimal heightTolerance = 0.25; 
      if (currentHeight < targetHeight - heightTolerance)
      {
          //todo: move lift up
      }
      else if (currentHeight > targetHeight + heightTolerance)
      {
          //todo: move lift down
      }
      else
      {
          currentHeightSelection = -1;
          return;
      }

  }

  public void manualDriveConditions(){
      if(xbox.getRawAxis(porting.lTrigger)>.2) {
        intake.set(intakeSpeed*-xbox.getTriggerAxis(Hand.kLeft));
      }else if (xbox.getRawAxis(porting.rTrigger)>.2) {
        intake.set(outtakeSpeed*xbox.getTriggerAxis(Hand.kRight));
      }
      else
        intake.set(0);
  }

  public void lineAlignment(){
    if(xbox.getBumper(Hand.kRight))
    {
        if(isDockingMode == false){
          isDockingMode = true;
        }

        if(dtr.sonarLeft <= 1 || dtr.sonarRight <= 1)
        {
          isDockingMode = false;
        }

        if(isDockingMode)
        {
          chassis.arcadeDrive(0,0);
        }

        if(Math.abs(dtr.sonarLeft - dtr.sonarRight) < isOnCenterThresholdInches)
        {
          chassis.arcadeDrive(.2, 0);
        }
        else if (dtr.sonarLeft > dtr.sonarRight)
        {
          dtr.turnRight(15);
          dtr.continueStraight(.2);
        }
        else if (dtr.sonarLeft<dtr.sonarRight)
        {
          dtr.turnLeft(15);
          dtr.continueStraight(.2);
        }
    }
    else {
      isDockingMode = false;
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
