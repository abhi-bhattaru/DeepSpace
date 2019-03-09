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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Joystick;

import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;

import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Relay;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.vision.VisionThread;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;

import java.time.LocalDateTime;
import java.util.*;

import javax.lang.model.util.ElementScanner6;

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

  //servos
  final int FrontLegRightStopServoPort = 4;
  final int FrontLegLeftStopServoPort = 5;
  final int RearLegStopServoPort = 6;

  	//ports
	final int leftDrivePwmPort = 0;
	final int rightDrivePwmPort = 1;
	final int intakePortL = 4;
  final int intakePortR = 5;

	//driveTrain
	Victor leftMotor = new Victor(leftDrivePwmPort);
  Victor rightMotor = new Victor(rightDrivePwmPort);
  DifferentialDrive chassis;
	JoystickLocations porting = new JoystickLocations();
  XboxController xbox = new XboxController(porting.xboxPort);
  Joystick joystick = new Joystick(porting.joystickPort);
  DriveTrain dtr;

  Victor GripperUpDownMotor = new Victor(2);
  Victor GripperRollerMotor = new Victor(3);

  DigitalInput limitSwitch = new DigitalInput(0);

  Servo FrontLegRightStopServo = new Servo(FrontLegRightStopServoPort);
  Servo FrontLegLeftStopServo = new Servo(FrontLegLeftStopServoPort);
  Servo RearLegStopServo = new Servo(RearLegStopServoPort);


  List<Double> liftHeights = new ArrayList<Double>();

  Victor intakeLeft = new Victor(intakePortL);
	Victor intakeRight = new Victor(intakePortR);
  
  int currentHeightSelection = 0;
  Encoder encoder1 = new Encoder(1,2);

  double intakeSpeed=1.0;
  double outtakeSpeed=1.0;
  
  SpeedControllerGroup intake;

  boolean rollerEnabled = false;

  Relay light;
  
  boolean isDockingMode;
  final double isOnCenterThresholdInches = .1; //Arbitrary value will need to be calibrated

  DoubleSolenoid GripperDoubleSolenoid;
  DoubleSolenoid frontLeft;
  DoubleSolenoid frontRight;
  DoubleSolenoid rearLeg;
  DoubleSolenoid kickerDoubleSolenoid;
  Solenoid GripperRollerSolenoid;
  int[] forwardLegsPorts = {0,2,4};
  int[] reverseLegsPorts = {1,3,5};

  long kickDelayMS = 200;
  boolean isKick = false;
  long kickTimeMS =0;

  //DoubleSolenoid Gripper;
  //DoubleSolenoid Roller;
  //DoubleSolenoid Kicker;
  int[] forwardScoringPorts = {0,2,4};
  int[] reverseScoringPorts = {1,3,5};

  int legsPCMPort = 0;
  int scoringPCMPort = 1;

  Compressor c;

  //Pistons pistons;

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


      // define static heights
      liftHeights.add(0.0); //0
      liftHeights.add(2.0); //1
      liftHeights.add(7.5); //2 
      liftHeights.add(14.6); //3
      liftHeights.add(18.33); //4

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
    //visionThread.start();

    //frontLeft = new DoubleSolenoid(legsPCMPort, forwardLegsPorts[0], reverseLegsPorts[0] );
    //frontRight = new DoubleSolenoid(legsPCMPort, forwardLegsPorts[1], reverseLegsPorts[1] );
    //rearLeg = new DoubleSolenoid(legsPCMPort, forwardLegsPorts[2], reverseLegsPorts[2] );

    /*Gripper = new DoubleSolenoid(scoringPCMPort, forwardScoringPorts[0], reverseScoringPorts[0]);
    Roller = new DoubleSolenoid(scoringPCMPort, forwardScoringPorts[1], reverseScoringPorts[1]);
    Kicker = new DoubleSolenoid(scoringPCMPort, forwardScoringPorts[2], reverseScoringPorts[2]);

    c = new Compressor(0);
    
    pistons = new Pistons(frontLeft, frontRight, rearLeg, Gripper, Roller, Kicker, c);
    pistons.initPistons();*/

    GripperDoubleSolenoid = new DoubleSolenoid(0,0,1);
    GripperRollerSolenoid = new Solenoid(0,2);
    
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
    dtr.joystickDrive();

    //gripper
    if (xbox.getTriggerAxis(Hand.kLeft)> 0)
    {
      GripperDoubleSolenoid.set(DoubleSolenoid.Value.kForward); //close
    }
    else if ( xbox.getTriggerAxis(Hand.kRight)> 0)
    {
      GripperDoubleSolenoid.set(DoubleSolenoid.Value.kReverse); //open
    }
    else 
    {
      GripperDoubleSolenoid.set(DoubleSolenoid.Value.kOff);
    }
    
    
    rollerEnabled = xbox.getBumperPressed(Hand.kLeft);
    if (rollerEnabled == true)
    {
      GripperRollerSolenoid.set(true);
      GripperRollerMotor.set(.1);
    }
    
    if(isKick==false && xbox.getBumperPressed(Hand.kRight))
    {
      isKick = true;
      kickTimeMS = System.currentTimeMillis();
      GripperDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    else if (isKick ==true && System.currentTimeMillis()>=(kickTimeMS+kickDelayMS))
    {
      kickerDoubleSolenoid.set(DoubleSolenoid.Value.kForward); 
      isKick = false;
      kickTimeMS=0;
    }
    else
    {
      kickerDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    //read lift height based on button press
    if(joystick.getTopPressed())
    {
      currentHeightSelection++;
    }
    else if (joystick.getTriggerPressed())
    {
      currentHeightSelection--;
    }

    //Verify the height selection is valid
    if(currentHeightSelection < 0)
    {
      currentHeightSelection = 0;
    }
    else if (currentHeightSelection+1 > liftHeights.size())
    {
      currentHeightSelection = liftHeights.size();
    }

    SmartDashboard.putNumber("Lift Height Selection", currentHeightSelection);

    ChangeHeight();

    /*if(xbox.getYButton()){
      pistons.dropLegs();
    }else{
      pistons.retractLegs();
    }

    if(xbox.getAButton()){
      pistons.enableScoring();
    }else{
      pistons.disableScoring();
    }*/

    manualDriveConditions();

    // if(xbox.getBumper(Hand.kRight))
    // {
    //   lineAlignment();
    // }
    // else
    // {
    //   manualDriveConditions();
    // }

  }

  public void ChangeHeight(){

      if (limitSwitch.get()==true)
      {
        encoder1.reset();
        currentHeightSelection = 0;
        return;
      }
      // get current heights
      double targetHeight = liftHeights.get(currentHeightSelection);
      double currentHeight = encoder1.getDistance(); // todo: read from encoder

      // apply movement
      double heightTolerance = 0.25; 
      if (currentHeight < targetHeight - heightTolerance)
      {
        GripperUpDownMotor.set(.1);
      }
      else if (currentHeight > targetHeight + heightTolerance)
      {
        GripperUpDownMotor.set(-.1);
      }

  }

  public void Habitat(){
    
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
