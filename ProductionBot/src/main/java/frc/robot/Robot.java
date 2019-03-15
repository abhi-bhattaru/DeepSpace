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
import edu.wpi.first.wpilibj.Timer;
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
import java.util.function.Function;

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

  	//ports
	final int leftDrivePwmPort = 0;
	final int rightDrivePwmPort = 1;
	//final int intakePortL = 4;
  //final int intakePortR = 5;

	//driveTrain
	Victor LeftDriveMotor = new Victor(leftDrivePwmPort);
  Victor RightDriveMotor = new Victor(rightDrivePwmPort);
  DifferentialDrive chassis;
	JoystickLocations porting = new JoystickLocations();
  XboxController xbox = new XboxController(porting.xboxPort);
  Joystick joystick = new Joystick(porting.joystickPort);
  DriveTrain dtr;

  Victor GripperUpDownMotor = new Victor(2);
  Victor GripperRollerMotor = new Victor(3);

  DigitalInput limitSwitch = new DigitalInput(0);

  List<Double> liftHeights = new ArrayList<Double>();

  //Victor intakeLeft = new Victor(intakePortL);
	//Victor intakeRight = new Victor(intakePortR);
  
  int currentHeightSelection = 0;
  Encoder encoder1 = new Encoder(1,2);

  //double intakeSpeed=1.0;
  //double outtakeSpeed=1.0;
  
  //SpeedControllerGroup intake;

  boolean rollerEnabled = false;
  boolean isTestMode = false;

  Relay light;
  
  boolean isDockingMode;
  final double isOnCenterThresholdInches = .1; //Arbitrary value will need to be calibrated

  DoubleSolenoid frontLeft;
  DoubleSolenoid frontRight;
  DoubleSolenoid rearLeg;
  DoubleSolenoid kickerDoubleSolenoid;
  int[] forwardLegsPorts = {0,2,4};
  int[] reverseLegsPorts = {1,3,5};
  DoubleSolenoid GripperDoubleSolenoid = new DoubleSolenoid(0,0,1);
  Solenoid GripperRollerSolenoid = new Solenoid(0,2);

  long kickDelayMS = 200;
  boolean isKick = false;
  long kickTimeMS =0;

  int[] forwardScoringPorts = {0,2,4};
  int[] reverseScoringPorts = {1,3,5};

  int legsPCMPort = 0;
  int scoringPCMPort = 1;
  
  long elapsedTestTimeMs = 2000;
  long lastExecutionTimeMs = 0;
  int currentTestIndex = 0;
  
  List<Function<Boolean, Boolean>> listTest = new ArrayList<Function<Boolean, Boolean>>();

  boolean isStarted = false;

  Compressor c;

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
    
    RightDriveMotor.setInverted(true);

    chassis = new DifferentialDrive(LeftDriveMotor, RightDriveMotor);
		chassis.setExpiration(.1);
		chassis.setSafetyEnabled(false);
    dtr = new DriveTrain(chassis, xbox, porting);

    /*intakeLeft.setInverted(true);
		intakeRight.setSafetyEnabled(false);
    intakeLeft.setSafetyEnabled(false);    
    intake = new SpeedControllerGroup(intakeLeft, intakeRight);*/

    light = new Relay(0);
    light.set(Value.kOn);

    AddTests();

    lastExecutionTimeMs = System.currentTimeMillis() - elapsedTestTimeMs;

      // define static heights
      liftHeights.add(19.0); //0 rocket bottom hatch, cargo hatch
      liftHeights.add(27.5); //1 rocket bottom port
      liftHeights.add(47.0); //2 middle hatch
      liftHeights.add(55.5); //3 middle port
      liftHeights.add(75.0); //4 top hatch
      liftHeights.add(83.5); //5 top port
      liftHeights.add(32.0); //6 cargo port
      

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
  int state = 1;
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    dtr.chassis.setSafetyEnabled(true);
    int state = 1;
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

  public void startLeft(){
    switch (state){
      case 1: 
      dtr.chassis.setSafetyEnabled(false);
      dtr.chassis.arcadeDrive(.6, 0);
      state++;
      break;
      case 2:
      Timer.delay(2.5);
			dtr.chassis.arcadeDrive(0, 0);
			state++;
      break;
      case 3:
      dtr.chassis.arcadeDrive(.2, -90);
      break;
      case 4:
      Timer.delay(2.5);
			dtr.chassis.arcadeDrive(0, 0);
			state++;
      break;
      case 5:
      dtr.chassis.arcadeDrive(.4, 0);
      break;
      case 6:
      Timer.delay(2.5);
			dtr.chassis.arcadeDrive(0, 0);
			state++;
			break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    //First time robot is booted up
    //if(isStarted == false)
    //{        
     // SetDefault();
      //sStarted = true;
    //}
    

    //check for switching from test mode
    /*if(isTestMode==true)
    {
      isTestMode = false;
      SetDefault();
    }*/


    //First time robot is booted up
    /*if(isStarted == false)
    {        
      SetDefault();
      isStarted = true;
    }*/

    //check for switching from test mode
    /*if(isTestMode==true)
    {
      isTestMode = false;
      SetDefault();
    }*/

    dtr.chassis.setSafetyEnabled(true);
    dtr.joystickDrive();

    //endgame
    if (xbox.getYButtonPressed())
    {
      rearLeg.set(DoubleSolenoid.Value.kForward);
    }

    //gripper
    if (xbox.getXButton())
    {
      GripperDoubleSolenoid.set(DoubleSolenoid.Value.kForward); //close
    }
    else if ( xbox.getBButton())
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
      GripperRollerSolenoid.set(false);
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
    /*if(joystick.getTopPressed())
    {
      currentHeightSelection++;
    }
    else if (joystick.getTriggerPressed())
    {
      currentHeightSelection--;
    }*/

    //Verify the height selection is valid
    /*if(currentHeightSelection < 0)
    {
      currentHeightSelection = 0;
    }
    else if (currentHeightSelection+1 > liftHeights.size())
    {
      currentHeightSelection = liftHeights.size();
    }

    SmartDashboard.putNumber("Lift Height Selection", currentHeightSelection);

    ChangeHeight();*/

        Timer.delay(.002);
  }

  public void SetDefault()
  {
    GripperRollerSolenoid.set(true);
    LeftDriveMotor.set(0);
    RightDriveMotor.set(0);
    GripperUpDownMotor.set(0);
    GripperRollerMotor.set(0);
    GripperDoubleSolenoid.set(DoubleSolenoid.Value.kOff);
    kickerDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
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
 int mode =1;
  /*public void Habitat(){
    switch(mode){
      case 1:
      rearLeg.set(DoubleSolenoid.Value.kForward);
      mode++;
      break;
      case 2:
      Timer.delay(2.5);
			dtr.chassis.arcadeDrive(0, 0);
			mode++;
      break;
      case 3:
      dtr.chassis.arcadeDrive(.6, 0);
      mode++;
      break;
      case 4:
      Timer.delay(2.5);
			dtr.chassis.arcadeDrive(0, 0);
			mode++;
      break;
      case 5:
      rearLeg.set(DoubleSolenoid.Value.kReverse);
      mode++;
      break;
      case 6:
      Timer.delay(2.5);
			dtr.chassis.arcadeDrive(0, 0);
			mode++;
			break;

    }
  }*/

  public void AddTests()
  {    
    listTest.add((a) ->
    {
      SmartDashboard.putString("Diagnostic Test", "AllSolenoidsForwardUntoDawn");
      GripperDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
      kickerDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
      GripperRollerSolenoid.set(true);
      return true;
    });

    listTest.add((a) ->
    {
      SmartDashboard.putString("Diagnostic Test", "AllSolenoidsReverse");
      GripperDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      kickerDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      GripperRollerSolenoid.set(false);
      return true;
    });

    listTest.add((a) ->
    {
      SmartDashboard.putString("Diagnostic Test", "MotorsForward");
      LeftDriveMotor.set(.1);
      RightDriveMotor.set(-.1);
      GripperUpDownMotor.set(.1);
      GripperRollerMotor.set(.1);
      return true;
    });

    listTest.add((a) ->
    {
      SmartDashboard.putString("Diagnostic Test", "MotorsReverse");
      LeftDriveMotor.set(-.1);
      RightDriveMotor.set(.1);
      GripperUpDownMotor.set(-.1);
      GripperRollerMotor.set(-.1);
      return true;
    });

    listTest.add((a) ->
    {
      SmartDashboard.putString("Diagnostic Test", "ResetDevices");
      SetDefault();
      return true;
    });
  }



  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

      if(isTestMode == false){
        currentTestIndex = 0;
        isTestMode = true;
      } 

      if(System.currentTimeMillis() >= elapsedTestTimeMs + lastExecutionTimeMs)
      {
        listTest.get(currentTestIndex).apply(true);
        lastExecutionTimeMs = System.currentTimeMillis();
        currentTestIndex++;
        if(currentTestIndex >= listTest.size())
        {
          currentTestIndex = 0;
          SmartDashboard.putString("Diagnostic Test", "El Fin!");
        }
      }




  }
}
