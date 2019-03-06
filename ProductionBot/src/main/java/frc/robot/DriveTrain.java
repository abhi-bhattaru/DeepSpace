package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain{
    double leftY;
    double leftX;
    double rightY;
    double rightX;
    double dampen;
    boolean dpUp;
    boolean dpDown;
    boolean dpLeft;
	boolean dpRight;
    
    DifferentialDrive chassis;
	XboxController xbox;
	JoystickLocations porting;
	
	private static final int kUltrasonicPortRight = 0;
	private AnalogInput m_ultrasonicRight = new AnalogInput(kUltrasonicPortRight);
	private static final int kUltrasonicPortLeft = 2;
	private AnalogInput m_ultrasonicLeft = new AnalogInput(kUltrasonicPortLeft);
	private static final double kValueToInches = 0.049;
	
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	Accelerometer accel= new BuiltInAccelerometer(Accelerometer.Range.k4G);
	
	double sonarRight;
	double sonarLeft;
	double[] accVal = new double[3];
	
	public enum driveModes {
		Field,
		Arcade,
		Tank,
		Car
	}
    
    public DriveTrain(DifferentialDrive d,XboxController x, JoystickLocations p) {
		chassis=d;
		xbox=x;
		porting=p;
	}
	
	public void joystickDrive(){
		changeDrive();
		updateAxes();
	}
    
    public void updateAxes() {
		leftY=xbox.getRawAxis(porting.lYAxis);
		leftX=xbox.getRawAxis(porting.lXAxis);
		rightX=xbox.getRawAxis(porting.rXAxis);
		rightY=xbox.getRawAxis(porting.rYAxis);
		dampen=1-3/4*xbox.getTriggerAxis(Hand.kRight);
		SmartDashboard.putNumber("leftY", leftY);
		SmartDashboard.putNumber("leftX", leftX);
		SmartDashboard.putNumber("rightY", rightY);
		SmartDashboard.putNumber("rightX", rightX);
		SmartDashboard.putNumber("dampen", dampen);
		
		if(Math.round(xbox.getPOV())!=1) {
			if(Math.round(xbox.getPOV())<90) {
				dpUp = true;
				dpDown = false;
				dpLeft = false;
				dpRight = false;
			}else if(Math.round(xbox.getPOV())<180) {
				dpUp = false;
				dpDown = false;
				dpLeft = false;
				dpRight = true;
			}else if(Math.round(xbox.getPOV())<270) {
				dpUp = false;
				dpDown = true;
				dpLeft = false;
				dpRight = false;
			}else if(Math.round(xbox.getPOV())<360) {
				dpUp = false;
				dpDown = false;
				dpLeft = true;
				dpRight = false;
			}
		}else {
			
				dpUp = false;
				dpDown = false;
				dpLeft = false;
				dpRight = false;
			
		}
    }
    
    	//updates the value of the sonar sensor
	public void getDistance() {
		this.sonarRight = m_ultrasonicRight.getValue()* kValueToInches;
		SmartDashboard.putNumber("distance right", sonarRight);
		
		this.sonarLeft = m_ultrasonicLeft.getValue()* kValueToInches;
		SmartDashboard.putNumber("distance left", sonarLeft);
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
    
    public void DriveArcade() {		
		double radians = getHeading()*Math.PI/180;//convert angle to radian value
		
		getHeading();//update gyro
		getDistance();//update sonar
		getAccel();//update accelerometer
		
		//math to find how to turn robot to make field oriented work
		double temp = Math.cos(radians)*leftX + Math.sin(radians)*leftY;
		leftY = -leftX*Math.sin(radians) + leftY*Math.cos(radians);
		leftX=temp;
						
		chassis.arcadeDrive( -dampen*leftY, dampen*leftX);//drive
		
		if(Math.abs(xbox.getRawAxis(porting.rXAxis)) > .25) {
			chassis.arcadeDrive(0, dampen*xbox.getRawAxis(porting.rXAxis));
		}
		
    }
    
    	//This changes which kind of drive system to use. Ignore for now
	public void changeDrive() {
		
		driveModes mode = DetermineMode();

		switch (mode) {
		case Field://button A does field oriented arcade
			DriveArcade();
			break;
		case Arcade://button B does just arcade
			chassis.arcadeDrive(dampen*-xbox.getRawAxis(porting.lYAxis),dampen*xbox.getRawAxis(porting.lXAxis));
			getHeading();
			getDistance();
			getAccel();
			break;
		case Tank://button X does tank drive
			getHeading();
			getDistance();
			getAccel();
			chassis.tankDrive(dampen*-xbox.getRawAxis(porting.lYAxis),dampen*-xbox.getRawAxis(porting.rYAxis) );
			break;
		case Car://button Y drive this like a car
			chassis.curvatureDrive(dampen*-xbox.getRawAxis(porting.lYAxis),dampen*-xbox.getRawAxis(porting.rXAxis) , false);//cheeeeessssyyyy
			getHeading();
			getDistance();
			getAccel();
			break;
		default: break;
		}
	}
	
	public driveModes DetermineMode(){
		 driveModes mode = driveModes.Car;

		if(xbox.getAButton()){
			mode= driveModes.Field;
		}
		else if(xbox.getBButton()){
			mode= driveModes.Arcade;
		}
		else if(xbox.getXButton()){
			mode=driveModes.Tank;
		}

		if(xbox.getYButton()) {
			dampen = Math.sqrt(.5);
		}
		else{
			dampen = 1;
		}

		return mode;
	}
    
    public boolean turnRight(double degrees) {
		//TODO Refactor to the correct angle
		// if(degrees>gyro.getAngle()) {
		// 	chassis.tankDrive(-.625, .625);
		// 	return false;
		// }
		// chassis.arcadeDrive(0, 0);
		chassis.arcadeDrive(.2,degrees);
		return true;
	}
	
	public boolean turnLeft(double degrees) {
		//TODO Refactor to the correct angle
		// if(-degrees<gyro.getAngle()) {
		// 	chassis.tankDrive(.625, -.625);
		// 	return false;
		// }
		// chassis.arcadeDrive(0, 0);
		chassis.arcadeDrive(.2, degrees*-1);
		return true;
	}

	public boolean continueStraight(double speed){
		chassis.arcadeDrive(speed, 0);
		return true;
	}
	
	public boolean driveUntil(double inches) {
		getDistance();
		if(sonarRight==inches||sonarRight==inches) {
			chassis.arcadeDrive(0, 0);
			return true;
		}
		chassis.arcadeDrive(.625, 0);
		return false;
    }
}