package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;

public class Pistons{
    DoubleSolenoid frontLeft;
    DoubleSolenoid frontRight;
    DoubleSolenoid rearLeg;

    DoubleSolenoid Gripper;
    DoubleSolenoid Roller;
    DoubleSolenoid Kicker;

    int legsPCMPort = 0;
    int scoringPCMPort = 0;

    Compressor c;
    
    
    public Pistons(DoubleSolenoid frontLeft, DoubleSolenoid frontRight, DoubleSolenoid rearLeg, DoubleSolenoid Gripper,    DoubleSolenoid Roller, DoubleSolenoid Kicker, Compressor c){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeg = rearLeg;

        this.Gripper = Gripper;
        this.Roller = Roller;
        this.Kicker = Kicker;

        this.c = c;
    }

    public void initPistons(){
        c.setClosedLoopControl(true);
        c.start();

        frontLeft.set(Value.kReverse);
        frontRight.set(Value.kReverse);
        rearLeg.set(Value.kReverse);

        Gripper.set(Value.kReverse);
        Roller.set(Value.kReverse);
        Kicker.set(Value.kReverse);
    }

    public void dropLegs(){
        frontLeft.set(Value.kForward);
        frontRight.set(Value.kForward);
        rearLeg.set(Value.kForward);
    }

    public void retractLegs(){
        frontLeft.set(Value.kReverse);
        frontRight.set(Value.kReverse);
        rearLeg.set(Value.kReverse);
    }

    public void enableScoring(){
        Gripper.set(Value.kForward);
        Roller.set(Value.kForward);
        Kicker.set(Value.kForward);
    }

    public void disableScoring(){
        Gripper.set(Value.kReverse);
        Roller.set(Value.kReverse);
        Kicker.set(Value.kReverse);
    }
}