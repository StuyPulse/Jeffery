package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.Encoder;

public class Arm extends IArm {

    // motors
    private final CANSparkMax shoulder;
    private final CANSparkMax wrist;

    // encoders
    private final Encoder shoulderGreyhill;
    private final Encoder wristGreyhill;

    // controllers
    private final Controller shoulderController; 
    private final Controller wristController;

    // targeting
    private final SmartNumber shoulderTargetAngle;
    private final SmartNumber wristTargetAngle;

    public Arm() {
        // replace
        shoulder = new CANSparkMax(-1, MotorType.kBrushed);
        wrist = new CANSparkMax(-1, MotorType.kBrushed);

        shoulderGreyhill = new Encoder(-1, -1);


    }

    @Override
    public double getTargetShoulderAngle() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getTargetWristAngle() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getShoulderAngleDegrees() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getWristAngleDegrees() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void setTargetShoulderAngle(double angle) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setTargetWristAngle(double angle) {
        // TODO Auto-generated method stub
        
    }
    
    public void periodic() {

    }

}
