package com.stuypulse.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IArm extends SubsystemBase {

    public static IArm getInstance() {
        return RobotBase.isReal() ?  new SimArm() : new SimArm();
    }
    
    public abstract double getTargetShoulderAngle();

    public abstract double getTargetWristAngle();

    public abstract double getShoulderAngleDegrees();

    public abstract double getWristAngleDegrees();

    public abstract void setTargetShoulderAngle(double angle);

    public abstract void setTargetWristAngle(double angle);

    public void addTargetAngle1(double addedheight) {
        setTargetShoulderAngle(getTargetShoulderAngle() + addedheight);
    }
    
    public void addTargetAngle2(double addedheight) {
        setTargetWristAngle(getTargetWristAngle() + addedheight);
    }

    public boolean jointOneAtAngle(double maxerror) {
        return Math.abs(getShoulderAngleDegrees() - getTargetShoulderAngle()) < maxerror;
    }

    public boolean jointTwoAtAngle(double maxerror) {
        return Math.abs(getWristAngleDegrees() - getTargetWristAngle()) < maxerror;
    }

}
