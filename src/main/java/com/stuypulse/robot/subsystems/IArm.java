package com.stuypulse.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IArm extends SubsystemBase {

    /*
    public static IArm getInstance() {
        return RobotBase.isReal() ? new Arm() : new SimArm();
    }
    */

    public abstract double getTargetAngle1();

    public abstract double getTargetAngle2();

    public abstract double getAngle1();

    public abstract double getAngle2();

    public abstract void setTargetAngle1(double angle);

    public abstract void setTargetAngle2(double angle);


    public abstract void moveArm(double speed);

    public abstract void moveWrist(double speed);


    public void addTargetAngle1(double addedheight) {
        setTargetAngle1(getTargetAngle1() + addedheight);
    }
    
    public void addTargetAngle2(double addedheight) {
        setTargetAngle2(getTargetAngle2() + addedheight);
    }

    public boolean jointOneAtAngle(double maxerror) {
        return Math.abs(getAngle1() - getTargetAngle1()) < maxerror;
    }

    public boolean jointTwoAtAngle(double maxerror) {
        return Math.abs(getAngle2() - getTargetAngle2()) < maxerror;
    }

}
