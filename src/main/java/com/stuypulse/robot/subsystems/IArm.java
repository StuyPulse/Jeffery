package com.stuypulse.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IArm extends SubsystemBase {

    /*
    public static IArm getInstance() {
        return RobotBase.isReal() ? new Arm() : new SimArm();
    }
    */

    public abstract double getTargetAngle();

    public abstract double getAngle();

    public abstract void setTargetAngle(double angle);

    public abstract void move(double speed);

    public void addTargetAngle(double addedheight) {
        setTargetAngle(getTargetAngle() + addedheight);
    }

    public boolean atAngle(double maxerror) {
        return Math.abs(getAngle() - getTargetAngle()) < maxerror;
    }

}
