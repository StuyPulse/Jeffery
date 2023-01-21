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

    public void moveShoulder(double angle) {
        setTargetShoulderAngle(getTargetShoulderAngle() + angle);
    }
    
    public void moveWrist(double angle) {
        setTargetWristAngle(getTargetWristAngle() + angle);
    }

    public boolean isShoulderAtAngle(double maxError) {
        return Math.abs(getShoulderAngleDegrees() - getTargetShoulderAngle()) < maxError;
    }

    public boolean isWristAtAngle(double maxError) {
        return Math.abs(getWristAngleDegrees() - getTargetWristAngle()) < maxError;
    }

}
