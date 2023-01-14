package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings.Arm;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;




public class SimArm extends IArm {
    
    private final SingleJointedArmSim simArm;

    private SmartNumber targetAngle;


    public SimArm() {
        simArm = new SingleJointedArmSim(DCMotor.getNEO(2), Arm.GEARING, Arm.WEIGHT, Arm.LENGTH, Arm.MINANGLE, Arm.MAXANGLE, Arm.MASS, true);
    }


    @Override
    public double getTargetAngle() {
        return targetAngle.get();
    }


    @Override
    public double getAngle() {
        return simArm.getAngleRads();
    }


    @Override
    public void setTargetAngle(double angle) {
        targetAngle.set(angle);
    }


    @Override
    public void move(double speed) {
        simArm.setInput(SLMath.clamp(speed, -1, +1) * 12);
    }

    public void simulationPeriodic() {
        simArm.update(0.02);
        
    }

}