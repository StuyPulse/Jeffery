package com.stuypulse.robot.util;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DoubleJointedArmSim extends SubsystemBase{

    private final SingleJointedArmSim shoulderSim;
    private final SingleJointedArmSim wristSim; 
   
    public DoubleJointedArmSim(DCMotor shoulderDCMotorGearbox, double shoulderGearing, 
        double shoulderMomentOfInertia, double shoulderArmLengthMeters, double shoulderMinAngleRads, 
        double shoulderMaxAngleRads, double shoulderArmMass, 
        DCMotor wristDCMotorGearbox, double wristGearing, double wristMomentOfIntertia,
        double wristArmLengthMeters, double wristMinAngleRads, double wristMaxAngleRads,
        double wristArmMass) {

        shoulderSim = new SingleJointedArmSim(shoulderDCMotorGearbox, shoulderGearing, shoulderMomentOfInertia, shoulderArmLengthMeters, shoulderMinAngleRads, shoulderMaxAngleRads, shoulderArmMass, true);
        wristSim = new SingleJointedArmSim(wristDCMotorGearbox, wristGearing, wristMomentOfIntertia, wristArmLengthMeters, wristMinAngleRads, wristMaxAngleRads, wristArmMass, true);
    }

    public DoubleJointedArmSim(SingleJointedArmSim shoulder, SingleJointedArmSim wrist) {
        this.shoulderSim = shoulder;
        this.wristSim = wrist;
    }

    public double getShoulderAngleDegrees() {
        return Math.toDegrees(shoulderSim.getOutput(0));
    }
        
    public double getWristAngleDegrees() {
        return Math.toDegrees(wristSim.getOutput(0));
    }

    public double getShoulderAngleRadians() {
        return shoulderSim.getOutput(0);
    }

    public double getWristAngleRadians() {
        return wristSim.getOutput(0);
    }

    public void setInput(double shoulderVoltage, double wristVoltage) {
        shoulderSim.setInput(shoulderVoltage);
        wristSim.setInput(wristVoltage);
    }

    public void update(double dtSeconds) {
        shoulderSim.update(dtSeconds);
        wristSim.update(dtSeconds);
    }

    /**
     * Less Used Methods Below
     */

    public static double estimateMOI(double lengthMeters, double massKg) {
        return 1.0 / 3.0 * massKg * lengthMeters * lengthMeters;
    }
    
}
