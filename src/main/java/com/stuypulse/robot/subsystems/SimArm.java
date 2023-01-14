package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings.Arm;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;




public class SimArm extends IArm {
    
    private final SingleJointedArmSim simArm;

    private SmartNumber targetAngle;

    public Controller controller;
    
    private final MechanismLigament2d longArm;


    public SimArm() {
        simArm = new SingleJointedArmSim(DCMotor.getNEO(2), Arm.GEARING, Arm.WEIGHT, Arm.LENGTH, Arm.MINANGLE, Arm.MAXANGLE, Arm.MASS, true);
        controller = new PIDController(Arm.PID.kP, Arm.PID.kI, Arm.PID.kD);

        Mechanism2d arm = new Mechanism2d(2, 2);
        MechanismRoot2d armRoot = arm.getRoot("Intake Root", 1, 1);
        longArm = new MechanismLigament2d("Arm", 1, 1);

        targetAngle = new SmartNumber("Target Angle", 0);

        armRoot.append(longArm);
        addChild("Intake Mechanism2d", arm);

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
        simArm.setInput(SLMath.clamp(controller.update(getTargetAngle(), getAngle()), -RoboRioSim.getVInVoltage(), RoboRioSim.getVInVoltage()));

        simArm.update(0.02);
        longArm.setAngle(getAngle());
    }

}