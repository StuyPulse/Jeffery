package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Arm.*;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.filters.MotionProfile;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


// IGNORE***: Two single jointed arms, first one contantly change r. use Ia = rFsin(theta) to find a, mass should be same, find r.

/* Todo: 
    - find moment of inertia at different configurations
    - Fix constants(add them to settings)
    - Write feedforward for Arm
    - rename methods
    - TEST
*/

public class SimArm extends IArm {
    
    private final SingleJointedArmSim armSim;
    private final SingleJointedArmSim wristSim;

    private MechanismLigament2d armLigament;
    private MechanismLigament2d wristLigament;

    private final Mechanism2d arm;

    private final Controller armController; 
    private final Controller wristController;

    private final SmartNumber armTargetAngle;
    private final SmartNumber wristTargetAngle;

    public SimArm() { 
        setSubsystem("SimArm");
        
        // simulation
        armSim = new SingleJointedArmSim(DCMotor.getNEO(1), ArmArm.GEARING, ArmArm.JKG, ArmArm.LENGTH, ArmArm.MINANGLE, ArmArm.MAXANGLE, ArmArm.MASS, true);
        wristSim = new SingleJointedArmSim(DCMotor.getNEO(1), Wrist.GEARING, Wrist.JKG, Wrist.LENGTH, Wrist.MINANGLE, Wrist.MAXANGLE, Wrist.MASS, true);
    
        
        //controller initialization
        armController = new Feedforward.Elevator(Settings.Arm.ArmArm.Feedforward.kG,Settings.Arm.ArmArm.Feedforward.kS, Settings.Arm.ArmArm.Feedforward.kA,Settings.Arm.ArmArm.Feedforward.kV).position()
            .add(new PIDController(Settings.Arm.ArmArm.PID.kP, Settings.Arm.ArmArm.PID.kI, Settings.Arm.ArmArm.PID.kD))
            .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT));
        //reminder to change FeedForward to Arm
        
        wristController = new Feedforward.Elevator(Settings.Arm.Wrist.Feedforward.kG,Settings.Arm.Wrist.Feedforward.kS, Settings.Arm.Wrist.Feedforward.kA,Settings.Arm.Wrist.Feedforward.kV).position()
            .add(new PIDController(Settings.Arm.Wrist.PID.kP, Settings.Arm.Wrist.PID.kI, Settings.Arm.Wrist.PID.kD))
            .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT));
        

        // ligament initialization
        arm = new Mechanism2d(2, 2);
        MechanismRoot2d armRoot = arm.getRoot("Arm Root", 1 , 1);
        wristLigament = new MechanismLigament2d("Wrist", 1, getWristAngleDegrees());
        armLigament = new MechanismLigament2d("Arm", 1, getArmAngleDegrees());
        armRoot.append(armLigament);

        // addChild("Arm Mechanism2d", arm);

        // Mechanism2d wrist = armLigament.append(new MechanismLigament2d("Wrist Arm", 1, 1));
        // armLigament.append(wrist);
        // addChild("Wrist Mechanism2d", wrist);

        // SmartNumber initialization
        armTargetAngle = new SmartNumber("Arm/Target Arm Angle", 0);
        wristTargetAngle = new SmartNumber("Arm/Target Wrist Angle", 0);

        SmartDashboard.putData("Arm Mech2d", arm);

    }
    
    public double getTargetArmAngle() {
        return armTargetAngle.get();
    }

    public double getTargetWristAngle() {
        return wristTargetAngle.get();
    }


    public double getArmAngleDegrees() {
        return Math.toDegrees(armSim.getOutput(0));
    }

    
    public double getWristAngleDegrees() {
        return Math.toDegrees(wristSim.getOutput(0));
    }

    public void setTargetArmAngle(double angle) {
        armTargetAngle.set(angle);
    }

    public void setTargetWristAngle(double angle) {
        wristTargetAngle.set(angle);
    }

    @Override
    public void periodic() {
        
        // armSim.setInput(MathUtil.clamp(
        //     armController.update(getTargetArmAngle(), getArmAngle()),
        //     -RoboRioSim.getVInVoltage(),
        //     +RoboRioSim.getVInVoltage()));

        // wristSim.setInput(MathUtil.clamp(
        //     wristController.update(getTargetWristAngle(), getWristAngle()),
        //     -RoboRioSim.getVInVoltage(),
        //     +RoboRioSim.getVInVoltage()));
    MechanismRoot2d wristRoot = arm.getRoot("Wrist Root", 1 + Math.cos(getArmAngleDegrees()),  
        1 + Math.sin(getArmAngleDegrees()));
        wristLigament = new MechanismLigament2d("Wrist", 1, getWristAngleDegrees());
        wristRoot.append(wristLigament);

        armSim.setInput(0);

        armSim.update(0.02);

        armLigament.setAngle(getArmAngleDegrees());

        SmartDashboard.putNumber("Arm Angle", getArmAngleDegrees());
        SmartDashboard.putNumber("Wrist Angle", getWristAngleDegrees());
        SmartDashboard.putNumber("Arm Voltage", armController.getOutput());
        SmartDashboard.putNumber("Wrist Voltage", wristController.getOutput());
    }
}