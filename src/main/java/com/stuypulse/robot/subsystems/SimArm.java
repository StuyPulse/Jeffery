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
import edu.wpi.first.wpilibj.util.Color8Bit;


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
    
    private MechanismLigament2d baseLigament;
    private MechanismLigament2d armLigament;
    private MechanismLigament2d wristLigament;

    private MechanismRoot2d armRoot;
    private MechanismRoot2d wristRoot;

    private final Mechanism2d arm;

    private final Controller armController; 
    private final Controller wristController;

    private final SmartNumber armTargetAngle;
    private final SmartNumber wristTargetAngle;

    public SimArm() { 
        setSubsystem("SimArm");
        
        // simulation
        armSim = new SingleJointedArmSim(DCMotor.getNEO(1), ArmArm.GEARING, ArmArm.JKG+Wrist.JKG, ArmArm.LENGTH, ArmArm.MINANGLE, ArmArm.MAXANGLE, ArmArm.MASS, true);
        wristSim = new SingleJointedArmSim(DCMotor.getNEO(1), Wrist.GEARING, Wrist.JKG, Wrist.LENGTH, Wrist.MINANGLE, Wrist.MAXANGLE, Wrist.MASS, true);
    
        
        //controller initialization

        armController = new Feedforward.Motor(Settings.Arm.ArmArm.Feedforward.kS, Settings.Arm.ArmArm.Feedforward.kA,Settings.Arm.ArmArm.Feedforward.kV).position()
            .add(new FeedforwardArm(ArmArm.Feedforward.kG))
            .add(new PIDController(Settings.Arm.ArmArm.PID.kP, Settings.Arm.ArmArm.PID.kI, Settings.Arm.ArmArm.PID.kD))
            // .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT))
            .setOutputFilter(x -> MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage() ));
        
        wristController = new Feedforward.Elevator(Settings.Arm.Wrist.Feedforward.kG,Settings.Arm.Wrist.Feedforward.kS, Settings.Arm.Wrist.Feedforward.kA,Settings.Arm.Wrist.Feedforward.kV).position()
            .add(new PIDController(Settings.Arm.Wrist.PID.kP, Settings.Arm.Wrist.PID.kI, Settings.Arm.Wrist.PID.kD))
            .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT));
        

        // ligament initialization
        arm = new Mechanism2d(4, 4);
        armRoot = arm.getRoot("Arm Root", 2 , 2);
        wristRoot = arm.getRoot("Wrist Root", 2, 2);
       
        baseLigament = new MechanismLigament2d("Base", 2, 0);
        wristLigament = new MechanismLigament2d("Wrist", 0.5, getWristAngleDegrees());
        armLigament = new MechanismLigament2d("Arm", 1, getArmAngleDegrees());
        baseLigament.setColor(new Color8Bit(0, 255, 0));
        armLigament.setColor(new Color8Bit(255, 0, 255));
        wristLigament.setColor(new Color8Bit(0, 0, 255));

        armRoot.append(baseLigament);
        armRoot.append(armLigament);
        wristRoot.append(wristLigament);

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
        
        armSim.setInput(armController.update(getTargetArmAngle(), getArmAngleDegrees()));

        // wristSim.setInput(MathUtil.clamp(
        //     wristController.update(getTargetWristAngle(), getWristAngleDegrees()),
        //     -RoboRioSim.getVInVoltage(),
        //     +RoboRioSim.getVInVoltage()));
        wristSim.setInput(0);

        armSim.update(0.02);
        wristSim.update(0.02);

        wristRoot.setPosition(2 + Math.cos(Math.toRadians(getArmAngleDegrees())),  
        2 + Math.sin(Math.toRadians(getArmAngleDegrees())));

        baseLigament.setAngle(-90);
        armLigament.setAngle(getArmAngleDegrees());
        wristLigament.setAngle(getWristAngleDegrees());


        SmartDashboard.putNumber("Arm/Arm Angle", getArmAngleDegrees());
        SmartDashboard.putNumber("Arm/Wrist Angle", getWristAngleDegrees());
        SmartDashboard.putNumber("Arm/Arm Voltage", armController.getOutput());
        SmartDashboard.putNumber("Arm/Wrist Voltage", wristController.getOutput());
    }
}