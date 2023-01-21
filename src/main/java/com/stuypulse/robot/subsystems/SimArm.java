package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.util.DoubleJointedArmSim;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.network.SmartNumber;

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

    private final DoubleJointedArmSim armSim;
    
    private MechanismLigament2d baseLigament;
    private MechanismLigament2d shoulderLigament;
    private MechanismLigament2d wristLigament;

    private MechanismRoot2d shoulderRoot;
    private MechanismRoot2d wristRoot;

    private final Mechanism2d arm;

    private final Controller shoulderController; 
    private final Controller wristController;

    private final SmartNumber shoulderTargetAngle;
    private final SmartNumber wristTargetAngle;

    public SimArm() { 
        setSubsystem("SimArm");
        
        // simulation
        armSim = new DoubleJointedArmSim(new SingleJointedArmSim(DCMotor.getNEO(1), Shoulder.GEARING, Shoulder.JKG+Wrist.JKG, Shoulder.LENGTH, Shoulder.MINANGLE, Shoulder.MAXANGLE, Shoulder.MASS, true), 
            new SingleJointedArmSim(DCMotor.getNEO(1), Wrist.GEARING, Wrist.JKG, Wrist.LENGTH, Wrist.MINANGLE, Wrist.MAXANGLE, Wrist.MASS, true));
    
        
        //controller initialization

        shoulderController = new Feedforward.Motor(Settings.Arm.Shoulder.Feedforward.kS, Settings.Arm.Shoulder.Feedforward.kA,Settings.Arm.Shoulder.Feedforward.kV).position()
            .add(new FeedforwardArm(Shoulder.Feedforward.kG))
            .add(new PIDController(Settings.Arm.Shoulder.PID.kP, Settings.Arm.Shoulder.PID.kI, Settings.Arm.Shoulder.PID.kD))
            // .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT))
            .setOutputFilter(x -> MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage() ));
        
        wristController = new Feedforward.Motor(Settings.Arm.Wrist.Feedforward.kS, Settings.Arm.Wrist.Feedforward.kA,Settings.Arm.Wrist.Feedforward.kV).position()
            .add(new FeedforwardArm(Wrist.Feedforward.kG))
            .add(new PIDController(Settings.Arm.Wrist.PID.kP, Settings.Arm.Wrist.PID.kI, Settings.Arm.Wrist.PID.kD))
            // .setSetpointFilter(new MotionProfile(ArmArm.VEL_LIMIT, ArmArm.ACCEL_LIMIT));
            .setOutputFilter(x -> MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage() ));


        // ligament initialization
        arm = new Mechanism2d(4, 4);
        shoulderRoot = arm.getRoot("Arm Root", 2 , 2);
        wristRoot = arm.getRoot("Wrist Root", 2, 2);
       
        baseLigament = new MechanismLigament2d("Base", 2, 0);
        wristLigament = new MechanismLigament2d("Wrist", 0.5, getWristAngleDegrees());
        shoulderLigament = new MechanismLigament2d("Arm", 1, getShoulderAngleDegrees());
        baseLigament.setColor(new Color8Bit(0, 255, 0));
        shoulderLigament.setColor(new Color8Bit(255, 0, 255));
        wristLigament.setColor(new Color8Bit(0, 0, 255));

        shoulderRoot.append(baseLigament);
        shoulderRoot.append(shoulderLigament);
        wristRoot.append(wristLigament);

        shoulderTargetAngle = new SmartNumber("Arm/Target Arm Angle", 0);
        wristTargetAngle = new SmartNumber("Arm/Target Wrist Angle", 0);

        SmartDashboard.putData("Arm Mech2d", arm);

    }
    
    public double getTargetShoulderAngle() {
        return shoulderTargetAngle.get();
    }

    public double getTargetWristAngle() {
        return wristTargetAngle.get();
    }


    public double getShoulderAngleDegrees() {
        return armSim.getShoulderAngleDegrees();
    }

    
    public double getWristAngleDegrees() {
        return armSim.getWristAngleDegrees();
    }

    public void setTargetShoulderAngle(double angle) {
        shoulderTargetAngle.set(angle);
    }

    public void setTargetWristAngle(double angle) {
        wristTargetAngle.set(angle);
    }

    @Override
    public void periodic() {        
        armSim.setInput(shoulderController.update(getTargetShoulderAngle(), getShoulderAngleDegrees()), wristController.update(getTargetWristAngle(), getWristAngleDegrees()));

        armSim.update(0.02);

        wristRoot.setPosition(2 + Math.cos(Math.toRadians(getShoulderAngleDegrees())),  
        2 + Math.sin(Math.toRadians(getShoulderAngleDegrees())));

        baseLigament.setAngle(-90);
        shoulderLigament.setAngle(getShoulderAngleDegrees());
        wristLigament.setAngle(getWristAngleDegrees());


        SmartDashboard.putNumber("Arm/Arm Angle", getShoulderAngleDegrees());
        SmartDashboard.putNumber("Arm/Wrist Angle", getWristAngleDegrees());
        SmartDashboard.putNumber("Arm/Arm Voltage", shoulderController.getOutput());
        SmartDashboard.putNumber("Arm/Wrist Voltage", wristController.getOutput());
    }
}