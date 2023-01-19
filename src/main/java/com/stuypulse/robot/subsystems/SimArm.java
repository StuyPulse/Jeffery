package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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
    
    private final LinearSystemSim<N2, N1, N1> armSim;
    private final LinearSystemSim<N2, N1, N1> wristSim;

    private final MechanismLigament2d armLigament;
    private final MechanismLigament2d wristLigament;

    private final Controller controller1; 
    private final Controller controller2;

    private final SmartNumber targetAngle1;
    private final SmartNumber targetAngle2;

    public SimArm() {
        setSubsystem("SimArm");
        
        // simulation
        armSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(Settings.Arm.ArmArm.Feedforward.kV, Settings.Arm.ArmArm.Feedforward.kA));
        wristSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(Settings.Arm.Wrist.Feedforward.kV, Settings.Arm.Wrist.Feedforward.kA))

        //controller initialization
        controller1 = new PIDController(Settings.Arm.ArmArm.PID.kP, Settings.Arm.ArmArm.PID.kI, Settings.Arm.ArmArm.PID.kD)
            .add(new ElevatorFeedforward(Settings.Arm.Wrist.Feedforward.kG,Settings.Arm.Wrist.Feedforward.kA,Settings.Arm.Wrist.Feedforward.kV));
        controller2 = new PIDController(Settings.Arm.Wrist.PID.kP, Settings.Arm.Wrist.PID.kI, Settings.Arm.Wrist.PID.kD)
            .add(new ElevatorFeedforward(Settings.Arm.Wrist.Feedforward.kG,Settings.Arm.Wrist.Feedforward.kA,Settings.Arm.Wrist.Feedforward.kV));

        // ligament initialization
        Mechanism2d arm = new Mechanism2d(2, 2);
        MechanismRoot2d armRoot = arm.getRoot("Arm Root", 1 , 1);
        armLigament = armRoot.append(new MechanismLigament2d("Arm Arm", 1, 1));

        addChild("Arm Mechanism2d", arm);

        // Mechanism2d wrist = armLigament.append(new MechanismLigament2d("Wrist Arm", 1, 1));
        // armLigament.append(wrist);
        // addChild("Wrist Mechanism2d", wrist);

        // SmartNumber initialization
        targetAngle1 = new SmartNumber("Arm/Target Angle 1", 0);
        targetAngle2 = new SmartNumber("Arm/Target Angle 2", 0);
    }
    
    public double getTargetAngle1() {
        return targetAngle1.get();
    }

    public double getTargetAngle2() {
        return targetAngle2.get();
    }


    public double getAngle1() {
        return armSim.getOutput(0);
    }

    
    public double getAngle2() {
        return wristSim.getOutput(0);
    }

    public void setTargetAngle1(double angle) {
        targetAngle1.set(angle);
    }

    public void setTargetAngle2(double angle) {
        targetAngle2.set(angle);
    }

    public void moveArm(double speed) {   
        armSim.setInput(SLMath.clamp(speed, -1, +1) * 12);
    }

    public void moveWrist(double speed) {
        wristSim.setInput(SLMath.clamp(speed, -1, +1) * 12);
    }

    @Override
    public void periodic() {
        controller1.update(getTargetAngle1(), getAngle1());
        controller2.update(getTargetAngle2(), getAngle2());
        
        armSim.setInput(MathUtil.clamp(
            controller1.update(getTargetAngle1(), getAngle1()),
            -RoboRioSim.getVInVoltage(),
            +RoboRioSim.getVInVoltage()));

        wristSim.setInput(MathUtil.clamp(
            controller2.update(getTargetAngle2(), getAngle2()),
            -RoboRioSim.getVInVoltage(),
            +RoboRioSim.getVInVoltage()));

        armLigament.setAngle(getAngle1() + 90);

        SmartDashboard.putNumber("Arm Angle", getAngle1());
        SmartDashboard.putNumber("Wrist Angle", getAngle2());
        SmartDashboard.putNumber("Arm Voltage", controller1.getOutput());
        SmartDashboard.putNumber("Wrist Voltage", controller2.getOutput());
    }
}