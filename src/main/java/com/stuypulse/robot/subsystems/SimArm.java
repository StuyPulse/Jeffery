package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Arm;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;


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
    
    private final Controller controller1;
    private final Controller controller2;

    private final SmartNumber targetAngle1;
    private final SmartNumber targetAngle2;

    public SimArm() {
        setSubsystem("SimArm");
        
        // simulation
        armSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(kV.get(), kA.get()));
        wristSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(kV.get(),kA.get()))

        //controller initaliation
        controller1 = new PIDController(Settings.Arm.kP, Settings.Arm.kI, Settings.Arm.kD).add(new Feedforward(kG,kA,kV));
        controller2 = new PIDController(Settings.Arm.kP, Settings.Arm.kI, Settings.Arm.kD).add(new Feedforward(kG,kA,kV));


        targetAngle1 = new SmartNumber("Arm/Target Angle 1", 0);
        targetAngle2 = new SmartNumber("Arm/Target Angle 2", 0);
    }
    
    /** 
        @return target angle of arm
    */
    public double getTargetAngle1() {
        return targetAngle1.get();
    }

    /** 
        @return target angle of wrist
    */
    public double getTargetAngle2() {
        return targetAngle2.get();
    }

    /** 
        @return current angle of arm
    */
    public double getAngle1() {
        return armSim.getOutput(0);
    }

    /** 
        @return current angle of wrist
    */
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
            controller1.update(getTargetAngle1(), getAngle1());
            +RoboRioSim.getVInVoltage()));

        wristSim.setInput(MathUtil.clamp(
            controller2.update(getTargetAngle2(), getAngle2()),
            -RoboRioSim.getVInVoltage(),
            +RoboRioSim.getVInVoltage()));
    }
}