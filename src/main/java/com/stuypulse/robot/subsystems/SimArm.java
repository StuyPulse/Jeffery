package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

public class SimArm extends IArm {

    private final LinearSystem<N2, N1, N1> arm;

    private final LinearSystemLoop<N2, N1, N1> loop;
    private final KalmanFilter<N2, N1, N1> filter;
    private final LinearQuadraticRegulator<N2, N1, N1> controller; 
    private TrapezoidProfile.State lastRef;

    private final Encoder greyhill;
    private final SmartNumber target;

    //easier set voltage to cos(angle) * Voltage(when level)

    public SimArm() {
        arm = LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(2), -1, -1);
        filter = new KalmanFilter<>(Nat.N2(), Nat.N1(), arm, VecBuilder.fill(0.015, 0.17), VecBuilder.fill(0.01), 0.020);
        controller = new LinearQuadraticRegulator<>(arm, VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)), VecBuilder.fill(12.0), 0.020);
        loop = new LinearSystemLoop<>(arm, controller, filter, 12.0, 0.020);
        lastRef = new TrapezoidProfile.State();


        target =  new SmartNumber("Arm/Target Height", 0);
        greyhill = new Encoder(-1, -1);


        
        Mechanism2d mechanism = new Mechanism2d(150, 300);


        loop.reset(VecBuilder.fill(greyhill.getDistance(), greyhill.getRate()));
        

    }

    @Override
    public double getTargetAngle() {
        return target.get();
    }

    @Override
    public double getAngle() {
        // return sim.getOutput(0);
    }

    @Override
    public void setTargetAngle(double height) {
        target.set(height);
        
    }
    
}
