package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChargeDrive extends CommandBase {
    private AngleController control;
    private SwerveDrive swerve;

    private SmartNumber kP, kD;

    private Gamepad driver;

    public ChargeDrive(SwerveDrive swerve, Gamepad driver) {
        this.swerve = swerve;
        this.driver = driver;

        kP = new SmartNumber("Charge Drive/kP", 3.5);
        kD = new SmartNumber("Charge Drive/kD", 0.1);

        control = new AnglePIDController(kP, 0, kD);

        addRequirements(swerve);
    }
    
    @Override
    public void execute() {
        Vector2D stick = driver.getLeftStick();

        Vector2D t = new Vector2D(stick.magnitude(), 0);
        System.out.println(t);
        if(Math.abs(stick.x) > Math.abs(stick.y)) {
            if (stick.x < 0) t = t.rotate(Angle.k180deg);
        } else {
            if (stick.y < 0) t = t.rotate(Angle.fromDegrees(-90));
            else             t = t.rotate(Angle.k90deg);
        }

        swerve.setStates(t.mul(Settings.Driver.MAX_TELEOP_SPEED.get()), control.update(Angle.kZero, Angle.fromRotation2d(swerve.getAngle().times(-1))));
    }  
}
    









