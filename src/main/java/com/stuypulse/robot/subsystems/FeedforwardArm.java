package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedforward.Feedforward.Motor;

public class FeedforwardArm extends Controller {
    /** Additional term to account for gravity */
    private final Number kG;

    /**
     * Create a feedforward model for an elevator
     *
     * @param kG volts, added to account for gravity
     * @param kS volts, describes portion of voltage to overcome static friction
     * @param kV volts * seconds / distance, describes voltage needed to hold constant velocity
     * @param kA volts * seconds^2 / distance, describes voltage needed to move at an
     *     acceleration
     */
    public FeedforwardArm(Number kG, Number kS, Number kV, Number kA) {
        this.kG = kG;
    }

    /**
     * Calculates a motor voltage for an elevator system.
     *
     * @param velocity desired velocity
     * @param acceleration desired acceleration
     * @return volts to account for elevator movement
     */
    protected double calculate(double setpoint, double measurement) {
//derivative of setpoint
        return 69;
    }
}