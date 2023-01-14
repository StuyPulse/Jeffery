/* Copyright (c) 2021 StuyPulse Robotics. All rights reserved. */
/* This work is licensed under the terms of the MIT license */
/* found in the root directory of this project. */

package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.network.SmartBoolean;

import com.stuypulse.robot.constants.Ports.Pneumatics;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pump extends SubsystemBase {

    private final SmartBoolean enabled;
    private final Compressor compressor;
    private final AnalogInput pressureGauge;

    public Pump() {
        enabled = new SmartBoolean("Pump/Compressor Enabled", false);
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        pressureGauge = new AnalogInput(Pneumatics.ANALOG_PRESSURE_SWITCH_PORT);

        // Add Children to Subsystem
        addChild("Compressor", compressor);
        addChild("Pressure Gauge", pressureGauge);
    }

    // Start Compressing the Robot
    public void compress() {
        this.set(true);
    }

    // Stop Compressing
    public void stop() {
        this.set(false);
    }

    // Get the current pressure of the pneumatics
    public double getPressure() {
        return 5000.0 * pressureGauge.getAverageVoltage();
    }

    // Set the compressor to on or off
    public void set(boolean compressing) {
        enabled.set(compressing);
    }

    @Override
    public void periodic() {
        if (enabled.get()) {
            compressor.enableDigital();
        } else {
            compressor.disable();
        }

        // SmartDashboard
        SmartDashboard.putNumber("Pump/Robot Air Pressure", getPressure() / 1000.0);
    }
}