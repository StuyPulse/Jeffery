package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {

    public final CANSparkMax motor;
    public final Encoder greyhill;
    public final DigitalInput bottomlimitSwitch;
    public final DigitalInput topLimitSwitch;

    public Elevator(){
        motor = new CANSparkMax();
        
        
    }
}
