package com.stuypulse.robot.util;

public class Pitch {
    public static double calculate(double pitch, double roll, double yaw) {
        
        return -1 * Math.signum(Math.atan(
            Math.tan(pitch) * Math.cos(yaw) + 
            Math.tan(roll) * Math.sin(yaw))) * 
                Math.atan(Math.sqrt(
                    Math.pow(Math.tan(roll), 2) + 
                    Math.pow(Math.tan(pitch), 2)
                    )
                );
    }
}