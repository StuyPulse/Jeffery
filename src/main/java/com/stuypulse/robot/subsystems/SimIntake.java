package com.stuypulse.robot.subsystems;

public class SimIntake {
    public enum IntakeMode {
        CONE(1, 1),
        CUBE(-1, 1);

        private final double roller1;
        private final double roller2;

        private IntakeMode(double roller1, double roller2) {
            this.roller1 = roller1;
            this.roller2 = roller2;
        }

    }
}
