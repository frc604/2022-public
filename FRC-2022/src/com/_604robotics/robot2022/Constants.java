package com._604robotics.robot2022;

public final class Constants {
    public static final class Swerve {
        public static final int pigeonID = 0;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class FrontLeft {
            public static final int driveMotorPort = 0;
            public static final int steeringMotorPort = 1;
            public static final int canCoderPort = 0;
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRight {
            public static final int driveMotorPort = 2;
            public static final int steeringMotorPort = 3;
            public static final int canCoderPort = 1;
        }
        
        /* Rear Right Module - Module 2 */
        public static final class RearRight {
            public static final int driveMotorPort = 4;
            public static final int steeringMotorPort = 5;
            public static final int canCoderPort = 2;
        }

        /* Rear Left Module - Module 3 */
        public static final class RearLeft {
            public static final int driveMotorPort = 6;
            public static final int steeringMotorPort = 7;
            public static final int canCoderPort = 3;
        }
    }

    public static final class Intake {
        public static final int intakeMotorPort = 8;
        public static final int deployMotorPort = 16;
    }

    public static final class Launcher {
        public static final int launcherFrontMotor = 11;
        public static final int launcherRearMotor = 12;
    }

    public static final class BallPath {
        public static final int ballPathTopMotor = 10;
        public static final int ballPathBottomMotor = 9;
        
        public static final int ballPathInakeBeamBreak = 1;
        public static final int ballPathBottomBeamBreak = 0;
        public static final int ballPathTopBeamBreak = 2;

        public static final int ballPathColorSensor = 3;
    }

    public static final class Climber {
        public static final int climberMotor0 = 13;
        public static final int climberMotor1 = 14;
        public static final int climberMotor2 = 15;
    }
}
