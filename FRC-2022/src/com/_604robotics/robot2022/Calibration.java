package com._604robotics.robot2022;

import com._604robotics.quixsam.mathematics.DoubleInterpolatableTreeMap;
import com._604robotics.quixsam.mathematics.Interpolatable;
import com._604robotics.robotnik.math.MathUtils;
import com._604robotics.robotnik.motorcontrol.controllers.MotorControllerPIDConfig;
import com._604robotics.robotnik.motorcontrol.gearing.GearRatio;
import com._604robotics.robotnik.swerve.QuixHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Calibration {
    public static final double g = 9.81;  // m/s/s

    public static final class Swerve {
        public static final double stickDeadband = 0.15;

        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.5);
        public static final double wheelBase = Units.inchesToMeters(20.5);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final GearRatio driveGearRatio = new GearRatio(1.0, 6.75); //6.75:1
        public static final GearRatio steeringGearRatio = new GearRatio(1.0, 12.8); //12.8:1

        /* Swerve Current Limiting */
        public static final int steeringContinuousCurrentLimit = 25;
        public static final int steeringPeakCurrentLimit = 40;
        public static final double steeringPeakCurrentDuration = 0.1;
        public static final boolean steeringEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Steering Motor PID Values */
        public static final MotorControllerPIDConfig steeringPIDConfig = new MotorControllerPIDConfig(0.2, 0.0, 0.0);

        /* Drive Motor PID Values */
        public static final MotorControllerPIDConfig drivePIDConfig = new MotorControllerPIDConfig(0.2, 0.0, 0.0);

        /* Drive Motor Characterization Values */
        public static final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.2, 2.2201, 0.16343);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5;  // m/s
        public static final double maxAngularVelocity = Math.PI * 2.0;  // rad/s

        /* Swerve Teleop Slew Rate Limits */
        public static final double linearSlewRate = 10.0;  // m/s/s
        public static final double angularSlewRate = 20.0;  // rad/s/s

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean steeringMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class FrontLeft {
            public static final Translation2d modulePosition = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
            public static final double angleOffset = 256.552734375;
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRight {
            public static final Translation2d modulePosition = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
            public static final double angleOffset = 213.486328125;
        }

        /* Rear Right Module - Module 2 */
        public static final class RearRight {
            public static final Translation2d modulePosition = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);
            public static final double angleOffset = 273.603515625;
        }

        /* Rear Left Module - Module 3 */
        public static final class RearLeft {
            public static final Translation2d modulePosition = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
            public static final double angleOffset = 203.73046875;
        }

    }

    public static final class Auto {
        public static final PIDController xController = new PIDController(3, 0.0, 0.0);
        public static final PIDController yController = new PIDController(3, 0.0, 0.0);
        public static final PIDController thetaController = new PIDController(2, 0.0, 0.0);

        public static final QuixHolonomicDriveController controller = new QuixHolonomicDriveController(xController, yController, thetaController);
    }

    public static final class PoseEstimator {
        public static final Matrix<N3, N1> stateStdDevs = Matrix.mat(Nat.N3(), Nat.N1()).fill(0.6, 0.6, Units.degreesToRadians(0.01));
        public static final Matrix<N1, N1> localMeasurementStdDevs = Matrix.mat(Nat.N1(), Nat.N1()).fill(Units.degreesToRadians(0.01));
        public static final Matrix<N3, N1> visionMeasurementStdDevs = Matrix.mat(Nat.N3(), Nat.N1()).fill(0.2, 0.2, Units.degreesToRadians(0.01));

        // Robot transforms
        public static final Matrix<N4, N4> robotToLauncherPivot = MathUtils.makeYRotTFMatrix(
            Units.inchesToMeters(2.0),
            Units.inchesToMeters(0),
            Units.inchesToMeters(20.25),
            Units.degreesToRadians(0)
        );

        public static final Matrix<N4, N4> launcherPivotToLauncher = MathUtils.makeYRotTFMatrix(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Units.degreesToRadians(10)
        );

        public static final Matrix<N4, N4> launcherToCamera = MathUtils.makeYRotTFMatrix(
            Units.inchesToMeters(-6.73),
            Units.inchesToMeters(0),
            Units.inchesToMeters(8.23),
            Units.degreesToRadians(-45)
        );

        // Precompute common inter-robot transforms.
        public static final Matrix<N4, N4> robotToCamera =
            Calibration.PoseEstimator.robotToLauncherPivot.times(
                Calibration.PoseEstimator.launcherPivotToLauncher.times(
                    Calibration.PoseEstimator.launcherToCamera));

        public static final Matrix<N4, N4> cameraToHorizontalCamera =
            MathUtils.makeYRotTFMatrix(
                0.0,
                0.0,
                0.0,
                new Rotation2d(
                    MathUtils.extractTFMatrixYRot(Calibration.PoseEstimator.robotToCamera)).unaryMinus().getRadians());

        public static final Matrix<N4, N4> robotToHorizontalCamera =
            Calibration.PoseEstimator.robotToCamera.times(
                Calibration.PoseEstimator.cameraToHorizontalCamera);

        public static final Matrix<N4, N4> horizontalCameraToRobot = robotToHorizontalCamera.inv();

        // Field transforms
        public static final double goalHeight = Units.inchesToMeters(102.3);
        public static final double goalDiameter = Units.inchesToMeters(53.0);
        public static final double goalRadius = goalDiameter / 2;

        public static final Matrix<N4, N4> fieldToGoal = MathUtils.makeYRotTFMatrix(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            goalHeight,
            Units.degreesToRadians(0)
        );

    }

    public static final class Intake {
        public static final double intakeSpeed = 0.4; //RPM
        public static final double ejectSpeed = -0.1; // yes this negative is intentional. Ball path roller provides most of the ejection energy.
        public static final double yeetSpeed = -0.3;  // we are yeeting under the intake roller.
        public static final double unJamSpeed = 0.1; //RPM
        public static final boolean intakeInvert = true;

        public static final GearRatio intakeRatio = new GearRatio(1.5, 1.0);
        public static final MotorControllerPIDConfig intakePID = new MotorControllerPIDConfig(0.1, 0.0, 0.0);
        public static final SimpleMotorFeedforward intakeFeedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

        public static final boolean deployInvert = false;

        public static final GearRatio deployRatio = new GearRatio(1.0, 6.43);
        public static final MotorControllerPIDConfig deployPID = new MotorControllerPIDConfig(0.2, 0.0, 0.0);
        public static final ArmFeedforward deployFeedforward = new ArmFeedforward(0.0, 0.35, 1.05);

        public static final TrapezoidProfile.Constraints deployProfileConstraints = new TrapezoidProfile.Constraints(
            3 * Math.PI, 4 * Math.PI
        );

        public static final TrapezoidProfile.State deployedState = new TrapezoidProfile.State(0, 0);
        public static final TrapezoidProfile.State retractedState = new TrapezoidProfile.State(Math.PI / 2, 0);
    }

    public static final class Launcher {
        public static final double launchHeight = Units.inchesToMeters(27);
        public static final double launchAngle = Math.toRadians(70.0);  // TODO: Pull from transforms

        public static final double ejectSpeed = 1000; // RPM
        public static final double unJamSpeed = 100; // RPM
        public static final double atSpeedTolerance = 100; // +/- RPM
        public static final boolean launcherInvert = false;

        public static final GearRatio launcherRatio = new GearRatio(1.0, 1.0);
        public static final MotorControllerPIDConfig launcherFrontPID = new MotorControllerPIDConfig(0.2, 0.0, 0.0);
        public static final SimpleMotorFeedforward launcherFrontFeedforward = new SimpleMotorFeedforward(0.5579, 0.00183, 2.7455e-5);

        public static final MotorControllerPIDConfig launcherRearPID = new MotorControllerPIDConfig(0.2, 0.0, 0.0);
        public static final SimpleMotorFeedforward launcherRearFeedforward = new SimpleMotorFeedforward(0.5579, 0.00183, 2.7455e-5);

        public static final double distanceToExitVel(double distance) {
            // Computes exit velocity given distance from the goal.
            // This is useful because it's easier to measure distance than exit velocity.

            // Y-component: 0 = dy - v0y * t + 0.5 * g * t^2
            // Solve for greater t, which is the "down" part of the parabola:
            // t = (v0y + sqrt(v0y^2 - 2 * g * dy)) / g
            // X-component: 0 = dx - v0x * t
            // Solve for t:
            // t = dx / v0x
            // Substitute and solve for v:
            // v0x = v * cos(theta)
            // v0y = v * sin(theta)
            double dy = PoseEstimator.goalHeight - Launcher.launchHeight;
            double v = (Math.sqrt(g) * distance) /
              (Math.cos(Launcher.launchAngle) * 
               Math.sqrt(2.0) *
               Math.sqrt(distance * Math.tan(Launcher.launchAngle) - dy));
            return v;
        }

        // Lookup of exit velocity in m/s to RPM.
        // 2.42 m/s horiz (7.08 m/s exit) -> 3000 RPM
        // 3.20 m/s horiz (9.36 m/s exit) -> 3600 RPM
        public static final DoubleInterpolatableTreeMap<Double> velocityToRPMLookup = new DoubleInterpolatableTreeMap<>(
            new Pair<>(distanceToExitVel(Units.inchesToMeters(88 - 13.5)), Interpolatable.interDouble(2950)),
            new Pair<>(distanceToExitVel(Units.inchesToMeters(94 - 13.5)), Interpolatable.interDouble(3050)),
            new Pair<>(distanceToExitVel(Units.inchesToMeters(111 - 13.5)), Interpolatable.interDouble(3150)),
            new Pair<>(distanceToExitVel(Units.inchesToMeters(124 - 13.5)), Interpolatable.interDouble(3350)),
            new Pair<>(distanceToExitVel(Units.inchesToMeters(134 - 13.5)), Interpolatable.interDouble(3450)),
            new Pair<>(distanceToExitVel(Units.inchesToMeters(158 - 13.5)), Interpolatable.interDouble(3750))
            // new Pair<>(6.0, Interpolatable.interDouble(2800)),
            // new Pair<>(9.0, Interpolatable.interDouble(3800))
        );
        public static final double minVel = distanceToExitVel(Units.inchesToMeters(88 - 13.5)); // TODO: Enforce consistency with velocityToRPMLookup.
        public static final double maxVel = distanceToExitVel(Units.inchesToMeters(158 - 13.5)); // TODO: Enforce consistency with velocityToRPMLookup.
        // public static final double minVel = 7.0; // TODO: Enforce consistency with velocityToRPMLookup.
        // public static final double maxVel = 10.0; // TODO: Enforce consistency with velocityToRPMLookup.

        public static final double minDistanceSOTF = Units.inchesToMeters(90);
        public static final double maxDistanceSOTF = Units.inchesToMeters(110);

        public static final double sotfConstantTangentialVelocity = 1.0; // m/s
    }


    public static final class BallPath {
        public static final double intakePower = 0.6;
        public static final double intakeEjectPower = 0.6;
        public static final double intakeYeetPower = 0.6;
        public static final double shootFeedSpeedSlow = 0.5;  // m/s
        public static final double shootFeedSpeedFast = 0.5;  // m/s
        // public static final double ejectSpeed = Units.rotationsPerMinuteToRadiansPerSecond(100); //RPM
        // public static final double unJamSpeed = Units.rotationsPerMinuteToRadiansPerSecond(-100); //RPM
        // public static final boolean launcherInvert = false;

        public static final GearRatio ballPathTopRatio = new GearRatio(1.0, 1.5);
        public static final MotorControllerPIDConfig ballPathTopPID = new MotorControllerPIDConfig(0, 0.1, 0.0, 0.0);
        public static final MotorControllerPIDConfig ballPathTopVelocityPID = new MotorControllerPIDConfig(1, 0.01, 0.0, 0.0);
        public static final SimpleMotorFeedforward ballPathTopVelcoityFeedforward = new SimpleMotorFeedforward(2.5, 1.0143, 0.02066);


        public static final GearRatio ballPathBottomRatio = new GearRatio(1.0, 1.5);
        public static final MotorControllerPIDConfig ballPathBottomPID = new MotorControllerPIDConfig(0, 0.1, 0.0, 0.0);
    }

    public static final class Climber {
        public static final boolean invert = true;

        public static final GearRatio climberRatio = new GearRatio(12.0, 60.0);
        public static final double pitchDiameter = Units.inchesToMeters(1.432);  // 18T #25 sprocket
        public static final MotorControllerPIDConfig climberPID = new MotorControllerPIDConfig(3.5, 0.01, 0.0);
        public static final double iZone = Units.inchesToMeters(2.0);  // in
        public static final double iMax = 0.25;  // V

        public static final double bottomSoftLimit = 0.01;
        public static final double topSoftLimit = 1.42; 

        public static final double firstRungPositon = 1.42;  // reach first rung
        public static final double touchFirstRungPositon = 1.35; // latch onto first rung
        public static final double partialLiftPosition = 0.7; // lift partially
        public static final double ratchetPosition = 0.01; // lift to stationary hook
        public static final double handoffPosition = 0.13; // latch onto stationary hook
        public static final double nextRungPositon = 1.05;  // reach next rung

        public static final double positionTolerance = Units.inchesToMeters(0.5);
        public static final double fixedTopZeroAngle = 30;  // Angle when hanging on the fixed top hook (degrees)
        public static final double fixedTopZeroAngleTolerance = 1;  // degrees
        public static final double antennaTouchingAngle = 58;  // Angle when antennas are touching
        public static final double antennaTouchingAngleTolerance = 2;  // degrees

        public static final double unloadedffkS = 0.0;
        public static final double unloadedffkG = -0.09;
        public static final double unloadedffkV = 6;
        public static final double unloadedffkA = 0.0;

        public static final ElevatorFeedforward unloadedFF = new ElevatorFeedforward(
            unloadedffkS,
            unloadedffkG,
            unloadedffkV,
            unloadedffkA
        );

        public static final double loadedffkS = 0.0;
        public static final double loadedffkG = -0.9;
        public static final double loadedffkV = 6;
        public static final double loadedffkA = 0.0;

        public static final ElevatorFeedforward loadedFF = new ElevatorFeedforward(
            loadedffkS,
            loadedffkG,
            loadedffkV,
            loadedffkA
        );

        public static final TrapezoidProfile.Constraints moveFastTrapConstraints = new TrapezoidProfile.Constraints(2.0, 4.0);  // velocity, acceleration
        public static final TrapezoidProfile.Constraints moveTransitionTrapConstraints = new TrapezoidProfile.Constraints(1.0, 4.0);  // velocity, acceleration
        public static final TrapezoidProfile.Constraints moveSlowTrapConstraints = new TrapezoidProfile.Constraints(0.5, 2.0);  // velocity, acceleration
    }
}
