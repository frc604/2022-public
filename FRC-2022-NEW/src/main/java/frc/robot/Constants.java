// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.quixlib.math.MathUtils;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.motorcontrol.PIDConfig;
import frc.quixlib.swerve.QuixSwerveController;
import frc.quixlib.vision.Fiducial;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int pigeonID = 0;
  public static final String limelightTableName = "limelight";

  public static final class Intake {
    public static final int intakeID = 8;
    public static final int deployID = 16;

    public static final MechanismRatio rollerRatio = new MechanismRatio(1.0, 1.5);
    public static final MechanismRatio deployArmRatio = new MechanismRatio(1.0, 25.7);

    public static final PIDConfig deployPIDConfig = new PIDConfig(0.1, 0.0, 0.0);
    public static final ArmFeedforward deployFF = new ArmFeedforward(0, 0.7, 0.45);
    public static final Constraints deployTrapazoidalConstraints =
        new Constraints(2.0 * Math.PI, 4.0 * Math.PI);

    // Approximations for simulation.
    public static final double deployArmMass = 3.5; // kg
    public static final double deployArmLength = 0.2; // m
    public static final double rollerMOI = 0.0005; // Moment of inertia (kg * m^2)
  }

  public static final class Ballpath {
    public static final int ballPathTopMotor = 10;
    public static final int ballPathBottomMotor = 9;

    public static final MechanismRatio rollerRatio = new MechanismRatio(1.0, 1.5);

    // Approximations for simulation.
    public static final double rollerMOI = 0.0005; // Moment of inertia (kg * m^2)
  }

  public static final class Climber {
    public static final int climberMotor0 = 13;
    public static final int climberMotor1 = 14;
    public static final int climberMotor2 = 15;
    public static final boolean climberMotorInvert = true;

    public static final double drumRadius = Units.inchesToMeters(0.6);
    public static final MechanismRatio climberRatio =
        new MechanismRatio(12.0, 60.0, 2.0 * Math.PI * drumRadius);

    public static final double minHeight = 0.0; // m
    public static final double maxHeight = 1.42; // m

    // Approximations for simulation.
    public static final double carriageMass = 5.0; // kg
  }

  public static final class Launcher {
    public static final int launcherFrontMotorID = 11;
    public static final int launcherRearMotorID = 12;
    public static final boolean launcherFrontMotorInvert = true;

    public static final MechanismRatio flywheelRatio = new MechanismRatio(1.0, 1.0);

    // Approximations for simulation.
    public static final double MOI = 0.0012; // Moment of inertia (kg * m^2)
  }

  public static final class Swerve {
    public static final double maxDriveSpeed = 4.5; // m/s
    public static final double maxAngularVelocity = Math.PI * 2.0; // rad/s
    public static final double trackWidth = Units.inchesToMeters(20.5);
    public static final double wheelBase = Units.inchesToMeters(20.5);
    public static final double wheelDiameter = Units.inchesToMeters(3.94);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final MechanismRatio driveRatio =
        new MechanismRatio(1.0, 6.75, wheelCircumference);
    public static final MechanismRatio steeringRatio = new MechanismRatio(1.0, 12.8);
    public static final PIDConfig drivePIDConfig = new PIDConfig(0.2, 0.0, 0.0);
    public static final SimpleMotorFeedforward driveFeedforward =
        new SimpleMotorFeedforward(0.2, 2.2201, 0.16343);
    public static final PIDConfig steeringPIDConfig = new PIDConfig(0.2, 0.0, 0.0);

    /* Swerve Teleop Slew Rate Limits */
    public static final double linearSlewRate = 100.0; // m/s/s
    public static final double angularSlewRate = 200.0; // rad/s/s
    public static final double stickDeadband = 0.15;

    /* Module Slew Rate Limits */
    // Max module acceleration can be high since drive wheels can be backdriven.
    public static final double maxModuleAcceleration = 1000.0; // m/s/s
    public static final double maxModuleSteeringRate = 4.0 * Math.PI; // rad/s/s

    /* Allowable scrub */
    public static final double autoScrubLimit = 0.05; // m/s
    public static final double teleopScrubLimit = 0.25; // m/s

    public static final QuixSwerveController driveController =
        new QuixSwerveController(
            new PIDController(5, 0, 0), new PIDController(5, 0, 0), new PIDController(5, 0, 0));

    /* Front Left Module - Module 0 */
    public static final class FrontLeft {
      public static final int driveMotorID = 0;
      public static final int steeringMotorID = 1;
      public static final int canCoderID = 0;
      public static final Translation2d modulePosition =
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
      public static final double absEncoderOffsetRad = Math.toRadians(256.552734375);
    }

    /* Front Right Module - Module 1 */
    public static final class FrontRight {
      public static final int driveMotorID = 2;
      public static final int steeringMotorID = 3;
      public static final int canCoderID = 1;
      public static final Translation2d modulePosition =
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
      public static final double absEncoderOffsetRad = Math.toRadians(213.486328125);
    }

    /* Rear Right Module - Module 2 */
    public static final class RearRight {
      public static final int driveMotorID = 4;
      public static final int steeringMotorID = 5;
      public static final int canCoderID = 2;
      public static final Translation2d modulePosition =
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);
      public static final double absEncoderOffsetRad = Math.toRadians(273.603515625);
    }

    /* Rear Left Module - Module 3 */
    public static final class RearLeft {
      public static final int driveMotorID = 6;
      public static final int steeringMotorID = 7;
      public static final int canCoderID = 3;
      public static final Translation2d modulePosition =
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
      public static final double absEncoderOffsetRad = Math.toRadians(203.73046875);
    }
  }

  // Placeholder connected to Example Subsystem so it doesn't error.
  public static final class Example {
    public static final int motorID = 99;
    public static final MechanismRatio motorRatio = new MechanismRatio(1.0, 1.0);
  }

  public static final class Transforms {
    static final Matrix<N4, N4> robotToLauncherPivot =
        MathUtils.makeYRotTFMatrix(
            Units.inchesToMeters(2.0),
            Units.inchesToMeters(0),
            Units.inchesToMeters(20.25),
            Units.degreesToRadians(0));

    static final Matrix<N4, N4> launcherPivotToLauncher =
        MathUtils.makeYRotTFMatrix(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Units.degreesToRadians(5));

    static final Matrix<N4, N4> launcherToCamera =
        MathUtils.makeYRotTFMatrix(
            Units.inchesToMeters(-6.73),
            Units.inchesToMeters(0),
            Units.inchesToMeters(8.23),
            Units.degreesToRadians(-45));

    public static final Matrix<N4, N4> robotToCamera =
        robotToLauncherPivot.times(launcherPivotToLauncher.times(launcherToCamera));
  }

  public static final Fiducial[] aprilTags =
      new Fiducial[] {
        Fiducial.aprilTagFromBlueAllianceCoords(0, -0.004, 7.579, 0.886, 0, 0, 0),
        Fiducial.aprilTagFromBlueAllianceCoords(1, 3.233, 5.487, 1.725, 0, 0, 0),
        Fiducial.aprilTagFromBlueAllianceCoords(2, 3.068, 5.331, 1.376, 0, 0, Math.toRadians(-90)),
        Fiducial.aprilTagFromBlueAllianceCoords(3, 0.004, 5.059, 0.806, 0, 0, 0),
        Fiducial.aprilTagFromBlueAllianceCoords(4, 0.004, 3.512, 0.806, 0, 0, 0),
        Fiducial.aprilTagFromBlueAllianceCoords(
            5, 0.121, 1.718, 0.891, 0, 0, Math.toRadians(46.25)),
        Fiducial.aprilTagFromBlueAllianceCoords(
            6, 0.873, 0.941, 0.891, 0, 0, Math.toRadians(46.25)),
        Fiducial.aprilTagFromBlueAllianceCoords(
            7, 1.615, 0.157, 0.891, 0, 0, Math.toRadians(46.25)),
        Fiducial.aprilTagFromBlueAllianceCoords(
            10, 16.463, 0.651, 0.886, 0, 0, Math.toRadians(180)),
        Fiducial.aprilTagFromBlueAllianceCoords(
            11, 13.235, 2.743, 1.725, 0, 0, Math.toRadians(180)),
        Fiducial.aprilTagFromBlueAllianceCoords(12, 13.391, 2.900, 1.376, 0, 0, Math.toRadians(90)),
        Fiducial.aprilTagFromBlueAllianceCoords(
            13, 16.455, 3.176, 0.806, 0, 0, Math.toRadians(180)),
        Fiducial.aprilTagFromBlueAllianceCoords(
            14, 16.455, 4.717, 0.806, 0, 0, Math.toRadians(180)),
        Fiducial.aprilTagFromBlueAllianceCoords(
            15, 16.335, 6.515, 0.894, 0, 0, Math.toRadians(223.8)),
        Fiducial.aprilTagFromBlueAllianceCoords(
            16, 15.590, 7.293, 0.891, 0, 0, Math.toRadians(223.8)),
        Fiducial.aprilTagFromBlueAllianceCoords(
            17, 14.847, 8.069, 0.891, 0, 0, Math.toRadians(223.8)),
        Fiducial.aprilTagFromBlueAllianceCoords(40, 7.874, 4.913, 0.703, 0, 0, Math.toRadians(114)),
        Fiducial.aprilTagFromBlueAllianceCoords(41, 7.431, 3.759, 0.703, 0, 0, Math.toRadians(204)),
        Fiducial.aprilTagFromBlueAllianceCoords(42, 8.585, 3.316, 0.703, 0, 0, Math.toRadians(-66)),
        Fiducial.aprilTagFromBlueAllianceCoords(43, 9.028, 4.470, 0.703, 0, 0, Math.toRadians(24)),
        Fiducial.aprilTagFromBlueAllianceCoords(
            50, 7.679, 4.326, 2.418, 0, Math.toRadians(26.75), Math.toRadians(159)),
        Fiducial.aprilTagFromBlueAllianceCoords(
            51, 8.018, 3.564, 2.418, 0, Math.toRadians(26.75), Math.toRadians(249)),
        Fiducial.aprilTagFromBlueAllianceCoords(
            52, 8.780, 3.903, 2.418, 0, Math.toRadians(26.75), Math.toRadians(339)),
        Fiducial.aprilTagFromBlueAllianceCoords(
            53, 8.441, 4.665, 2.418, 0, Math.toRadians(26.75), Math.toRadians(69)),
      };
}
