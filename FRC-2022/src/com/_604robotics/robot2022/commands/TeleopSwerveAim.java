package com._604robotics.robot2022.commands;
import com._604robotics.quixsam.mathematics.DoubleInterpolatableTreeMap;
import com._604robotics.robot2022.Calibration;
import com._604robotics.robot2022.subsystems.Swerve;
import com._604robotics.robotnik.math.MathUtils;
import com._604robotics.robotnik.motion.MotionConstraints;
import com._604robotics.robotnik.motion.MotionState;
import com._604robotics.robotnik.motion.TrapezoidalMotionProfile;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerveAim extends CommandBase {
    private double rotation;
    private Translation2d translation;
    private double scaleFactor;
    private boolean fieldRelative;
    private boolean squared;

    private PIDController aimPID;
    private TrapezoidalMotionProfile aimProfile;

    private DoubleInterpolatableTreeMap<Double> sussy = new DoubleInterpolatableTreeMap<>();

    private Swerve swerve;
    private XboxController controller;

    SlewRateLimiter xVelRateLimiter = new SlewRateLimiter(Calibration.Swerve.linearSlewRate);
    SlewRateLimiter yVelRateLimiter = new SlewRateLimiter(Calibration.Swerve.linearSlewRate);
    SlewRateLimiter angularRateLimiter = new SlewRateLimiter(Calibration.Swerve.angularSlewRate);

    private double prevTime;
    private boolean first = true;

    public TeleopSwerveAim(Swerve swerve, XboxController controller, boolean fieldRelative, boolean squared) {
        this(swerve, controller, 1.0, fieldRelative, squared);
    }

    public TeleopSwerveAim(Swerve swerve, XboxController controller, double scaleFactor, boolean fieldRelative, boolean squared) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.controller = controller;
        this.scaleFactor = scaleFactor;
        this.fieldRelative = fieldRelative;
        this.squared = squared;

        this.aimPID = new PIDController(7, 0, 0.4);
        // this.aimPID.enableContinuousInput(0, Math.PI * 2);
        // this.aimProfile = new TrapezoidalMotionProfile(new MotionState(), new MotionState(), new MotionConstraints(0, 0));

        swerve.zeroModuleEncoders();
    }

    @Override
    public void initialize() {
        prevTime = Timer.getFPGATimestamp();

        first = true;
    }

    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - prevTime;
        prevTime = currentTime;

        double yAxis = controller.getLeftY();
        double xAxis = controller.getLeftX();
        double rAxis = controller.getRightX();

        /* Deadbands */
        double deadband = Calibration.Swerve.stickDeadband;
        var deadbandedXY = applyDeadbandOnXY(xAxis, yAxis, deadband);
        xAxis = deadbandedXY.getFirst();
        yAxis = deadbandedXY.getSecond();
        rAxis = applyDeadband(rAxis, deadband);

        if (squared) {
            yAxis = square(yAxis);
            xAxis = square(xAxis);
            rAxis = square(rAxis);
        }

        /* Scale */
        yAxis *= scaleFactor;
        xAxis *= scaleFactor;
        rAxis *= scaleFactor;

        // Convert joystick axes to velocities.
        // Note that axes are swapped and negated.
        double xVel = -yAxis * Calibration.Swerve.maxSpeed;
        double yVel = -xAxis * Calibration.Swerve.maxSpeed;
        double angularVel = -rAxis * Calibration.Swerve.maxAngularVelocity;

        // Apply slew rates on velocities.
        translation = new Translation2d(
            xVelRateLimiter.calculate(xVel),
            yVelRateLimiter.calculate(yVel));
        rotation = angularRateLimiter.calculate(angularVel);

        // Compute angle to goal, compensating for linear velocity.
        // Note that this doesn't exactly account for the launcher's position on the robot.
        Pose2d estimatedPose = swerve.getEstimatedPose();
        double estimatedYaw = estimatedPose.getRotation().getRadians();

        // Compute ball horizontal velocity vector.
        double ballExitVel = Calibration.Launcher.velocityToRPMLookup.get(swerve.getDistanceToGoal());
        double launcherAngle = Math.toRadians(70.0);  // TODO: Pull from calibration
        double ballHorizontalVel = ballExitVel * Math.cos(launcherAngle);
        Matrix<N2, N1> ballHorizontalVelVec = Matrix.mat(Nat.N2(), Nat.N1()).fill(
            ballHorizontalVel * Math.cos(estimatedYaw),
            ballHorizontalVel * Math.sin(estimatedYaw));

        // Compute shotYaw, which is the angle of (robot velocity + ball velocity).
        Matrix<N2, N1> totalVel = ballHorizontalVelVec;
        double shotYaw = MathUtils.unwrapAngle(
            Math.atan2(totalVel.get(1, 0), totalVel.get(0, 0)),
            estimatedYaw);

        double targetYaw = MathUtils.unwrapAngle(
            Math.atan2(-estimatedPose.getY(), -estimatedPose.getX()),
            estimatedYaw);

        // SmartDashboard.putNumber("Shot Yaw", shotYaw);
        // SmartDashboard.putNumber("Target Yaw", targetYaw);

        swerve.drive(translation, aimPID.calculate(shotYaw, targetYaw), fieldRelative);
    }

    // Apply deadband in joystick polar coordinates instead of on the raw joystick values.
    private Pair<Double, Double> applyDeadbandOnXY(double x, double y, double deadband) {
        var rAndTheta = MathUtils.cart2pol(x, y);
        double deadbandedR = applyDeadband(rAndTheta.getFirst(), deadband);
        return MathUtils.pol2cart(deadbandedR, rAndTheta.getSecond());
    }

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0;
        } else {
            return (1 / (1 - deadband))  * (value - 1 * Math.signum(value)) + 1 * Math.signum(value);
        }
    }

    private double square(double value) {
        return Math.signum(value) * Math.pow(value, 2);
    }
}
