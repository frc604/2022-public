package com._604robotics.robot2022.commands;
import com._604robotics.quixsam.mathematics.DoubleInterpolatableTreeMap;
import com._604robotics.robot2022.Calibration;
import com._604robotics.robot2022.ShotCalculator;
import com._604robotics.robot2022.subsystems.Swerve;
import com._604robotics.robotnik.math.MathUtils;
import com._604robotics.robotnik.motion.TrapezoidalMotionProfile;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerveAimRadial extends CommandBase {
    private double rotation;
    private Translation2d translation;
    private double scaleFactor;
    private boolean fieldRelative;
    private boolean squared;

    private PIDController aimPID;
    private final PIDController radiusPID;

    private TrapezoidalMotionProfile aimProfile;

    private DoubleInterpolatableTreeMap<Double> sussy = new DoubleInterpolatableTreeMap<>();

    private Swerve swerve;
    private XboxController controller;
    private ShotCalculator shotCalculator;

    SlewRateLimiter xVelRateLimiter = new SlewRateLimiter(Calibration.Swerve.linearSlewRate);
    SlewRateLimiter yVelRateLimiter = new SlewRateLimiter(Calibration.Swerve.linearSlewRate);
    SlewRateLimiter angularRateLimiter = new SlewRateLimiter(Calibration.Swerve.angularSlewRate);

    private double lockedRadius;
    private double prevTime;

    public TeleopSwerveAimRadial(Swerve swerve, XboxController controller, ShotCalculator shotCalculator, boolean fieldRelative, boolean squared) {
        this(swerve, controller, shotCalculator, 1.0, fieldRelative, squared);
    }

    public TeleopSwerveAimRadial(Swerve swerve, XboxController controller, ShotCalculator shotCalculator, double scaleFactor, boolean fieldRelative, boolean squared) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.controller = controller;
        this.shotCalculator = shotCalculator;
        this.scaleFactor = scaleFactor;
        this.fieldRelative = fieldRelative;
        this.squared = squared;

        this.aimPID = new PIDController(7, 0, 0.4);
        this.radiusPID = new PIDController(2, 0, 0);
        // this.aimPID.enableContinuousInput(0, Math.PI * 2);
        // this.aimProfile = new TrapezoidalMotionProfile(new MotionState(), new MotionState(), new MotionConstraints(0, 0));

        swerve.zeroModuleEncoders();
    }

    @Override
    public void initialize() {
        // Calculate radius lock distance
        lockedRadius = MathUtil.clamp(swerve.getDistanceToGoal(), Calibration.Launcher.minDistanceSOTF, Calibration.Launcher.maxDistanceSOTF);

        prevTime = Timer.getFPGATimestamp();
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
        double xVel = -yAxis * 2.0;
        double yVel = -xAxis * 2.0;
        double angularVel = -rAxis * Calibration.Swerve.maxAngularVelocity;

        // v = r dot w
        double tangentialVelocity = Calibration.Launcher.sotfConstantTangentialVelocity * Math.signum(xAxis);
        swerve.setOrbitTangentialVelocity(tangentialVelocity);

        // UCM
        double xSpeedCircular = tangentialVelocity * -swerve.getAngleAroundGoal().getSin();
        double ySpeedCircular = tangentialVelocity * swerve.getAngleAroundGoal().getCos();

        // Radius Convergence Vector
        double radiusConvergence = radiusPID.calculate(swerve.getDistanceToGoal(), lockedRadius);

        xSpeedCircular  = xSpeedCircular + radiusConvergence * Math.cos(swerve.getAngleAroundGoal().getRadians());
        ySpeedCircular  = ySpeedCircular + radiusConvergence * Math.sin(swerve.getAngleAroundGoal().getRadians());

        translation = new Translation2d(xSpeedCircular, ySpeedCircular);

        // Desired shot yaw and launch velocity.
        var shotInfo = shotCalculator.computeShotInfo();

        double estimatedYaw = swerve.getEstimatedPose().getRotation().getRadians();

        // SmartDashboard.putNumber("Current Yaw", estimatedYaw);
        // SmartDashboard.putNumber("Target Yaw", shotInfo.yaw);

        swerve.drive(translation, aimPID.calculate(estimatedYaw, shotInfo.yaw), fieldRelative);
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
