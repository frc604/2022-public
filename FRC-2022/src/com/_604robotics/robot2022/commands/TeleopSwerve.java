package com._604robotics.robot2022.commands;
import com._604robotics.robot2022.Calibration;
import com._604robotics.robot2022.subsystems.Swerve;
import com._604robotics.robotnik.math.MathUtils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {
    private double rotation;
    private Translation2d translation;
    private double scaleFactor;
    private boolean fieldRelative;
    private boolean squared;

    private Swerve swerve;
    private XboxController controller;

    SlewRateLimiter xVelRateLimiter = new SlewRateLimiter(Calibration.Swerve.linearSlewRate);
    SlewRateLimiter yVelRateLimiter = new SlewRateLimiter(Calibration.Swerve.linearSlewRate);
    SlewRateLimiter angularRateLimiter = new SlewRateLimiter(Calibration.Swerve.angularSlewRate);

    public TeleopSwerve(Swerve swerve, XboxController controller, boolean fieldRelative, boolean squared) {
        this(swerve, controller, 1.0, fieldRelative, squared);
    }

    public TeleopSwerve(Swerve swerve, XboxController controller, double scaleFactor, boolean fieldRelative, boolean squared) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.controller = controller;
        this.scaleFactor = scaleFactor;
        this.fieldRelative = fieldRelative;
        this.squared = squared;

        swerve.zeroModuleEncoders();
    }

    @Override
    public void execute() {
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

        swerve.drive(translation, rotation, fieldRelative);
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
