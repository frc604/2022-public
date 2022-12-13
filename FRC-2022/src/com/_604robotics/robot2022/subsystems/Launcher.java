package com._604robotics.robot2022.subsystems;

import java.util.function.DoubleSupplier;

import com._604robotics.robot2022.Calibration;
import com._604robotics.robot2022.Constants;
import com._604robotics.robot2022.ShotCalculator;
import com._604robotics.robotnik.devices.FalconEncoder;
import com._604robotics.robotnik.motorcontrol.Motor;
import com._604robotics.robotnik.motorcontrol.QuixTalonFX;
import com._604robotics.robotnik.motorcontrol.controllers.TalonPID;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase{
    private final QuixTalonFX launcherFrontMotor = new QuixTalonFX(
        Constants.Launcher.launcherFrontMotor,
        "Launcher Front Motor",
        Motor.kFalcon500,
        this
    );

    private final QuixTalonFX launcherRearMotor = new QuixTalonFX(
        Constants.Launcher.launcherRearMotor,
        "Launcher Rear Motor",
        Motor.kFalcon500,
        this
    );

    private final FalconEncoder launcherFrontEncoder = new FalconEncoder(
        launcherFrontMotor,
        Calibration.Launcher.launcherRatio
    );

    private final FalconEncoder launcherRearEncoder = new FalconEncoder(
        launcherRearMotor,
        Calibration.Launcher.launcherRatio
    );


    private final TalonPID launcherFrontPID = new TalonPID(
        launcherFrontMotor,
        launcherFrontEncoder,
        Calibration.Launcher.launcherFrontPID
    );

    private final TalonPID launcherRearPID = new TalonPID(
        launcherRearMotor,
        launcherRearEncoder,
        Calibration.Launcher.launcherRearPID
    );

    private final SimpleMotorFeedforward launcherFrontFeedforward = Calibration.Launcher.launcherFrontFeedforward;
    private final SimpleMotorFeedforward launcherRearFeedforward = Calibration.Launcher.launcherRearFeedforward;

    private final ShotCalculator shotCalculator;

    private double targetVelocity = 0;
    private boolean isLaunch = false;

    private int speedBufferSize = 5;  // 5 samples is 100 ms
    CircularBuffer frontSpeeds = new CircularBuffer(speedBufferSize);
    CircularBuffer rearSpeeds = new CircularBuffer(speedBufferSize);

    public Launcher(ShotCalculator shotCalculator) {
        this.shotCalculator = shotCalculator;
        launcherRearMotor.setInverted(true);

        launcherRearMotor.getController().setNeutralMode(NeutralMode.Coast);
        launcherFrontMotor.getController().setNeutralMode(NeutralMode.Coast);

        // Conver to RPM
        launcherFrontEncoder.setdistancePerRotation(60);
        launcherRearEncoder.setdistancePerRotation(60);

        SmartDashboard.putNumber("Shooter Setpoint", 3200);
    }

    public void stop() {
        launcherFrontMotor.stopMotor();
        launcherRearMotor.stopMotor();
    }

    public void run(double targetVelocity) {
        launcherFrontPID.setSetpointVelocity(targetVelocity, launcherFrontFeedforward.calculate(targetVelocity));
        launcherRearPID.setSetpointVelocity(targetVelocity, launcherRearFeedforward.calculate(targetVelocity));
        this.targetVelocity = targetVelocity;
    }

    private boolean allAtSpeed(CircularBuffer buffer, double targetSpeed) {
        for (int i=0; i<speedBufferSize; ++i) {
            if (Math.abs(buffer.get(i) - targetSpeed) > Calibration.Launcher.atSpeedTolerance) {
                return false;
            }
        }
        return true;
    }

    public boolean atSpeed() {
        boolean atSpeed = false;
        var shotInfo = shotCalculator.computeShotInfo();
        if (isLaunch && !shotInfo.feasible) {
            atSpeed = false;
        } else if (allAtSpeed(frontSpeeds, targetVelocity) && allAtSpeed(rearSpeeds, targetVelocity)) {
            atSpeed = true;
        }
        SmartDashboard.putBoolean("Launcher At Speed", atSpeed);
        return atSpeed;
    }

    public boolean atSpeedWithoutDistance() {
        boolean atSpeed = false;
        // If we are launching at the min or max RPMs, we are likely too close or too far.
        if (allAtSpeed(frontSpeeds, targetVelocity) && allAtSpeed(rearSpeeds, targetVelocity)) {
            atSpeed = true;
        }

        SmartDashboard.putBoolean("Launcher At Speed Without Distance", atSpeed);
        return atSpeed;
    }

    public void launch() {
        var shotInfo = shotCalculator.computeShotInfo();
        double rpmSetpoint = Calibration.Launcher.velocityToRPMLookup.get(shotInfo.velocity);
        // SmartDashboard.putNumber("Shooter Setpoint", rpmSetpoint);
        run(rpmSetpoint);
        isLaunch = true;
    }

    public void launchFromDistance(double distance) {
        run(Calibration.Launcher.velocityToRPMLookup.get(Calibration.Launcher.distanceToExitVel(distance)));
        isLaunch = true;
    }

    public void eject() {
        run(Calibration.Launcher.ejectSpeed);
        isLaunch = false;
    }

    public void unJam() {
        run(Calibration.Launcher.unJamSpeed);
        isLaunch = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Launcher Target Speed", targetVelocity);
        SmartDashboard.putNumber("Launcher Front Speed", launcherFrontEncoder.getVelocity());
        SmartDashboard.putNumber("Launcher Rear Speed", launcherRearEncoder.getVelocity());

        frontSpeeds.addFirst(launcherFrontEncoder.getVelocity());
        rearSpeeds.addFirst(launcherRearEncoder.getVelocity());
    }
}
