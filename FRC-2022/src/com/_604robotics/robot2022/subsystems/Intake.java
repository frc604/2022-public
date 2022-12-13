package com._604robotics.robot2022.subsystems;

import com._604robotics.robot2022.Calibration;
import com._604robotics.robot2022.Constants;
import com._604robotics.robotnik.devices.FalconEncoder;
import com._604robotics.robotnik.motorcontrol.Motor;
import com._604robotics.robotnik.motorcontrol.QuixTalonFX;
import com._604robotics.robotnik.motorcontrol.controllers.TalonPID;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final QuixTalonFX intakeMotor = new QuixTalonFX(
        Constants.Intake.intakeMotorPort,
        "Intake Motor",
        Motor.kFalcon500,
        this
    );

    private final FalconEncoder intakeEncoder = new FalconEncoder(
        intakeMotor,
        Calibration.Intake.intakeRatio
    );

    private final TalonPID intakePID = new TalonPID(
        intakeMotor,
        intakeEncoder,
        Calibration.Intake.intakePID
    );

    private final SimpleMotorFeedforward intakeFeedforward = Calibration.Intake.intakeFeedforward;


    private final QuixTalonFX deployMotor = new QuixTalonFX(
        Constants.Intake.deployMotorPort,
        "Intake Deploy Motor",
        Motor.kFalcon500,
        this
    );

    private final FalconEncoder deployEncoder = new FalconEncoder(
        deployMotor,
        Calibration.Intake.deployRatio
    );

    private final TalonPID deployPID = new TalonPID(
        deployMotor,
        deployEncoder,
        Calibration.Intake.deployPID
    );

    private final Timer deployProfileTimer = new Timer();

    private TrapezoidProfile deployProfile;

    private final ArmFeedforward deployFeedforward = Calibration.Intake.deployFeedforward;

    public Intake() {
        deployMotor.getController().configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 45, 45, 0.0));

        intakeMotor.setInverted(Calibration.Intake.intakeInvert);
        intakeEncoder.setdistancePerRotation(2 * Math.PI);

        deployMotor.setInverted(Calibration.Intake.deployInvert);
        deployEncoder.setdistancePerRotation(Math.PI / 2);

        deployEncoder.zero();

        deployProfile = new TrapezoidProfile(
            Calibration.Intake.deployProfileConstraints,
            Calibration.Intake.deployedState,
            new TrapezoidProfile.State(
                deployEncoder.getPosition(),
                0.0
            )
        );
        
        deployProfileTimer.reset();
        deployProfileTimer.start();

        // Start intake retracted.
        retract();
    }

    public void run(double velocity) {
        intakeMotor.set(velocity);
    }

    public void intake() {
        run(Calibration.Intake.intakeSpeed);
    }

    public void eject() {
        run(Calibration.Intake.ejectSpeed);
    }

    public void unJam() {
        run(Calibration.Intake.unJamSpeed);
    }

    public void yeet() {
        run(Calibration.Intake.yeetSpeed);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    public void deploy() {
        deployProfile = new TrapezoidProfile(
            Calibration.Intake.deployProfileConstraints,
            Calibration.Intake.deployedState,
            new TrapezoidProfile.State(
                deployEncoder.getPosition(),
                0.0
            )
        );

        deployProfileTimer.reset();
        deployProfileTimer.start();
    }

    public void retract() {
        deployProfile = new TrapezoidProfile(
            Calibration.Intake.deployProfileConstraints,
            Calibration.Intake.retractedState,
            new TrapezoidProfile.State(
                deployEncoder.getPosition(),
                0.0
            )
        );

        deployProfileTimer.reset();
        deployProfileTimer.start(); 
    }

    @Override
    public void periodic() {
        TrapezoidProfile.State desiredState = deployProfile.calculate(deployProfileTimer.get());

        SmartDashboard.putNumber("Deploy Position", deployEncoder.getPosition());
        SmartDashboard.putNumber("Deploy Setpoint", desiredState.position);

        SmartDashboard.putNumber("Deploy Current", deployMotor.getController().getStatorCurrent());

        deployPID.setSetpointPosition(desiredState.position, deployFeedforward.calculate(desiredState.position, desiredState.velocity));
    }
}
