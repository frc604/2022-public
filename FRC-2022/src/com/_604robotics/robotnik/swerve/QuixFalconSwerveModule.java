package com._604robotics.robotnik.swerve;

import com._604robotics.robotnik.devices.FalconEncoder;
import com._604robotics.robotnik.devices.QuixCANCoder;
import com._604robotics.robotnik.motorcontrol.Motor;
import com._604robotics.robotnik.motorcontrol.QuixTalonFX;
import com._604robotics.robotnik.motorcontrol.controllers.MotorControllerPIDConfig;
import com._604robotics.robotnik.motorcontrol.controllers.TalonPID;
import com._604robotics.robotnik.motorcontrol.gearing.CalculableRatio;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class QuixFalconSwerveModule extends QuixSwerveModule{

    public QuixFalconSwerveModule(
        String name,
        int id,
        SubsystemBase subsystem,
        Translation2d position,
        QuixTalonFX driveMotor,
        QuixTalonFX steeringMotor,
        FalconEncoder driveEncoder,
        FalconEncoder steeringEncoder,
        QuixCANCoder absSteeringEncoder,
        TalonPID drivePID,
        TalonPID steeringPID,
        SimpleMotorFeedforward driveFeedforward,
        CalculableRatio driveRatio,
        CalculableRatio steeringRatio,
        double angleOffset,
        double wheelDiameter,
        double maxDriveVelocity
    ) {
        super(name, id, subsystem, position, driveMotor, steeringMotor, absSteeringEncoder, driveEncoder, steeringEncoder, drivePID, steeringPID, driveFeedforward, driveRatio, steeringRatio, angleOffset, wheelDiameter, maxDriveVelocity);

        driveMotor.getController().configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        steeringMotor.getController().configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

        driveEncoder.setInverted(driveMotor.getInverted());
    }


    public static QuixFalconSwerveModule createModule(
        String name,
        int id,
        SubsystemBase subsystem,
        Translation2d position,
        int driveMotorPort,
        int steeringMotorPort,
        int absSteeringEncoderPort,
        boolean invertDriveMotor,
        boolean invertSteeringMotor,
        MotorControllerPIDConfig drivePIDConfig,
        MotorControllerPIDConfig steeringPIDConfig,
        SimpleMotorFeedforward driveFeedforward,
        CalculableRatio driveRatio,
        CalculableRatio steeringRatio,
        double angleOffset,
        double wheelDiameter,
        double maxDriveVelocity
    ) {
        QuixTalonFX driveMotor = new QuixTalonFX(driveMotorPort, name + " Drive Motor", Motor.kFalcon500, subsystem);
        QuixTalonFX steeringMotor = new QuixTalonFX(steeringMotorPort, name + " Steering Motor", Motor.kFalcon500, subsystem);

        QuixCANCoder absSteeringEncoder = new QuixCANCoder(absSteeringEncoderPort, name + " Absolute Steering Encoder");

        driveMotor.setInverted(invertDriveMotor);
        steeringMotor.setInverted(invertSteeringMotor);

        driveMotor.getController().setNeutralMode(NeutralMode.Brake);
        steeringMotor.getController().setNeutralMode(NeutralMode.Coast);

        FalconEncoder driveEncoder = new FalconEncoder(driveMotor);
        FalconEncoder steeringEncoder = new FalconEncoder(steeringMotor);

        TalonPID drivePID = new TalonPID(driveMotor, driveEncoder, drivePIDConfig);
        TalonPID steeringPID = new TalonPID(steeringMotor, steeringEncoder, steeringPIDConfig);

        return new QuixFalconSwerveModule(name, id, subsystem, position, driveMotor, steeringMotor, driveEncoder, steeringEncoder, absSteeringEncoder, drivePID, steeringPID, driveFeedforward, driveRatio, steeringRatio, angleOffset, wheelDiameter, maxDriveVelocity);
    }
}
