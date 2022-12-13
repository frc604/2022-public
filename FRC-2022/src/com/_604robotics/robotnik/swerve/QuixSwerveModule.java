package com._604robotics.robotnik.swerve;

import com._604robotics.robotnik.devices.AbsoluteEncoder;
import com._604robotics.robotnik.devices.IntegratedEncoder;
import com._604robotics.robotnik.motorcontrol.QuixMotorController;
import com._604robotics.robotnik.motorcontrol.controllers.MotorControllerPID;
import com._604robotics.robotnik.motorcontrol.gearing.CalculableRatio;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class QuixSwerveModule {
    protected String name;
    protected int id;
    protected SubsystemBase subsystem;
    protected Translation2d position;
    protected QuixMotorController driveMotor;
    protected QuixMotorController steeringMotor;
    protected AbsoluteEncoder absSteeringEncoder;
    protected IntegratedEncoder driveEncoder;
    protected IntegratedEncoder steeringEncoder;
    protected MotorControllerPID drivePID;
    protected MotorControllerPID steeringPID;
    protected SimpleMotorFeedforward driveFeedforward;
    protected CalculableRatio driveRatio;
    protected CalculableRatio steeringRatio;

    protected double angleOffset;
    protected double wheelDiameter;
    protected double maxDriveVelocity;

    private double lastAngle;
    // Position in converted encoder units that corresponds to a zero module steering angle.
    private double steeringEncoderZeroPosition = 0.0;


    public QuixSwerveModule(
        String name,
        int id,
        SubsystemBase subsystem,
        Translation2d position,
        QuixMotorController driveMotor,
        QuixMotorController steeringMotor,
        AbsoluteEncoder absSteeringEncoder,
        IntegratedEncoder driveEncoder,
        IntegratedEncoder steeringEncoder,
        MotorControllerPID drivePID,
        MotorControllerPID steeringPID,
        SimpleMotorFeedforward driveFeedforward,
        CalculableRatio driveRatio,
        CalculableRatio steeringRatio,
        double angleOffset,
        double wheelDiameter,
        double maxDriveVelocity
    ) {
        this.name = name;
        this.id = id;
        this.subsystem = subsystem;
        this.position = position;
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.absSteeringEncoder = absSteeringEncoder;
        this.driveEncoder = driveEncoder;
        this.steeringEncoder = steeringEncoder;
        this.drivePID = drivePID;
        this.steeringPID = steeringPID;
        this.driveFeedforward = driveFeedforward;
        this.driveRatio = driveRatio;
        this.steeringRatio = steeringRatio;
        this.angleOffset = angleOffset;
        this.wheelDiameter = wheelDiameter;
        this.maxDriveVelocity = maxDriveVelocity;

        this.driveMotor.setCurrentLimit(40);
        this.driveMotor.enableCurrentLimit(true);

        this.steeringMotor.setCurrentLimit(30);
        this.steeringMotor.enableCurrentLimit(true);

        this.driveEncoder.setdistancePerRotation(this.driveRatio.calculate(Math.PI * wheelDiameter));
        this.steeringEncoder.setdistancePerRotation(this.steeringRatio.calculate(360.0));

        zeroToAbsPosition();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredStateClosedLoop(QuixSwerveModuleState desiredState){
        desiredState = QuixSwerveModuleState.optimize(desiredState, getState().angle);

        drivePID.setSetpointVelocity(desiredState.speedMetersPerSecond, driveFeedforward.calculate(desiredState.speedMetersPerSecond));

        // SmartDashboard.putNumber("Module " + this.id + " Velocity Setpoint:", desiredState.speedMetersPerSecond);
        // SmartDashboard.putNumber("Module " + this.id + " Velocity Current:", this.getState().speedMetersPerSecond);

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= maxDriveVelocity * 0.01) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        steeringPID.setSetpointPosition(angle + steeringEncoderZeroPosition);
        lastAngle = angle;
    }

    public void setDesiredStateOpenLoop(QuixSwerveModuleState desiredState){
        desiredState = QuixSwerveModuleState.optimize(desiredState, getState().angle);

        double percentOutput = desiredState.speedMetersPerSecond / maxDriveVelocity;
        driveMotor.set(percentOutput);

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= maxDriveVelocity * 0.01) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        steeringPID.setSetpointPosition(angle + steeringEncoderZeroPosition);
        lastAngle = angle;
    }

    public int getID() {
        return id;
    }

    public Translation2d getPosition() {
        return position;
    }

    public double getAbsEncoderAngle() {
        return absSteeringEncoder.getAbsPosition();
    }

    public void zeroToAbsPosition() {
        double absAngle = absSteeringEncoder.getAbsPosition() - angleOffset;
        steeringEncoderZeroPosition = steeringEncoder.getPosition() - absAngle;
    }

    public double getAngle() {
        return this.steeringEncoder.getPosition() - steeringEncoderZeroPosition;
    }

    public QuixSwerveModuleState getState(){
        double velocity = this.driveEncoder.getVelocity();
        Rotation2d angle = Rotation2d.fromDegrees(getAngle());
        return new QuixSwerveModuleState(velocity, angle);
    }

    // Returns the velocity vector in the module frame.
    public Matrix<N2, N1> getVelocityVector() {
        var state = getState();
        double xVel = state.speedMetersPerSecond * Math.cos(state.angle.getRadians());
        double yVel = state.speedMetersPerSecond * Math.sin(state.angle.getRadians());
        return Matrix.mat(Nat.N2(), Nat.N1()).fill(xVel, yVel);
    }
}
