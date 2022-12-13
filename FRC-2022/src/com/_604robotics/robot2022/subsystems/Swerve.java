package com._604robotics.robot2022.subsystems;
import com._604robotics.quixsam.QuixsamSwerveLocalizer;
import com._604robotics.quixsam.odometry.SwerveDriveOdometryMeasurement;
import com._604robotics.robot2022.Calibration;
import com._604robotics.robot2022.Constants;
import com._604robotics.robotnik.math.MathUtils;
import com._604robotics.robotnik.swerve.QuixFalconSwerveModule;
import com._604robotics.robotnik.swerve.QuixSwerveDriveKinematics;
import com._604robotics.robotnik.swerve.QuixSwerveDriveOdometry;
import com._604robotics.robotnik.swerve.QuixSwerveModule;
import com._604robotics.robotnik.swerve.QuixSwerveModuleState;
import com._604robotics.robotnik.vision.Limelight;
import com._604robotics.robotnik.vision.VisionCamera.PipelineVisionPacket;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private final QuixFalconSwerveModule frontLeft = QuixFalconSwerveModule.createModule(
        "Front Left",
        0,
        this,
        Calibration.Swerve.FrontLeft.modulePosition,
        Constants.Swerve.FrontLeft.driveMotorPort,
        Constants.Swerve.FrontLeft.steeringMotorPort,
        Constants.Swerve.FrontLeft.canCoderPort,
        Calibration.Swerve.driveMotorInvert,
        Calibration.Swerve.steeringMotorInvert,
        Calibration.Swerve.drivePIDConfig,
        Calibration.Swerve.steeringPIDConfig,
        Calibration.Swerve.driveFeedforward,
        Calibration.Swerve.driveGearRatio,
        Calibration.Swerve.steeringGearRatio,
        Calibration.Swerve.FrontLeft.angleOffset,
        Calibration.Swerve.wheelDiameter,
        Calibration.Swerve.maxSpeed
    );

    private final QuixFalconSwerveModule frontRight = QuixFalconSwerveModule.createModule(
        "Front Right",
        1,
        this,
        Calibration.Swerve.FrontRight.modulePosition,
        Constants.Swerve.FrontRight.driveMotorPort,
        Constants.Swerve.FrontRight.steeringMotorPort,
        Constants.Swerve.FrontRight.canCoderPort,
        Calibration.Swerve.driveMotorInvert,
        Calibration.Swerve.steeringMotorInvert,
        Calibration.Swerve.drivePIDConfig,
        Calibration.Swerve.steeringPIDConfig,
        Calibration.Swerve.driveFeedforward,
        Calibration.Swerve.driveGearRatio,
        Calibration.Swerve.steeringGearRatio,
        Calibration.Swerve.FrontRight.angleOffset,
        Calibration.Swerve.wheelDiameter,
        Calibration.Swerve.maxSpeed
    );

    private final QuixFalconSwerveModule rearRight = QuixFalconSwerveModule.createModule(
        "Rear Right",
        2,
        this,
        Calibration.Swerve.RearRight.modulePosition,
        Constants.Swerve.RearRight.driveMotorPort,
        Constants.Swerve.RearRight.steeringMotorPort,
        Constants.Swerve.RearRight.canCoderPort,
        Calibration.Swerve.driveMotorInvert,
        Calibration.Swerve.steeringMotorInvert,
        Calibration.Swerve.drivePIDConfig,
        Calibration.Swerve.steeringPIDConfig,
        Calibration.Swerve.driveFeedforward,
        Calibration.Swerve.driveGearRatio,
        Calibration.Swerve.steeringGearRatio,
        Calibration.Swerve.RearRight.angleOffset,
        Calibration.Swerve.wheelDiameter,
        Calibration.Swerve.maxSpeed
    );

    private final QuixFalconSwerveModule rearLeft = QuixFalconSwerveModule.createModule(
        "Rear Left",
        3,
        this,
        Calibration.Swerve.RearLeft.modulePosition,
        Constants.Swerve.RearLeft.driveMotorPort,
        Constants.Swerve.RearLeft.steeringMotorPort,
        Constants.Swerve.RearLeft.canCoderPort,
        Calibration.Swerve.driveMotorInvert,
        Calibration.Swerve.steeringMotorInvert,
        Calibration.Swerve.drivePIDConfig,
        Calibration.Swerve.steeringPIDConfig,
        Calibration.Swerve.driveFeedforward,
        Calibration.Swerve.driveGearRatio,
        Calibration.Swerve.steeringGearRatio,
        Calibration.Swerve.RearLeft.angleOffset,
        Calibration.Swerve.wheelDiameter,
        Calibration.Swerve.maxSpeed
    );

    private final QuixSwerveModule[] modules = {
        frontLeft,
        frontRight,
        rearRight,
        rearLeft
    };

    /* Kinematics */
    public QuixSwerveDriveKinematics kinematics = new QuixSwerveDriveKinematics(
        frontLeft.getPosition(),
        frontRight.getPosition(),
        rearRight.getPosition(),
        rearLeft.getPosition()
    );

    /* Gyro */
    private final PigeonIMU imu = new PigeonIMU(Constants.Swerve.pigeonID);

    private double continousAngleOffset = 0.0;

    /* Odometry */
    public final QuixSwerveDriveOdometry odometry = new QuixSwerveDriveOdometry(
        kinematics,
        getContinousYaw(),
        new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(0))
    );

    // public QuixsamSwerveLocalizer quixsam = new QuixsamSwerveLocalizer(
    //     "quixsam",
    //     kinematics,
    //     new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(0)),
    //     new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(0)),
    //     getHeading()
    // );

    public final QuixsamSwerveLocalizer localizer = new QuixsamSwerveLocalizer(
        "quixsam",
        kinematics.getInternalSwerverDriveKinematics(),
        new Pose2d(),
        new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.01)),
        getContinousYaw()
      );

    private final Limelight limelight = new Limelight("Limelight", new Vector3D(0.0, 0.0, 0.0), 0.0);
    private Field2d fieldSim = new Field2d();

    private double orbitTangentialVelocity = Calibration.Launcher.sotfConstantTangentialVelocity;

    public Swerve() {
        SmartDashboard.putData("Field", fieldSim);
        imu.configFactoryDefault();
        zeroGyro();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        QuixSwerveModuleState[] desiredStates =
            kinematics.toQuixSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(),
                                    translation.getY(),
                                    rotation,
                                    getContinousYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(),
                                    translation.getY(),
                                    rotation)
                                );
        QuixSwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Calibration.Swerve.maxSpeed);

        for(QuixSwerveModule module : modules){
            module.setDesiredStateOpenLoop(desiredStates[module.getID()]);
        }
    }

    public void driveClosedLoop(Translation2d translation, double rotation, boolean fieldRelative) {
        QuixSwerveModuleState[] desiredStates =
            kinematics.toQuixSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(),
                                    translation.getY(),
                                    rotation,
                                    getContinousYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(),
                                    translation.getY(),
                                    rotation)
                                );
        QuixSwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Calibration.Swerve.maxSpeed);

        for(QuixSwerveModule module : modules){
            module.setDesiredStateClosedLoop(desiredStates[module.getID()]);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(QuixSwerveModuleState[] desiredStates) {
        QuixSwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Calibration.Swerve.maxSpeed);

        for(QuixSwerveModule module : modules){
            module.setDesiredStateClosedLoop(desiredStates[module.getID()]);
        }
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, getContinousYaw());
    }

    public void zeroGyro(){
        zeroGyro(new Rotation2d());
    }

    public void zeroGyro(Rotation2d value){
        imu.setYaw(value.getDegrees());
    }

    public void zeroModuleEncoders() {
        for(QuixSwerveModule module : modules){
            module.zeroToAbsPosition();
        }
    }

    public void zeroContinousYaw() {
        zeroContinousYaw(0.0);
    }

    public void zeroContinousYaw(double value) {
        continousAngleOffset = imu.getYaw() - value;
    }

    public double getRobotYTilt() {
        // Axis is swapped and backwards.
        // +Y tilt is tilting forwards.
        return -imu.getRoll();
    }

    public double getRobotYTiltRate() {
        double[] xyz_dps = new double[3];
        imu.getRawGyro(xyz_dps);
        return -xyz_dps[1];
    }

    public QuixSwerveModuleState[] getStates(){
        QuixSwerveModuleState[] states = new QuixSwerveModuleState[4];
        for(QuixSwerveModule module : modules){
            states[module.getID()] = module.getState();
        }
        return states;
    }

    // Returns the velocity vector in the robot frame.
    public Matrix<N2, N1> getVelocityVector() {
        var chassisSpeeds = kinematics.toChassisSpeeds(getStates());
        return Matrix.mat(Nat.N2(), Nat.N1()).fill(
            chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    // Returns the velocity vector in the field-frame.
    public Matrix<N2, N1> getFieldRelativeVelocityVector() {
        return MathUtils.rotateVector(
            getVelocityVector(),
            getEstimatedPose().getRotation().getRadians());
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        imu.getYawPitchRoll(ypr);

        return (Calibration.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    public Rotation2d getContinousYaw() {
        double continousYaw = imu.getYaw();

        return (Calibration.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - (continousYaw - continousAngleOffset)) : Rotation2d.fromDegrees(continousYaw - continousAngleOffset);
    }

    public QuixSwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Pose2d getEstimatedPose() {
        return localizer.getPose();
    }

    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    public void resetLocalizerPose(Pose2d pose) {
        localizer.reset(pose, getContinousYaw());
    }

    public double getDistanceToGoal() {
        return Math.sqrt(Math.pow(localizer.getPose().getX(), 2) + Math.pow(localizer.getPose().getY(), 2));
    }

    public Rotation2d getAngleAroundGoal() {
        Pose2d estimatedPose = getEstimatedPose();
        return new Rotation2d((Math.atan2(estimatedPose.getY(), estimatedPose.getX()) + 2*Math.PI) % (2*Math.PI)); // From 0 to 2pi
    }

    public double getCurrentAngularVel() {
        return kinematics.toChassisSpeeds(getStates()).omegaRadiansPerSecond;
    }

    public double getOrbitTangentialVelocity() {
        return orbitTangentialVelocity;
    }

    public void setOrbitTangentialVelocity(double orbitTangentialVelocity) {
        this.orbitTangentialVelocity = orbitTangentialVelocity;
    }

    public void quixsamPeriodic() {
        PipelineVisionPacket latestMeasurement = limelight.getLatestMeasurement();

        SwerveDriveOdometryMeasurement odometryMeasurement = new SwerveDriveOdometryMeasurement(
            getContinousYaw(),
            0.1, 0.1, new Rotation2d(0.1),
            QuixSwerveDriveKinematics.quixSwerveModuleStateToSwerveModuleState(frontLeft.getState()),
            QuixSwerveDriveKinematics.quixSwerveModuleStateToSwerveModuleState(frontRight.getState()),
            QuixSwerveDriveKinematics.quixSwerveModuleStateToSwerveModuleState(rearLeft.getState()),
            QuixSwerveDriveKinematics.quixSwerveModuleStateToSwerveModuleState(rearRight.getState())
        );

        if (latestMeasurement.hasTargets()) {
            // System.out.println("Target Yaw: " + packet.getBestTarget().getYaw() + ", Target Pitch: " + packet.getBestTarget().getPitch());
            localizer.update(odometryMeasurement, latestMeasurement);
        } else {
            localizer.update(odometryMeasurement);
        }

        localizer.periodic();

        fieldSim.getObject("Odometry").setPose(convertPoseToCenterOfField(odometry.getPoseMeters()));
        fieldSim.getObject("Filtered").setPose(convertPoseToCenterOfField(localizer.getPose()));
    }

    @Override
    public void periodic(){
        odometry.update(getContinousYaw(), getStates());

        SmartDashboard.putNumber("Pigeon", getYaw().getDegrees());
        SmartDashboard.putNumber("Pigeon Continous", getContinousYaw().getDegrees());
    }


    public static Pose2d convertPoseToCenterOfField(Pose2d pose) {
        return new Pose2d(
           pose.getTranslation().plus(new Translation2d(Units.inchesToMeters(649 / 2.0), Units.inchesToMeters(319 / 2.0))),
           pose.getRotation()
         );
    }
}