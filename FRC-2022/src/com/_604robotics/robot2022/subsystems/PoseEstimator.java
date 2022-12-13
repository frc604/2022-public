package com._604robotics.robot2022.subsystems;

import com._604robotics.robot2022.Calibration;
import com._604robotics.robotnik.math.MathUtils;
import com._604robotics.robotnik.swerve.QuixSwerveDriveKinematics;
import com._604robotics.robotnik.swerve.QuixSwerveModuleState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.Timer;

public class PoseEstimator {
    private SwerveDrivePoseEstimator estimator;
    private TimeInterpolatableBuffer<Double> gryoBuffer = TimeInterpolatableBuffer.createDoubleBuffer(1);

    public PoseEstimator(QuixSwerveDriveKinematics kinematics) {
        estimator = new SwerveDrivePoseEstimator(
            new Rotation2d(),
            new Pose2d(0, 0, new Rotation2d()),
            kinematics.getInternalSwerverDriveKinematics(),
            Calibration.PoseEstimator.stateStdDevs,
            Calibration.PoseEstimator.localMeasurementStdDevs,
            Calibration.PoseEstimator.visionMeasurementStdDevs
        );
    }

    public boolean readyToUpdateVision(){
        return gryoBuffer.getSample(0) != null;
    }

    public void updateSwerve(Rotation2d gyroAngle, QuixSwerveModuleState[] states){
        estimator.update(gyroAngle, QuixSwerveDriveKinematics.quixSwerveModuleStatesToSwerveModuleStates(states));
        gryoBuffer.addSample(Timer.getFPGATimestamp(), gyroAngle.getRadians());
    }

    public Pose2d updateVision(Rotation2d bearing, Rotation2d elevation, double latency){
        double timeStamp = Timer.getFPGATimestamp() - (latency / 1000);

        Pose2d fieldToRobot = PoseEstimator.computeFieldToRobotFromVisionBE(
            bearing.unaryMinus(), elevation, new Rotation2d(gryoBuffer.getSample(timeStamp)));

        estimator.addVisionMeasurement(
            fieldToRobot,
            timeStamp
        );

        return fieldToRobot;
    }

    public Pose2d getPose(){
        return estimator.getEstimatedPosition();
    }

    public static Pose2d computeFieldToRobotFromVisionBE(
            Rotation2d camBearing,
            Rotation2d camElevation,
            Rotation2d gyroAngle
        ) {
        double cameraHeight = MathUtils.extractTFMatrixZ(Calibration.PoseEstimator.robotToCamera);
        double heightDelta = Calibration.PoseEstimator.goalHeight - cameraHeight;

        // Bearing/elevation from camera are in "pinhole" coordinates, not spherical.
        // https://docs.limelightvision.io/en/latest/theory.html#from-pixels-to-angles
        // To deal with this, first we convert bearing/elevation to a point in horizontal
        // camera frame. Note that the magnitude of the vector to this point is arbitrary,
        // but the direction is correct.
        Matrix<N3, N1> camFramePoint = MathUtils.pinholeBEtoPoint(camBearing, camElevation);
        Matrix<N3, N1> horizontalCamFramePoint = MathUtils.getPointInFrame(camFramePoint, Calibration.PoseEstimator.cameraToHorizontalCamera);

        // Next we use this point to compute bearing and elevation.
        Matrix<N2, N1> sphericalBE = MathUtils.cart2BESph(horizontalCamFramePoint);
        Rotation2d bearing = new Rotation2d(sphericalBE.get(0, 0));
        Rotation2d elevation = new Rotation2d(sphericalBE.get(1, 0));

        // Use elevation to compute distance to goal in horizontal camera frame.
        double distanceFromGoal = heightDelta / elevation.getTan();
        distanceFromGoal += Calibration.PoseEstimator.goalRadius;
        Translation2d horizontalCameraToGoalTranslation = new Translation2d(distanceFromGoal, 0);
        Translation2d rotatedGoalToHorizontalCameraTranslation = horizontalCameraToGoalTranslation.rotateBy(gyroAngle.plus(bearing)).unaryMinus();
        Matrix<N4, N4> horizontalCameraToGoal = MathUtils.makeZRotTFMatrix(rotatedGoalToHorizontalCameraTranslation.getX(), rotatedGoalToHorizontalCameraTranslation.getY(), heightDelta, gyroAngle.getRadians());
        Matrix<N4, N4> fieldToHorizontalCamera = Calibration.PoseEstimator.fieldToGoal.times(horizontalCameraToGoal);
        Matrix<N4, N4> fieldToRobot = fieldToHorizontalCamera.times(Calibration.PoseEstimator.horizontalCameraToRobot);

        // Turn this into a Pose2d
        return MathUtils.zRotTFMatrixToPose(fieldToRobot);
    }
}
