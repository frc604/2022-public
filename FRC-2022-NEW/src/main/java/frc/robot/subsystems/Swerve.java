// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.quixlib.devices.QuixIMU;
import frc.quixlib.math.MathUtils;
import frc.quixlib.swerve.QuixSwerve;
import frc.quixlib.swerve.QuixSwerveModule;
import frc.quixlib.swerve.QuixSwerveModuleFactory;
import frc.quixlib.vision.Fiducial;
import frc.quixlib.vision.QuixVisionCamera;
import frc.robot.Constants;

public class Swerve extends QuixSwerve {
  private final QuixVisionCamera m_camera;

  public Swerve(QuixIMU imu, QuixVisionCamera camera, Field2d fieldViz) {
    super(
        imu,
        camera,
        Constants.Swerve.maxDriveSpeed,
        Constants.Swerve.maxModuleAcceleration,
        Constants.Swerve.maxModuleSteeringRate,
        Constants.Swerve.driveController,
        Constants.aprilTags,
        fieldViz);
    m_camera = camera;
  }

  protected QuixSwerveModule[] createModules() {
    QuixSwerveModuleFactory swerveModuleFactory =
        new QuixSwerveModuleFactory(
            Constants.Swerve.wheelCircumference,
            Constants.Swerve.driveRatio,
            Constants.Swerve.steeringRatio,
            Constants.Swerve.drivePIDConfig,
            Constants.Swerve.driveFeedforward,
            Constants.Swerve.steeringPIDConfig);
    QuixSwerveModule[] modules = {
      swerveModuleFactory.createModule(
          Constants.Swerve.FrontLeft.modulePosition,
          Constants.Swerve.FrontLeft.driveMotorID,
          Constants.Swerve.FrontLeft.steeringMotorID,
          Constants.Swerve.FrontLeft.canCoderID,
          Constants.Swerve.FrontLeft.absEncoderOffsetRad),
      swerveModuleFactory.createModule(
          Constants.Swerve.FrontRight.modulePosition,
          Constants.Swerve.FrontRight.driveMotorID,
          Constants.Swerve.FrontRight.steeringMotorID,
          Constants.Swerve.FrontRight.canCoderID,
          Constants.Swerve.FrontRight.absEncoderOffsetRad),
      swerveModuleFactory.createModule(
          Constants.Swerve.RearRight.modulePosition,
          Constants.Swerve.RearRight.driveMotorID,
          Constants.Swerve.RearRight.steeringMotorID,
          Constants.Swerve.RearRight.canCoderID,
          Constants.Swerve.RearRight.absEncoderOffsetRad),
      swerveModuleFactory.createModule(
          Constants.Swerve.RearLeft.modulePosition,
          Constants.Swerve.RearLeft.driveMotorID,
          Constants.Swerve.RearLeft.steeringMotorID,
          Constants.Swerve.RearLeft.canCoderID,
          Constants.Swerve.RearLeft.absEncoderOffsetRad)
    };
    return modules;
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  public void simulateVision(Pose2d simPose) {
    // Update camera based on sim pose.
    final double goalHeight = 2.60; // m
    final double goalDiameter = 1.36; // m
    final double goalRadius = goalDiameter * 0.5;

    final Matrix<N4, N4> fieldToRobot =
        MathUtils.makeZRotTFMatrix(
            simPose.getX(), simPose.getY(), 0.0, simPose.getRotation().getRadians());
    final Matrix<N4, N4> fieldToCamera = fieldToRobot.times(Constants.Transforms.robotToCamera);

    // Simulate vision target by transforming the goal center by the radius towards the camera.
    Translation2d targetTranslation = new Translation2d(goalRadius, 0.0);
    Rotation2d targetRotation =
        new Rotation2d(
            MathUtils.zRotTFMatrixToPose(fieldToCamera).getX(),
            MathUtils.zRotTFMatrixToPose(fieldToCamera).getY());
    targetTranslation = targetTranslation.rotateBy(targetRotation);

    // Copy the set of AprilTags and add the vision target to it.
    Fiducial[] fiducials = new Fiducial[Constants.aprilTags.length + 1];
    for (int i = 0; i < Constants.aprilTags.length; i++) {
      fiducials[i] = Constants.aprilTags[i];
    }
    fiducials[Constants.aprilTags.length] =
        new Fiducial(targetTranslation.getX(), targetTranslation.getY(), goalHeight);

    m_camera.updateSim(fieldToCamera, fiducials);
  }
  // --- END STUFF FOR SIMULATION ---
}
