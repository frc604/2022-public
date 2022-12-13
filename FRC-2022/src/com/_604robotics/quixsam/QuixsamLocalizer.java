// package com._604robotics.quixsam;

// import java.util.HashMap;
// import java.util.TreeMap;

// import com._604robotics.quixsam.mathematics.DoubleInterpolatableTreeMap;
// import com._604robotics.quixsam.mathematics.Interpolatable;
// import com._604robotics.quixsam.odometry.DiffDriveOdometryMeasurement;
// import com._604robotics.robotnik.vision.VisionCamera;
// import com._604robotics.robotnik.vision.VisionCamera.Target;

// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
// import edu.wpi.first.wpiutil.math.Pair;

// public class QuixsamLocalizer {
//     private int currentID = 0;
//     private HashMap<Integer, Double> idMap;

//     private TreeMap<Double, DiffDriveOdometryMeasurement> odometryMap;
//     private DoubleInterpolatableTreeMap<Pose2d> poseMap = new DoubleInterpolatableTreeMap<>();

//     private DifferentialDriveOdometry rawOdometry;
//     private DifferentialDriveOdometry playbackOdometry;

//     private QuixsamNetworkTable networkTable;

//     public QuixsamLocalizer(QuixsamNetworkTable table, Pose2d initialPoseMeters, Rotation2d initialGyroAngle) {
//         rawOdometry = new DifferentialDriveOdometry(initialGyroAngle, initialPoseMeters);
//         playbackOdometry = new DifferentialDriveOdometry(initialGyroAngle, initialPoseMeters);
//     }

//     public void update(DiffDriveOdometryMeasurement odometry, VisionCamera.PipelineVisionPacket vision) {
//         currentID += 1;
//         double currentTime = Timer.getFPGATimestamp();

//         odometry.addTo(rawOdometry);
//         odometry.addTo(playbackOdometry);
//         odometryMap.put(currentTime, odometry);

//         poseMap.set(currentTime, Interpolatable.interPose2d(rawOdometry.getPoseMeters()));

//         double visionTime = currentTime - (vision.getLatency() / 1000);
//         Pose2d interpolatedPose = poseMap.get(visionTime);

//         idMap.put(currentID, visionTime);

//         Target bestTarget = vision.getBestTarget();
//         if (!bestTarget.getCorners().isEmpty()) {
//             for (Pair<Double, Double> corner : bestTarget.getCorners()) {
//                 networkTable.publishVision(currentID, corner.getFirst(), corner.getSecond(), 0.1, 0.1);
//             }
//         }

//         currentID += 1;
//         idMap.put(currentID, currentTime);
//         networkTable.publishOdometry(currentID, rawOdometry.getPoseMeters(), new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1)));
//     }

//     public void update(DiffDriveOdometryMeasurement odometry) {
//         currentID += 1;
//         double currentTime = Timer.getFPGATimestamp();

//         odometry.addTo(rawOdometry);
//         odometry.addTo(playbackOdometry);
//         odometryMap.put(currentTime, odometry);

//         poseMap.set(currentTime, Interpolatable.interPose2d(rawOdometry.getPoseMeters()));

//         idMap.put(currentID, currentTime);
//         networkTable.publishOdometry(currentID, rawOdometry.getPoseMeters(), new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1)));
//     }

//     public void computeEstimate(QuixsamEsimate estimate) {
//         int estimateID = estimate.getID();
//         double estimateTime = idMap.get(estimateID);

//         playbackOdometry.resetPosition(estimate.getPose());

//         Double lastKey = odometryMap.ceilingKey(estimateTime); //least key >= time

//         while (lastKey != null) {
//             DiffDriveOdometryMeasurement lastMeasurment = odometryMap.get(lastKey);
//             playbackOdometry.update(lastMeasurment.getGyroAngle(), lastMeasurment.getLeftDistanceMeters(), lastMeasurment.getRightDistanceMeters());
//             odometryMap.remove(lastKey);
//             lastKey = odometryMap.ceilingKey(lastKey);
//         }
//     }

//     public Pose2d getPose() {
//         return playbackOdometry.getPoseMeters();
//     }
// }
