package com._604robotics.quixsam;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentSkipListMap;

import com._604robotics.quixsam.odometry.QuixsamSwerveDriveOdometry;
import com._604robotics.quixsam.odometry.SendableOdometryMeasurment;
import com._604robotics.quixsam.odometry.SwerveDriveOdometryMeasurement;
import com._604robotics.quixsam.vision.SendableVisionMeasurment;
import com._604robotics.robotnik.math.MathUtils;
import com._604robotics.robotnik.vision.VisionCamera;
import com._604robotics.robotnik.vision.VisionCamera.Target;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

public class QuixsamSwerveLocalizer {
    private int currentID = 0;
    private HashMap<Integer, Double> idMap = new HashMap<>();

    private ConcurrentSkipListMap<Double, SwerveDriveOdometryMeasurement> odometryMap = new ConcurrentSkipListMap<>();
    private TimeInterpolatableBuffer<Pose2d> poseMap = TimeInterpolatableBuffer.createBuffer(10);

    private TreeMap<Double, Pair<SendableOdometryMeasurment, SendableVisionMeasurment>> buffer = new TreeMap<>();

    private QuixsamSwerveDriveOdometry rawOdometry;
    private QuixsamSwerveDriveOdometry playbackOdometry;

    private QuixsamNetworkTable networkTable;

    private QuixsamEsimate currentEstimate = new QuixsamEsimate(0, new Pose2d());

    private double timeTreshold = 0.2; // seconds

    public QuixsamSwerveLocalizer(String name, SwerveDriveKinematics kinematics, Pose2d priori, Pose2d prioriSigma, Rotation2d initialGyroAngle) {
        rawOdometry = new QuixsamSwerveDriveOdometry(kinematics, initialGyroAngle, priori);
        playbackOdometry = new QuixsamSwerveDriveOdometry(kinematics, initialGyroAngle, priori);

        networkTable = new QuixsamNetworkTable(name, priori, prioriSigma, this::computeEstimate);
    }

    public void update(SwerveDriveOdometryMeasurement odometry, VisionCamera.PipelineVisionPacket vision) {
        double currentTime = Timer.getFPGATimestamp();

        rawOdometry.updateWithTime(currentTime, odometry.getGyroAngle(), odometry.getModuleStates());
        playbackOdometry.updateWithTime(currentTime, odometry.getGyroAngle(), odometry.getModuleStates());
        odometryMap.put(currentTime, odometry);

        poseMap.addSample(currentTime, rawOdometry.getPoseMeters());

        double visionTime = currentTime - (vision.getLatency() / 1000);
        Pose2d interpolatedPose = poseMap.getSample(visionTime);

        // System.out.println("Interp Pose: " + interpolatedPose);
        // System.out.println("REAL Pose: " + rawOdometry.getPoseMeters());

        SendableOdometryMeasurment sendableOdometryMeasurment;
        SendableVisionMeasurment sendableVisionMeasurment = null;

        Target bestTarget = vision.getBestTarget();
        if (vision.hasTargets()) {
            sendableVisionMeasurment = new SendableVisionMeasurment(0);

            Matrix<N3, N1> camFramePoint = MathUtils.pinholeBEtoPoint(Rotation2d.fromDegrees(bestTarget.getYaw()), Rotation2d.fromDegrees(bestTarget.getPitch()));
            Matrix<N2, N1> sphericalBE = MathUtils.cart2BESph(camFramePoint);

            sendableVisionMeasurment.addMeasurment(new Pair<>(sphericalBE.get(0, 0), sphericalBE.get(1, 0)), new Pair<>(0.1, 0.1));

            sendableOdometryMeasurment = new SendableOdometryMeasurment(0, interpolatedPose, new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1)));
            buffer.put(visionTime, new Pair<>(sendableOdometryMeasurment, sendableVisionMeasurment));
        } else {
            sendableOdometryMeasurment = new SendableOdometryMeasurment(0, rawOdometry.getPoseMeters(), new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1)));
            buffer.put(currentTime, new Pair<>(sendableOdometryMeasurment, sendableVisionMeasurment));
        }
    }

    public void update(SwerveDriveOdometryMeasurement odometry) {
        double currentTime = Timer.getFPGATimestamp();

        rawOdometry.updateWithTime(currentTime, odometry.getGyroAngle(), odometry.getModuleStates());
        playbackOdometry.updateWithTime(currentTime, odometry.getGyroAngle(), odometry.getModuleStates());
        odometryMap.put(currentTime, odometry);

        poseMap.addSample(currentTime, rawOdometry.getPoseMeters());

        buffer.put(currentTime, new Pair<>(new SendableOdometryMeasurment(0, rawOdometry.getPoseMeters(), new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1))), null));
    }

    public void computeEstimate(QuixsamEsimate estimate) {
        currentEstimate = estimate;

        int estimateID = estimate.getID();
        double estimateTime = idMap.get(estimateID);

        TreeMap<Double, SwerveDriveOdometryMeasurement> tempOdometryMap = new TreeMap<>(odometryMap);

        Double lastKey = tempOdometryMap.ceilingKey(estimateTime); //least key >= time

        playbackOdometry.resetPosition(estimate.getPose(), tempOdometryMap.get(lastKey).getGyroAngle());

        Double prevTime = tempOdometryMap.lowerKey(lastKey);
        if (prevTime != null) {
            while (lastKey != null) {
                SwerveDriveOdometryMeasurement lastMeasurment = tempOdometryMap.get(lastKey);
                playbackOdometry.updateWithTime(prevTime, lastKey, lastMeasurment.getGyroAngle(), lastMeasurment.getModuleStates());
                prevTime = lastKey;
                tempOdometryMap.remove(lastKey);
                lastKey = tempOdometryMap.ceilingKey(lastKey);
            }
        }
    }

    public void reset(Pose2d pose, Rotation2d gyroAngle) {
        rawOdometry.resetPosition(pose, gyroAngle);
        playbackOdometry.resetPosition(pose, gyroAngle);
    }

    public QuixsamEsimate getEstimate() {
        return currentEstimate;
    }

    public Pose2d getPose() {
        return playbackOdometry.getPoseMeters();
    }

    public void periodic() {
        double currentTime = Timer.getFPGATimestamp();

        ArrayList<Double> keys = new ArrayList<>(buffer.keySet());
        for (double key : keys) {
            if (currentTime - key > timeTreshold) {
                currentID += 1;
                idMap.put(currentID, key);

                buffer.get(key).getFirst().setId(currentID);
                networkTable.publishOdometry(buffer.get(key).getFirst());

                if (buffer.get(key).getSecond() != null) {
                    buffer.get(key).getSecond().setId(currentID);
                    networkTable.publishVision(buffer.get(key).getSecond());
                }

                buffer.remove(key);
            }
        }
    }
}
