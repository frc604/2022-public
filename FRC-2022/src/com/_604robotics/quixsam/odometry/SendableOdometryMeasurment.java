package com._604robotics.quixsam.odometry;

import edu.wpi.first.math.geometry.Pose2d;

public class SendableOdometryMeasurment {
    private int id;
    private Pose2d pose;
    private Pose2d sigmas;

    public SendableOdometryMeasurment(int id, Pose2d pose, Pose2d sigmas) {
        this.id = id;
        this.pose = pose;
        this.sigmas = sigmas;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public Pose2d getPose() {
        return pose;
    }

    public Pose2d getSigmas() {
        return sigmas;
    }
}
