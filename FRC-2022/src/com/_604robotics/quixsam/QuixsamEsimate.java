package com._604robotics.quixsam;

import edu.wpi.first.math.geometry.Pose2d;

public class QuixsamEsimate {
    private int id;
    private Pose2d pose;

    public QuixsamEsimate(int id, Pose2d pose) {
        this.id = id;
        this.pose = pose;
    }

    public int getID() {
        return id;
    }

    public Pose2d getPose() {
        return pose;
    }
}
