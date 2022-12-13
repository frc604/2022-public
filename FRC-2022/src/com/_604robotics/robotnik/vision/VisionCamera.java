package com._604robotics.robotnik.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Pair;

import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/** A base class that represents a vision Camera. */
public abstract class VisionCamera extends SubsystemBase {
  private final Vector3D pose;
  private final double tilt;

  /** Instantiates a new Vision camera. */
  public VisionCamera(String name, Vector3D pose, double tilt) {
    super();

    this.pose = pose;
    this.tilt = tilt;
  }

  /**
   * Gets latest measurement.
   * @return the latest measurement
   */
  public abstract PipelineVisionPacket getLatestMeasurement();

  /**
   * Gets the tilt of the camera in degrees.
   * @return the pose
   */
  public double getTilt() {
    return tilt;
  }

  /**
   * Gets the pose of the camera.
   * @return the pose
   */
  public Vector3D getPose() {
    return pose;
  }

  /** A data class for a pipeline packet. */
  public static class PipelineVisionPacket {
    private final boolean hasTargets;
    private final Target bestTarget;
    private final List<Target> targets;
    private final double latency;

    public PipelineVisionPacket(
        boolean hasTargets, Target bestTarget, List<Target> targets, double latency) {
      this.hasTargets = hasTargets;
      this.bestTarget = bestTarget;
      this.targets = targets;
      this.latency = latency;
    }

    /**
     * If the vision packet has valid targets.
     *
     * @return if targets are found.
     */
    public boolean hasTargets() {
      return hasTargets;
    }

    /**
     * Gets best target.
     *
     * @return the best target.
     */
    public Target getBestTarget() {
      return bestTarget;
    }

    /**
     * Gets targets.
     *
     * @return the targets.
     */
    public List<Target> getTargets() {
      return targets;
    }

    /**
     * Gets latency.
     *
     * @return the latency.
     */
    public double getLatency() {
      return latency;
    }
  }

  /** A data class for a target measurement. */
  public static class Target {
    private final double yaw;
    private final double pitch;
    private final double area;
    private final double skew;
    private final ArrayList<Pair<Double, Double>> corners;

    /**
     * Instantiates a new target.
     *
     * @param yaw the yaw of the target
     * @param pitch the pitch of the target
     * @param area the area of the target
     * @param skew the skew of the target
     */
    public Target(double yaw, double pitch, double area, double skew) {
      this.yaw = yaw;
      this.pitch = pitch;
      this.area = area;
      this.skew = skew;
      this.corners = new ArrayList<>();
    }


        /**
     * Instantiates a new target.
     *
     * @param yaw the yaw of the target
     * @param pitch the pitch of the target
     * @param area the area of the target
     * @param skew the skew of the target
     * @param corners the corners of the target as a list of pairs of x and y
     */
    public Target(double yaw, double pitch, double area, double skew, ArrayList<Pair<Double, Double>> corners) {
      this.yaw = yaw;
      this.pitch = pitch;
      this.area = area;
      this.skew = skew;
      this.corners = corners;
    }

    public Target() {
      this(0.0, 0.0, 0.0, 0.0, new ArrayList<>());
    }

    /**
     * Returns a Vector3D from the yaw and pitch from the vision measurement.
     *
     * @return the vector 3D.
     */
    public Vector3D getVector3D() {
      return new Vector3D(Math.toRadians(yaw), Math.toRadians(pitch));
    }

    /**
     * Gets yaw.
     *
     * @return the yaw
     */
    public double getYaw() {
      return yaw;
    }

    /**
     * Gets pitch.
     *
     * @return the pitch
     */
    public double getPitch() {
      return pitch;
    }

    /**
     * Gets area.
     *
     * @return the area
     */
    public double getArea() {
      return area;
    }

    /**
     * Gets skew.
     *
     * @return the skew
     */
    public double getSkew() {
      return skew;
    }

    /**
     * Gets corners.
     *
     * @return the corners
     */
    public ArrayList<Pair<Double, Double>> getCorners() {
      return corners;
    }
  }
}
