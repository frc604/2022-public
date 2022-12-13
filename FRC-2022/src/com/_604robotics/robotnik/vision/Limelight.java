package com._604robotics.robotnik.vision;

import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Pair;

public class Limelight extends VisionCamera {
  private final NetworkTable limelightTable;

  public Limelight(String name, Vector3D pose, double tilt) {
    super(name, pose, tilt);
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    limelightTable.getEntry("stream").setNumber(2);  // USB webcam with primary camera in PIP.
  }

  @Override
  public PipelineVisionPacket getLatestMeasurement() {
    boolean hasTargets = limelightTable.getEntry("tv").getNumber(0).intValue() == 1;

    if (hasTargets) {
      double yaw = limelightTable.getEntry("tx").getDouble(0);
      double pitch = limelightTable.getEntry("ty").getDouble(0);
      double area = limelightTable.getEntry("ta").getDouble(0);
      double skew = limelightTable.getEntry("ts").getDouble(0);
      double latency = limelightTable.getEntry("tl").getDouble(0) + 11; // Magic number from LL website.

      double[] cornersLimelight = limelightTable.getEntry("tcornxy").getDoubleArray(new double[0]);

      ArrayList<Pair<Double, Double>> corners = new ArrayList<>();
      for (int i = 0; i < cornersLimelight.length; i+=2) {
        corners.add(new Pair<>(cornersLimelight[i], cornersLimelight[i+1]));
      }

      for (int i = 0; i < corners.size(); i++) {
        corners.set(i, llxyBE(corners.get(i).getFirst(), corners.get(i).getSecond()));
      }

      for (int i = 0; i < corners.size(); i++) {
        corners.set(i, new Pair<>(Units.degreesToRadians(corners.get(i).getFirst()), Units.degreesToRadians(corners.get(i).getSecond())));
      }

      Target target = new Target(yaw, pitch, area, skew, corners);
      List<Target> targets = new ArrayList<>();
      targets.add(target);

      return new PipelineVisionPacket(
          hasTargets,
          target,
          targets,
          latency);
    } else {
      return new PipelineVisionPacket(false, new Target(), new ArrayList<Target>(), 0.0);
    }
  }

  public static Pair<Double, Double> llxyBE(double x, double y) {
    double resolutionWidth = 320;
    double resolutionHeight = 240;

    double fovWidth = 59.6;
    double fovHeight = 49.7;

    double centerX = resolutionWidth / 2;
    double centerY = resolutionHeight / 2;

    double degPerPWidth = fovWidth / resolutionWidth;
    double defPerPHeight = fovHeight / resolutionHeight;

    double bearing = -(x - centerX) * degPerPWidth;
    double elevation = ((resolutionHeight - y) - centerY) * defPerPHeight;

    return new Pair<>(bearing, elevation);
  }
}
