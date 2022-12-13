package frc.quixlib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.quixlib.math.CameraMathUtils;
import frc.quixlib.math.MathUtils;
import java.util.ArrayList;
import java.util.List;

public class Limelight implements QuixVisionCamera {
  private final NetworkTable m_limelightTable;

  // Magic number from LL website.
  private static final double kImageCaptureLatencyMs = 11.0; // ms
  private static final double fovWidth = Math.toRadians(59.6); // rad
  private static final double fovHeight = Math.toRadians(49.7); // rad

  public Limelight(String tableName) {
    m_limelightTable = NetworkTableInstance.getDefault().getTable(tableName);
    m_limelightTable.getEntry("stream").setNumber(2); // USB webcam with primary camera in PIP.
  }

  public PipelineVisionPacket getLatestMeasurement() {
    boolean hasTargets = m_limelightTable.getEntry("tv").getNumber(0).intValue() == 1;

    if (hasTargets) {
      double yaw = m_limelightTable.getEntry("tx").getDouble(0);
      double pitch = m_limelightTable.getEntry("ty").getDouble(0);
      double area = m_limelightTable.getEntry("ta").getDouble(0);
      double skew = m_limelightTable.getEntry("ts").getDouble(0);
      double latency =
          (m_limelightTable.getEntry("tl").getDouble(0) + kImageCaptureLatencyMs) / 1000.0;

      double[] cornersLimelight =
          m_limelightTable.getEntry("tcornxy").getDoubleArray(new double[0]);

      ArrayList<Pair<Double, Double>> corners = new ArrayList<>();
      for (int i = 0; i < cornersLimelight.length; i += 2) {
        corners.add(new Pair<>(cornersLimelight[i], cornersLimelight[i + 1]));
      }

      for (int i = 0; i < corners.size(); i++) {
        corners.set(i, llxyBE(corners.get(i).getFirst(), corners.get(i).getSecond()));
      }

      for (int i = 0; i < corners.size(); i++) {
        corners.set(
            i,
            new Pair<>(
                Units.degreesToRadians(corners.get(i).getFirst()),
                Units.degreesToRadians(corners.get(i).getSecond())));
      }

      Target target = new Target(-1, yaw, pitch, area, skew, corners);
      List<Target> targets = new ArrayList<>();
      targets.add(target);

      return new PipelineVisionPacket(hasTargets, target, targets, latency);
    } else {
      return new PipelineVisionPacket(false, new Target(), new ArrayList<Target>(), 0.0);
    }
  }

  private static Pair<Double, Double> llxyBE(double x, double y) {
    double resolutionWidth = 320;
    double resolutionHeight = 240;

    double centerX = resolutionWidth / 2;
    double centerY = resolutionHeight / 2;

    double degPerPWidth = fovWidth / resolutionWidth;
    double defPerPHeight = fovHeight / resolutionHeight;

    double bearing = -(x - centerX) * degPerPWidth;
    double elevation = ((resolutionHeight - y) - centerY) * defPerPHeight;

    return new Pair<>(bearing, elevation);
  }

  public void updateSim(Matrix<N4, N4> fieldCameraT, Fiducial[] fiducials) {
    // Simualte the retrofreflective fiducial closest to the center of the camera.
    double bestDistance = Double.POSITIVE_INFINITY;
    double tx = 0;
    double ty = 0;
    for (final var fiducial : fiducials) {
      if (fiducial.getType() != Fiducial.Type.RETROREFLECTIVE) {
        continue;
      }
      var pinholeBE =
          CameraMathUtils.cart2BEPinhole(
              MathUtils.getPointInFrame(
                  Matrix.mat(Nat.N3(), Nat.N1())
                      .fill(fiducial.getX(), fiducial.getY(), fiducial.getZ()),
                  fieldCameraT));
      double bearing = pinholeBE.getFirst();
      double elevation = pinholeBE.getSecond();
      boolean targetVisible =
          Math.abs(bearing) < fovWidth * 0.5 && Math.abs(elevation) < fovHeight * 0.5;
      if (!targetVisible) {
        continue;
      }
      double dist = Math.sqrt(bearing * bearing + elevation * elevation);
      if (dist >= bestDistance) {
        continue;
      }
      bestDistance = dist;
      tx = bearing;
      ty = elevation;
    }

    m_limelightTable.getEntry("tv").setNumber(bestDistance == Double.POSITIVE_INFINITY ? 0 : 1);
    m_limelightTable.getEntry("tx").setDouble(Math.toDegrees(tx));
    m_limelightTable.getEntry("ty").setDouble(Math.toDegrees(ty));
    // Simulation has zero latency, so we need to cancel out the image capture latency.
    m_limelightTable.getEntry("tl").setDouble(-kImageCaptureLatencyMs);
  }
}
