package frc.quixlib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;
import frc.quixlib.math.CameraMathUtils;
import frc.quixlib.math.MathUtils;
import java.util.ArrayList;

public class QuixSimCamera implements QuixVisionCamera {
  private PipelineVisionPacket m_simPacket =
      new PipelineVisionPacket(false, new Target(), new ArrayList<Target>(), 0.0);
  ;

  private static final double fovWidth = Math.toRadians(60); // rad
  private static final double fovHeight = Math.toRadians(120); // rad
  private static final double aprilTagDetectionDistance = Units.feetToMeters(15); // m
  private static final double retroReflectiveDetectionDistance = Units.feetToMeters(25); // m

  public QuixSimCamera() {}

  public PipelineVisionPacket getLatestMeasurement() {
    return m_simPacket;
  }

  public void updateSim(Matrix<N4, N4> fieldCameraT, Fiducial[] fiducials) {
    double bestDistance = Double.POSITIVE_INFINITY;
    Target bestTarget = new Target();
    ArrayList<Target> targets = new ArrayList<Target>();
    for (final var fiducial : fiducials) {
      // Only "detect" fiducial within range.
      double dx = fieldCameraT.get(0, 3) - fiducial.getX();
      double dy = fieldCameraT.get(1, 3) - fiducial.getY();
      double dz = fieldCameraT.get(2, 3) - fiducial.getZ();
      double distToFiducial = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (distToFiducial
          > (fiducial.getType() == Fiducial.Type.APRILTAG
              ? aprilTagDetectionDistance
              : retroReflectiveDetectionDistance)) {
        continue;
      }

      // Only "detect" AprilTags if the camera is +/- 45 degrees from the front.
      if (fiducial.getType() == Fiducial.Type.APRILTAG) {
        var camPose = MathUtils.zRotTFMatrixToPose(fieldCameraT);
        Rotation2d cameraRotation =
            new Rotation2d(camPose.getX() - fiducial.getX(), camPose.getY() - fiducial.getY());
        double diff =
            MathUtils.constrainAngleNegPiToPi(cameraRotation.getRadians() - fiducial.getZRot());
        if (Math.abs(diff) > Math.PI * 0.25) {
          continue;
        }
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
      var target =
          new Target(fiducial.id(), Math.toDegrees(bearing), Math.toDegrees(elevation), 0, 0);
      targets.add(target);
      double distToCenter = Math.sqrt(bearing * bearing + elevation * elevation);
      if (distToCenter >= bestDistance) {
        continue;
      }
      bestDistance = distToCenter;
      bestTarget = target;
    }
    m_simPacket = new PipelineVisionPacket(!targets.isEmpty(), bestTarget, targets, 0.0);
  }
}
