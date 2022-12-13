package frc.quixlib.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.quixlib.vision.Fiducial;
import java.util.EnumSet;
import java.util.function.Consumer;

public class NTManager {
  NetworkTable m_localizerTable;
  // Robot to DS odometry measurements
  private final DoubleArrayPublisher m_odomPub;
  // Robot to DS vision measurements
  private final DoubleArrayPublisher m_visionPub;

  public NTManager(Consumer<NTPoseEstimate> newEstimateListener) {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    m_localizerTable = inst.getTable("localizer");
    m_odomPub = m_localizerTable.getDoubleArrayTopic("odometry/0").publish();
    m_visionPub = m_localizerTable.getDoubleArrayTopic("vision/0").publish();

    // Setup listener for when the estimate is updated.
    final var estimatesSub =
        m_localizerTable.getDoubleArrayTopic("estimates/0").subscribe(new double[] {});
    inst.addListener(
        estimatesSub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          newEstimateListener.accept(
              new NTPoseEstimate(
                  (int) event.valueData.value.getDoubleArray()[0],
                  new Pose2d(
                      event.valueData.value.getDoubleArray()[1],
                      event.valueData.value.getDoubleArray()[2],
                      new Rotation2d(event.valueData.value.getDoubleArray()[3]))));
        });
  }

  public void publishOdometry(NTOdometryMeasurement odometry) {
    m_odomPub.set(odometry.toArray());
  }

  public void publishVision(NTVisionMeasurement vision) {
    m_visionPub.set(vision.toArray());
  }

  public void publishTargets(Fiducial[] targets) {
    for (var target : targets) {
      var pub = m_localizerTable.getDoubleArrayTopic("targets/" + target.id()).publish();
      pub.set(new double[] {target.getX(), target.getY(), target.getZ()});
    }
  }
}
