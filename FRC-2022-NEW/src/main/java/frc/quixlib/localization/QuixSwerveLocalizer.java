package frc.quixlib.localization;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.quixlib.vision.Fiducial;
import frc.quixlib.vision.PipelineVisionPacket;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentSkipListMap;

public class QuixSwerveLocalizer {
  // Manages sending and receiving from NetworkTables.
  private final NTManager m_networkTable = new NTManager(this::newEstimateListener);

  // ID of the current measurement. Used to sync between Robot and DriverStation.
  private int m_currentID = 0;
  // Map of {id: time}
  private final HashMap<Integer, Double> m_idToTimeMap = new HashMap<>();
  // Map of {time : SwerveDriveOdometryMeasurement}
  private final ConcurrentSkipListMap<Double, SwerveDriveOdometryMeasurement> m_timeToOdometryMap =
      new ConcurrentSkipListMap<>();
  // Buffer of poses so we can get the interpolated pose at the time of a vision measurement.
  private final double kBufferHistorySeconds = 10.0; // s
  private final TimeInterpolatableBuffer<Pose2d> m_poseBuffer =
      TimeInterpolatableBuffer.createBuffer(kBufferHistorySeconds);
  // Map of {time: Pair<NTOdometryMeasurement, NTVisionMeasurement>}
  private final TreeMap<Double, Pair<NTOdometryMeasurement, NTVisionMeasurement>>
      m_timeToMeasurementMap = new TreeMap<>();

  // Continuous odometry from the last reset. Used as input to the localizer.
  private final SwerveDriveOdometry m_rawOdometry;
  // Odometry played back on top of the latest localiation estimate.
  private final SwerveDriveOdometry m_playbackOdometry;
  // Latest raw localization estimate from DS.
  private NTPoseEstimate m_latestRawEstimate = new NTPoseEstimate(0, new Pose2d());

  // Measurements within |kMutableTimeBuffer| of the current time are not considered final.
  // This gives us a chance to associate new vision measurements with an past interpolated
  // odometry measurements.
  private final double kMutableTimeBuffer = 0.2; // seconds

  public QuixSwerveLocalizer(
      SwerveDriveKinematics kinematics,
      Rotation2d initialGyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d priori,
      Fiducial[] targets) {
    m_rawOdometry = new SwerveDriveOdometry(kinematics, initialGyroAngle, modulePositions, priori);
    m_playbackOdometry =
        new SwerveDriveOdometry(kinematics, initialGyroAngle, modulePositions, priori);
    m_networkTable.publishTargets(targets);
  }

  /** Resets the localizer to the given pose. */
  public void resetPose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
    m_rawOdometry.resetPosition(gyroAngle, modulePositions, pose);
    m_playbackOdometry.resetPosition(gyroAngle, modulePositions, pose);
  }

  /** Raw odometry pose. */
  public Pose2d getOdometryPose() {
    return m_rawOdometry.getPoseMeters();
  }

  /** Localizer latency-compensated pose. */
  public Pose2d getPose() {
    return m_playbackOdometry.getPoseMeters();
  }

  /** Localizer pose from DS. Use for plotting/debugging only. */
  public Pose2d getRawPose() {
    return m_latestRawEstimate.getPose();
  }

  /** Update with odometry and optional vision. */
  public void update(SwerveDriveOdometryMeasurement odometry, PipelineVisionPacket vision) {
    final double currentTime = Timer.getFPGATimestamp();

    m_timeToOdometryMap.put(currentTime, odometry);
    m_poseBuffer.addSample(currentTime, m_rawOdometry.getPoseMeters());
    m_rawOdometry.update(odometry.getGyroAngle(), odometry.getModulePositionStates());
    m_playbackOdometry.update(odometry.getGyroAngle(), odometry.getModulePositionStates());

    double measurementTime;
    NTVisionMeasurement visionMeasurment = null;
    if (vision != null && vision.hasTargets()) {
      measurementTime = currentTime - vision.getLatency();
      visionMeasurment = new NTVisionMeasurement(0);
      for (var target : vision.getTargets()) {
        visionMeasurment.addMeasurement(
            target.getID(), target.getSphericalBE(), new Pair<>(0.1, 0.1));
      }

    } else {
      measurementTime = currentTime;
    }

    // Use the interpolated pose at the time of the vision measurement for odometry.
    final Pose2d interpolatedPose = m_poseBuffer.getSample(measurementTime).get();
    final NTOdometryMeasurement odometryMeasurment =
        new NTOdometryMeasurement(
            0, interpolatedPose, new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1)));

    m_timeToMeasurementMap.put(measurementTime, new Pair<>(odometryMeasurment, visionMeasurment));
    publishImmutableEntries();
  }

  /** Receives a pose estimate over NetworkTables and replays the latest odometry on top of it. */
  private void newEstimateListener(NTPoseEstimate estimate) {
    // Start playback odometry at the first time >= the current estimate.
    final double estimateTime = m_idToTimeMap.get(estimate.getID());
    Double curTime = m_timeToOdometryMap.ceilingKey(estimateTime);

    final var measurement = m_timeToOdometryMap.get(curTime);
    m_playbackOdometry.resetPosition(
        measurement.getGyroAngle(), measurement.getModulePositionStates(), estimate.getPose());

    // Traverse entries in |m_timeToOdometryMap| from |curTime| until the end to update
    // playback odometry.
    while (curTime != null) {
      final SwerveDriveOdometryMeasurement lastMeasurment = m_timeToOdometryMap.get(curTime);
      m_playbackOdometry.update(
          lastMeasurment.getGyroAngle(), lastMeasurment.getModulePositionStates());
      curTime = m_timeToOdometryMap.higherKey(curTime);
    }

    // Save for plotting/debugging purposes.
    m_latestRawEstimate = estimate;
  }

  /** Handles NT publishing, ID finalization, and cleanup. */
  private void publishImmutableEntries() {
    final double currentTime = Timer.getFPGATimestamp();

    // Times are in ascending order.
    final ArrayList<Double> times = new ArrayList<>(m_timeToMeasurementMap.keySet());
    for (double time : times) {
      // Entries older than |kMutableTimeBuffer| are considered immutable; assign them an ID and
      // publish them.
      if (currentTime - time > kMutableTimeBuffer) {
        // Finalize this ID at this time.
        m_idToTimeMap.put(m_currentID, time);

        // Always publish odometry.
        final var measurement = m_timeToMeasurementMap.get(time);
        final NTOdometryMeasurement odometryMeasurement = measurement.getFirst();
        odometryMeasurement.setId(m_currentID);
        m_networkTable.publishOdometry(odometryMeasurement);

        // Publish vision if available.
        final NTVisionMeasurement visionMeasurement = measurement.getSecond();
        if (visionMeasurement != null) {
          visionMeasurement.setId(m_currentID);
          m_networkTable.publishVision(visionMeasurement);
        }

        m_timeToMeasurementMap.remove(time);
        m_currentID += 1;
      }
    }
  }
}
