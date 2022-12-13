package frc.quixlib.vision;

import java.util.List;

/** A data class for a pipeline packet. */
public class PipelineVisionPacket {
  private final boolean m_hasTargets;
  private final Target m_bestTarget;
  private final List<Target> m_targets;
  private final double m_latency;

  public PipelineVisionPacket(
      boolean hasTargets, Target bestTarget, List<Target> targets, double latency) {
    m_hasTargets = hasTargets;
    m_bestTarget = bestTarget;
    m_targets = targets;
    m_latency = latency;
  }

  /**
   * If the vision packet has valid targets.
   *
   * @return if targets are found.
   */
  public boolean hasTargets() {
    return m_hasTargets;
  }

  /**
   * Gets best target.
   *
   * @return the best target.
   */
  public Target getBestTarget() {
    return m_bestTarget;
  }

  /**
   * Gets targets.
   *
   * @return the targets.
   */
  public List<Target> getTargets() {
    return m_targets;
  }

  /**
   * Gets latency.
   *
   * @return the latency in seconds.
   */
  public double getLatency() {
    return m_latency;
  }
}
