package frc.quixlib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N4;

public interface QuixVisionCamera {
  /** Returns the latest measurement. */
  public PipelineVisionPacket getLatestMeasurement();

  /**
   * Simulates vision by updating NetworkTables based on the 3D transform of the camera in field
   * coordinates.
   */
  public void updateSim(Matrix<N4, N4> fieldCameraT, Fiducial[] targets);
}
