package frc.quixlib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.quixlib.math.CameraMathUtils;
import java.util.ArrayList;

/** A data class for a target measurement. */
public class Target {
  private final int m_id;
  private final double m_yaw;
  private final double m_pitch;
  private final double m_area;
  private final double m_skew;
  private final ArrayList<Pair<Double, Double>> m_corners;

  /**
   * Instantiates a new target.
   *
   * @param id ID of the target
   * @param yaw the yaw of the target
   * @param pitch the pitch of the target
   * @param area the area of the target
   * @param skew the skew of the target
   */
  public Target(int id, double yaw, double pitch, double area, double skew) {
    m_id = id;
    m_yaw = yaw;
    m_pitch = pitch;
    m_area = area;
    m_skew = skew;
    m_corners = new ArrayList<>();
  }

  /**
   * Instantiates a new target.
   *
   * @param id ID of the target
   * @param yaw the yaw of the target
   * @param pitch the pitch of the target
   * @param area the area of the target
   * @param skew the skew of the target
   * @param corners the corners of the target as a list of pairs of x and y
   */
  public Target(
      int id,
      double yaw,
      double pitch,
      double area,
      double skew,
      ArrayList<Pair<Double, Double>> corners) {
    m_id = id;
    m_yaw = yaw;
    m_pitch = pitch;
    m_area = area;
    m_skew = skew;
    m_corners = corners;
  }

  public Target() {
    this(-1, 0.0, 0.0, 0.0, 0.0, new ArrayList<>());
  }

  /**
   * Returns the bearing and elevation of the target in spherical coodinates centered on the camera
   * frame.
   */
  public Pair<Double, Double> getSphericalBE() {
    final Matrix<N3, N1> camFramePoint =
        CameraMathUtils.pinholeBE2Cart(Math.toRadians(m_yaw), Math.toRadians(m_pitch));
    return CameraMathUtils.cart2BESph(camFramePoint);
  }

  /**
   * Gets ID.
   *
   * @return the ID
   */
  public int getID() {
    return m_id;
  }

  // /**
  //  * Gets yaw.
  //  *
  //  * @return the yaw
  //  */
  // public double getYaw() {
  //   return m_yaw;
  // }

  // /**
  //  * Gets pitch.
  //  *
  //  * @return the pitch
  //  */
  // public double getPitch() {
  //   return m_pitch;
  // }

  //   /**
  //    * Gets area.
  //    *
  //    * @return the area
  //    */
  //   public double getArea() {
  //     return m_area;
  //   }

  //   /**
  //    * Gets skew.
  //    *
  //    * @return the skew
  //    */
  //   public double getSkew() {
  //     return m_skew;
  //   }

  //   /**
  //    * Gets corners.
  //    *
  //    * @return the corners
  //    */
  //   public ArrayList<Pair<Double, Double>> getCorners() {
  //     return m_corners;
  //   }
}
