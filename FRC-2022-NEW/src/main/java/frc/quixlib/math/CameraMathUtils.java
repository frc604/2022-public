package frc.quixlib.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class CameraMathUtils {
  /**
   * Converts cartesian coordinates in camera frame to bearing/elevation that would be returned by a
   * pinhole camera model. X-forward, Y-left, Z-up.
   */
  public static Pair<Double, Double> cart2BEPinhole(Matrix<N3, N1> cartesian) {
    double x = cartesian.get(0, 0);
    double y = cartesian.get(1, 0);
    double z = cartesian.get(2, 0);

    double bearing = Math.atan2(y, x);
    double elevation = Math.atan2(z, x);

    // Negate bearing because camera returns +bearing to the right.
    return new Pair<>(-bearing, elevation);
  }

  /**
   * Returns the camera-frame vector given the bearing and elevation in a pinhole model. X-forward,
   * Y-left, Z-up.
   */
  public static Matrix<N3, N1> pinholeBE2Cart(double bearing, double elevation) {
    // Negate bearing because camera +bearing is to the right.
    double x = 1.0;
    double y = x * Math.tan(-bearing);
    double z = x * Math.tan(elevation);
    return Matrix.mat(Nat.N3(), Nat.N1()).fill(x, y, z);
  }

  /**
   * Converts cartesian coordinates in camera frame to bearing/elevation in a spherical coordinate
   * system. X-forward, Y-left, Z-up.
   */
  public static Pair<Double, Double> cart2BESph(Matrix<N3, N1> cartesian) {
    double x = cartesian.get(0, 0);
    double y = cartesian.get(1, 0);
    double z = cartesian.get(2, 0);

    double bearing = Math.atan2(y, x);
    double elevation = Math.atan2(z, Math.sqrt(x * x + y * y));

    return new Pair<>(bearing, elevation);
  }

  /**
   * Converts bearing/elevation in a spherical coordinate system to cartesian coordinates.
   * X-forward, Y-left, Z-up.
   */
  public static Matrix<N3, N1> beSph2Cart(double bearing, double elevation) {
    double x = Math.cos(bearing) * Math.cos(elevation);
    double y = Math.sin(bearing) * Math.cos(elevation);
    double z = Math.sin(elevation);

    return Matrix.mat(Nat.N3(), Nat.N1()).fill(x, y, z);
  }
}
