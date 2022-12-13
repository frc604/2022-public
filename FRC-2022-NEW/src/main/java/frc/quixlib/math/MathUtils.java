package frc.quixlib.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;

public class MathUtils {
  public static Pair<Double, Double> cart2pol(double x, double y) {
    double r = Math.sqrt(x * x + y * y);
    double theta = Math.atan2(y, x);
    return new Pair<>(r, theta);
  }

  public static Pair<Double, Double> pol2cart(double r, double theta) {
    double x = r * Math.cos(theta);
    double y = r * Math.sin(theta);
    return new Pair<>(x, y);
  }

  /** Constrains an angle to be within [-pi, pi). */
  public static double constrainAngleNegPiToPi(double angle) {
    double x = (angle + Math.PI) % (2.0 * Math.PI);
    if (x < 0) {
      x += 2.0 * Math.PI;
    }
    return x - Math.PI;
  }

  /** Returns |angle| placed within within [-pi, pi) of |referenceAngle|. */
  public static double placeInScope(double angle, double referenceAngle) {
    return referenceAngle + constrainAngleNegPiToPi(angle - referenceAngle);
  }

  /** Rotates a vector by a given angle. */
  public static Matrix<N2, N1> rotateVector(Matrix<N2, N1> vector, double theta) {
    double x1 = vector.get(0, 0);
    double y1 = vector.get(1, 0);
    double c = Math.cos(theta);
    double s = Math.sin(theta);
    double x2 = c * x1 - s * y1;
    double y2 = s * x1 + c * y1;
    return Matrix.mat(Nat.N2(), Nat.N1()).fill(x2, y2);
  }

  public static double extractTFMatrixZ(Matrix<N4, N4> matrix) {
    return matrix.get(2, 3);
  }

  public static double extractTFMatrixYRot(Matrix<N4, N4> matrix) {
    return Math.atan2(
        -matrix.get(2, 0),
        Math.sqrt(matrix.get(2, 1) * matrix.get(2, 1) + matrix.get(2, 2) * matrix.get(2, 2)));
  }

  public static Matrix<N4, N4> pose2DToZRotTFMatrix(Pose2d pose) {
    return makeZRotTFMatrix(pose.getX(), pose.getY(), 0.0, pose.getRotation().getRadians());
  }

  public static Pose2d zRotTFMatrixToPose(Matrix<N4, N4> zRotTFMatrix) {
    return new Pose2d(
        zRotTFMatrix.get(0, 3),
        zRotTFMatrix.get(1, 3),
        new Rotation2d(Math.atan2(zRotTFMatrix.get(1, 0), zRotTFMatrix.get(0, 0))));
  }

  public static Matrix<N3, N1> getPointInFrame(Matrix<N3, N1> point, Matrix<N4, N4> frame) {
    Matrix<N4, N1> pointMod =
        Matrix.mat(Nat.N4(), Nat.N1()).fill(point.get(0, 0), point.get(1, 0), point.get(2, 0), 1);
    return frame.inv().times(pointMod).block(Nat.N3(), Nat.N1(), 0, 0);
  }

  public static Matrix<N4, N4> makeYRotTFMatrix(double x, double y, double z, double theta) {
    Matrix<N4, N4> transformation_matrix =
        Matrix.mat(Nat.N4(), Nat.N4())
            .fill(
                Math.cos(theta),
                0,
                Math.sin(theta),
                x,
                0,
                1,
                0,
                y,
                -Math.sin(theta),
                0,
                Math.cos(theta),
                z,
                0,
                0,
                0,
                1);
    return transformation_matrix;
  }

  public static Matrix<N4, N4> makeZRotTFMatrix(double x, double y, double z, double theta) {
    Matrix<N4, N4> transformation_matrix =
        Matrix.mat(Nat.N4(), Nat.N4())
            .fill(
                Math.cos(theta),
                -Math.sin(theta),
                0,
                x,
                Math.sin(theta),
                Math.cos(theta),
                0,
                y,
                0,
                0,
                1,
                z,
                0,
                0,
                0,
                1);
    return transformation_matrix;
  }

  public static Pose2d convertPoseToCenterOfField(Pose2d pose) {
    return new Pose2d(
        pose.getTranslation()
            .plus(
                new Translation2d(
                    Units.inchesToMeters(12 * 54 / 2.0), Units.inchesToMeters(12 * 27 / 2.0))),
        pose.getRotation());
  }
}
