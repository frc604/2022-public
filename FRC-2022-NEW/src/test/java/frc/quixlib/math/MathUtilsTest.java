package frc.quixlib.math;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.*;

public class MathUtilsTest {
  static final double kTol = 1e-6;

  @Test
  public void cart2pol2carTest() {
    var pol = MathUtils.cart2pol(0.0, 0.0);
    var xy = MathUtils.pol2cart(pol.getFirst(), pol.getSecond());
    assertEquals(0.0, xy.getFirst(), kTol);
    assertEquals(0.0, xy.getSecond(), kTol);

    pol = MathUtils.cart2pol(1.23, -4.56);
    xy = MathUtils.pol2cart(pol.getFirst(), pol.getSecond());
    assertEquals(1.23, xy.getFirst(), kTol);
    assertEquals(-4.56, xy.getSecond(), kTol);
  }

  @Test
  public void constrainAngleNegPiToPiTest() {
    // Zero
    assertEquals(0.0, MathUtils.constrainAngleNegPiToPi(0.0), kTol);

    // Bounds
    assertEquals(-Math.PI, MathUtils.constrainAngleNegPiToPi(-Math.PI), kTol);
    assertEquals(-Math.PI, MathUtils.constrainAngleNegPiToPi(Math.PI), kTol);
    assertEquals(Math.PI - 1e-6, MathUtils.constrainAngleNegPiToPi(Math.PI - 1e-6), kTol);

    // > PI
    assertEquals(0.5, MathUtils.constrainAngleNegPiToPi(10.0 * Math.PI + 0.5), kTol);
    assertEquals(0.5, MathUtils.constrainAngleNegPiToPi(-10.0 * Math.PI + 0.5), kTol);
  }

  @Test
  public void unwrapAngleTest() {
    assertEquals(0, MathUtils.placeInScope(0, 0), kTol);
    assertEquals(0.5, MathUtils.placeInScope(0.5, 0), kTol);
    assertEquals(-0.5, MathUtils.placeInScope(-0.5, 0), kTol);

    assertEquals(Math.toRadians(720), MathUtils.placeInScope(0, Math.toRadians(800)), kTol);
    assertEquals(Math.toRadians(720), MathUtils.placeInScope(0, Math.toRadians(700)), kTol);
    assertEquals(Math.toRadians(-720), MathUtils.placeInScope(0, Math.toRadians(-800)), kTol);
    assertEquals(Math.toRadians(-720), MathUtils.placeInScope(0, Math.toRadians(-700)), kTol);

    assertEquals(
        Math.toRadians(765), MathUtils.placeInScope(Math.toRadians(45), Math.toRadians(800)), kTol);
    assertEquals(
        Math.toRadians(765), MathUtils.placeInScope(Math.toRadians(45), Math.toRadians(700)), kTol);

    assertEquals(
        Math.toRadians(675),
        MathUtils.placeInScope(-Math.toRadians(45), Math.toRadians(800)),
        kTol);
    assertEquals(
        Math.toRadians(675),
        MathUtils.placeInScope(-Math.toRadians(45), Math.toRadians(700)),
        kTol);

    assertEquals(
        Math.toRadians(360),
        MathUtils.placeInScope(Math.toRadians(360), Math.toRadians(300)),
        kTol);
    assertEquals(
        Math.toRadians(360),
        MathUtils.placeInScope(Math.toRadians(-360), Math.toRadians(300)),
        kTol);
    assertEquals(
        Math.toRadians(360),
        MathUtils.placeInScope(Math.toRadians(720), Math.toRadians(300)),
        kTol);
    assertEquals(
        Math.toRadians(360),
        MathUtils.placeInScope(Math.toRadians(-720), Math.toRadians(300)),
        kTol);
  }
}
