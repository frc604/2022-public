package frc.quixlib.motorcontrol;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.*;

public class MechanismRatioTest {
  static final double kTol = 5e-2;

  @Test
  public void testIdentity() {
    MechanismRatio gr = new MechanismRatio();

    // Motor to mechanism
    assertEquals(0.0, gr.sensorRadiansToMechanismPosition(0.0), kTol);
    assertEquals(1.23, gr.sensorRadiansToMechanismPosition(1.23), kTol);
    assertEquals(-4.56, gr.sensorRadiansToMechanismPosition(-4.56), kTol);

    // Mechanism to motor
    assertEquals(0.0, gr.mechanismPositionToSensorRadians(0.0), kTol);
    assertEquals(1.23, gr.mechanismPositionToSensorRadians(1.23), kTol);
    assertEquals(-4.56, gr.mechanismPositionToSensorRadians(-4.56), kTol);
  }

  @Test
  public void testGearDown() {
    MechanismRatio gr = new MechanismRatio(1.0, 2.0, 3.0);

    // Motor to mechanism
    assertEquals(0.0, gr.sensorRadiansToMechanismPosition(0.0), kTol);
    assertEquals(1.5, gr.sensorRadiansToMechanismPosition(2.0 * Math.PI), kTol);
    assertEquals(-3.0, gr.sensorRadiansToMechanismPosition(-4.0 * Math.PI), kTol);

    // Mechanism to motor
    assertEquals(0.0, gr.mechanismPositionToSensorRadians(0.0), kTol);
    assertEquals(2.0 * Math.PI, gr.mechanismPositionToSensorRadians(1.5), kTol);
    assertEquals(-4.0 * Math.PI, gr.mechanismPositionToSensorRadians(-3.0), kTol);
  }

  @Test
  public void testGearUp() {
    MechanismRatio gr = new MechanismRatio(2.0, 1.0, 0.5);

    // Motor to mechanism
    assertEquals(0.0, gr.sensorRadiansToMechanismPosition(0.0), kTol);
    assertEquals(1.0, gr.sensorRadiansToMechanismPosition(2.0 * Math.PI), kTol);
    assertEquals(-2.0, gr.sensorRadiansToMechanismPosition(-4.0 * Math.PI), kTol);

    // Mechanism to motor
    assertEquals(0.0, gr.mechanismPositionToSensorRadians(0.0), kTol);
    assertEquals(2.0 * Math.PI, gr.mechanismPositionToSensorRadians(1.0), kTol);
    assertEquals(-4.0 * Math.PI, gr.mechanismPositionToSensorRadians(-2.0), kTol);
  }
}
