package frc.quixlib.motorcontrol;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.*;

public class QuixTalonFXTest {
  static final double kTol = 5e-2;
  QuixTalonFX controller;
  QuixTalonFX controller_follower;
  QuixTalonFX controller_inverted;
  QuixTalonFX controller_with_ratio;
  QuixTalonFX controller_with_ratio_inverted;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);

    // Default controller and follower.
    controller = new QuixTalonFX(0, new MechanismRatio());
    controller_follower = new QuixTalonFX(1, controller);

    // Inverted controller.
    controller_inverted = new QuixTalonFX(2, new MechanismRatio(), true);

    // Controller with ratio.
    controller_with_ratio = new QuixTalonFX(3, new MechanismRatio(1.0, 2.0));
    controller_with_ratio_inverted = new QuixTalonFX(4, new MechanismRatio(1.0, 2.0), true);

    // Spoof robot enabled for 10s
    Unmanaged.feedEnable(10000);
  }

  @AfterEach
  public void shutdown() throws Exception {
    controller.close();
    controller_follower.close();
    controller_inverted.close();
  }

  @Test
  public void testUnitConversions() throws InterruptedException {
    // To native sensor position
    assertEquals(0.0, controller.toNativeSensorPosition(0.0), kTol);
    assertEquals(2048, controller.toNativeSensorPosition(2.0 * Math.PI), kTol);
    assertEquals(-2048, controller.toNativeSensorPosition(-2.0 * Math.PI), kTol);

    assertEquals(0.0, controller_with_ratio.toNativeSensorPosition(0.0), kTol);
    assertEquals(4096, controller_with_ratio.toNativeSensorPosition(2.0 * Math.PI), kTol);
    assertEquals(-4096, controller_with_ratio.toNativeSensorPosition(-2.0 * Math.PI), kTol);

    // From native sensor position
    assertEquals(0.0, controller.fromNativeSensorPosition(0.0), kTol);
    assertEquals(2.0 * Math.PI, controller.fromNativeSensorPosition(2048), kTol);
    assertEquals(-2.0 * Math.PI, controller.fromNativeSensorPosition(-2048), kTol);

    assertEquals(0.0, controller_with_ratio.fromNativeSensorPosition(0.0), kTol);
    assertEquals(2.0 * Math.PI, controller_with_ratio.fromNativeSensorPosition(4096), kTol);
    assertEquals(-2.0 * Math.PI, controller_with_ratio.fromNativeSensorPosition(-4096), kTol);

    // To native sensor velocity
    assertEquals(0.0, controller.toNativeSensorVelocity(0.0), kTol);
    assertEquals(204.8, controller.toNativeSensorVelocity(2.0 * Math.PI), kTol);
    assertEquals(-204.8, controller.toNativeSensorVelocity(-2.0 * Math.PI), kTol);

    assertEquals(0.0, controller_with_ratio.toNativeSensorVelocity(0.0), kTol);
    assertEquals(409.6, controller_with_ratio.toNativeSensorVelocity(2.0 * Math.PI), kTol);
    assertEquals(-409.6, controller_with_ratio.toNativeSensorVelocity(-2.0 * Math.PI), kTol);

    // From native sensor velocity
    assertEquals(0.0, controller.fromNativeSensorVelocity(0.0), kTol);
    assertEquals(2.0 * Math.PI, controller.fromNativeSensorVelocity(204.8), kTol);
    assertEquals(-2.0 * Math.PI, controller.fromNativeSensorVelocity(-204.8), kTol);

    assertEquals(0.0, controller_with_ratio.fromNativeSensorVelocity(0.0), kTol);
    assertEquals(2.0 * Math.PI, controller_with_ratio.fromNativeSensorVelocity(409.6), kTol);
    assertEquals(-2.0 * Math.PI, controller_with_ratio.fromNativeSensorVelocity(-409.6), kTol);
  }

  @Test
  public void testGetInverted() {
    assertFalse(controller.getInverted());
    assertFalse(controller_follower.getInverted());
    assertTrue(controller_inverted.getInverted());
    assertFalse(controller_with_ratio.getInverted());
    assertTrue(controller_with_ratio_inverted.getInverted());
  }

  @Test
  public void testSetGetPercentOutput() throws InterruptedException {
    // Zero
    controller.setPercentOutput(0.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getPercentOutput(), 0.0, kTol);
    assertEquals(controller_follower.getPercentOutput(), 0.0, kTol);

    // Positive value
    controller.setPercentOutput(0.3);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getPercentOutput(), 0.3, kTol);
    assertEquals(controller_follower.getPercentOutput(), 0.3, kTol);

    // Negative value
    controller.setPercentOutput(-0.5);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getPercentOutput(), -0.5, kTol);
    assertEquals(controller_follower.getPercentOutput(), -0.5, kTol);

    // > 1.0
    controller.setPercentOutput(1.5);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getPercentOutput(), 1.0, kTol);
    assertEquals(controller_follower.getPercentOutput(), 1.0, kTol);

    // < -1.0
    controller.setPercentOutput(-10.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getPercentOutput(), -1.0, kTol);
    assertEquals(controller_follower.getPercentOutput(), -1.0, kTol);
  }

  @Test
  public void testSetGetPercentOutputInverted() throws InterruptedException {
    // Zero
    controller_inverted.setPercentOutput(0.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller_inverted.getPercentOutput(), 0.0, kTol);

    // Positive value
    controller_inverted.setPercentOutput(0.3);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller_inverted.getPercentOutput(), 0.3, kTol);

    // Negative value
    controller_inverted.setPercentOutput(-0.5);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller_inverted.getPercentOutput(), -0.5, kTol);

    // > 1.0
    controller_inverted.setPercentOutput(1.5);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller_inverted.getPercentOutput(), 1.0, kTol);

    // < -1.0
    controller_inverted.setPercentOutput(-10.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller_inverted.getPercentOutput(), -1.0, kTol);
  }

  @Test
  public void testEncoderSetPosition() throws InterruptedException {
    // Set positive position
    controller.setSensorPosition(1.0);
    controller_inverted.setSensorPosition(1.0);
    controller_with_ratio.setSensorPosition(1.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getSensorPosition(), 1.0, kTol);
    assertEquals(controller_inverted.getSensorPosition(), 1.0, kTol);
    assertEquals(controller_with_ratio.getSensorPosition(), 1.0, kTol);

    // Set negative position
    controller.setSensorPosition(-15.0);
    controller_inverted.setSensorPosition(-15.0);
    controller_with_ratio.setSensorPosition(-15.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getSensorPosition(), -15.0, kTol);
    assertEquals(controller_inverted.getSensorPosition(), -15.0, kTol);
    assertEquals(controller_with_ratio.getSensorPosition(), -15.0, kTol);

    // Set zero
    controller.zeroSensorPosition();
    controller_inverted.zeroSensorPosition();
    controller_with_ratio.zeroSensorPosition();
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getSensorPosition(), 0.0, kTol);
    assertEquals(controller_inverted.getSensorPosition(), 0.0, kTol);
    assertEquals(controller_with_ratio.getSensorPosition(), 0.0, kTol);
  }

  @Test
  public void testSimSensor() throws InterruptedException {
    controller.setSensorPosition(10.0);
    controller_with_ratio.setSensorPosition(10.0);
    Thread.sleep(200); // Wait for CAN delay

    // Positive speed
    controller.setSimSensorVelocity(100.0, 0.02, controller.getMechanismRatio());
    controller_with_ratio.setSimSensorVelocity(
        100.0, 0.02, controller_with_ratio.getMechanismRatio());
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getSensorVelocity(), 100.0, kTol);
    assertEquals(controller.getSensorPosition(), 12.0, kTol);
    assertEquals(controller_with_ratio.getSensorVelocity(), 100.0, kTol);
    assertEquals(controller_with_ratio.getSensorPosition(), 12.0, kTol);

    // Negative speed
    controller.setSimSensorVelocity(-100.0, 0.02, controller.getMechanismRatio());
    controller_with_ratio.setSimSensorVelocity(
        -100.0, 0.02, controller_with_ratio.getMechanismRatio());
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getSensorVelocity(), -100.0, kTol);
    assertEquals(controller.getSensorPosition(), 10.0, kTol);
    assertEquals(controller_with_ratio.getSensorVelocity(), -100.0, kTol);
    assertEquals(controller_with_ratio.getSensorPosition(), 10.0, kTol);
  }

  @Test
  public void testSimSensorInverted() throws InterruptedException {
    controller_inverted.setSensorPosition(10.0);
    controller_with_ratio_inverted.setSensorPosition(10.0);
    Thread.sleep(200); // Wait for CAN delay

    // Positive speed
    controller_inverted.setSimSensorVelocity(100.0, 0.02, controller_inverted.getMechanismRatio());
    controller_with_ratio_inverted.setSimSensorVelocity(
        100.0, 0.02, controller_with_ratio_inverted.getMechanismRatio());
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller_inverted.getSensorVelocity(), 100.0, kTol);
    assertEquals(controller_inverted.getSensorPosition(), 12.0, kTol);
    assertEquals(controller_with_ratio_inverted.getSensorVelocity(), 100.0, kTol);
    assertEquals(controller_with_ratio_inverted.getSensorPosition(), 12.0, kTol);

    // Negative speed
    controller_inverted.setSimSensorVelocity(-100.0, 0.02, controller_inverted.getMechanismRatio());
    controller_with_ratio_inverted.setSimSensorVelocity(
        -100.0, 0.02, controller_with_ratio_inverted.getMechanismRatio());
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller_inverted.getSensorVelocity(), -100.0, kTol);
    assertEquals(controller_inverted.getSensorPosition(), 10.0, kTol);
    assertEquals(controller_with_ratio_inverted.getSensorVelocity(), -100.0, kTol);
    assertEquals(controller_with_ratio_inverted.getSensorPosition(), 10.0, kTol);
  }

  @Test
  public void testPositionControl() throws InterruptedException {
    double kMaxVoltage = 12.0;
    int kSlotZero = 0;
    int kSlotPositive = 1;
    int kSlotNegative = 2;

    controller.setPIDConfig(kSlotZero, new PIDConfig(0.0, 0.0, 0.0));
    controller.setPIDConfig(kSlotPositive, new PIDConfig(1.0, 0.0, 0.0));
    controller.setPIDConfig(kSlotNegative, new PIDConfig(-1.0, 0.0, 0.0));
    controller_with_ratio.setPIDConfig(kSlotZero, new PIDConfig(0.0, 0.0, 0.0));
    controller_with_ratio.setPIDConfig(kSlotPositive, new PIDConfig(1.0, 0.0, 0.0));
    controller_with_ratio.setPIDConfig(kSlotNegative, new PIDConfig(-1.0, 0.0, 0.0));

    // Zero gains
    controller.setPositionSetpoint(kSlotZero, 100.0);
    controller_with_ratio.setPositionSetpoint(kSlotZero, 100.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getPercentOutput(), 0.0, kTol);
    assertEquals(controller_with_ratio.getPercentOutput(), 0.0, kTol);

    // Feed-forward only
    controller.setPositionSetpoint(kSlotZero, 100.0, 0.5 * kMaxVoltage);
    controller_with_ratio.setPositionSetpoint(kSlotZero, 100.0, 0.5 * kMaxVoltage);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getPercentOutput(), 0.5, kTol);
    assertEquals(controller_with_ratio.getPercentOutput(), 0.5, kTol);

    // Positive kP, positive setpoint
    controller.setPositionSetpoint(kSlotPositive, 100.0);
    controller_with_ratio.setPositionSetpoint(kSlotPositive, 100.0);
    Thread.sleep(200); // Wait for CAN delay
    assertTrue(controller.getPercentOutput() > 0.0);
    assertTrue(controller_with_ratio.getPercentOutput() > 0.0);

    // Update position so error is zero.
    controller.setSensorPosition(100.0);
    controller_with_ratio.setSensorPosition(100.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getPercentOutput(), 0.0, kTol);
    assertEquals(controller_with_ratio.getPercentOutput(), 0.0, kTol);

    // Reset position.
    controller.setSensorPosition(100.0);
    controller_with_ratio.setSensorPosition(100.0);

    // Positive kP, negative setpoint
    controller.setPositionSetpoint(kSlotPositive, -100.0);
    controller_with_ratio.setPositionSetpoint(kSlotPositive, -100.0);
    Thread.sleep(200); // Wait for CAN delay
    assertTrue(controller.getPercentOutput() < 0.0);
    assertTrue(controller_with_ratio.getPercentOutput() < 0.0);

    // Negative kP, positive setpoint should result in no output.
    controller.setPositionSetpoint(kSlotNegative, 100.0);
    controller_with_ratio.setPositionSetpoint(kSlotNegative, 100.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getPercentOutput(), 0.0, kTol);
    assertEquals(controller_with_ratio.getPercentOutput(), 0.0, kTol);

    // Negative kP, negative setpoint should result in no output.
    controller.setPositionSetpoint(kSlotNegative, -100.0);
    controller_with_ratio.setPositionSetpoint(kSlotNegative, -100.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getPercentOutput(), 0.0, kTol);
    assertEquals(controller_with_ratio.getPercentOutput(), 0.0, kTol);
  }
}
