package frc.quixlib.motorcontrol;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.*;

public class QuixSparkMAXTest {
  static final double kTol = 5e-2;
  QuixSparkMAX controller;
  QuixSparkMAX controller_follower;
  QuixSparkMAX controller_inverted;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);

    // Default controller and follower.
    controller = new QuixSparkMAX(0, new MechanismRatio());
    controller_follower = new QuixSparkMAX(1, controller);

    // Inverted controller.
    controller_inverted = new QuixSparkMAX(2, new MechanismRatio(), true);

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
    assertEquals(1.0, controller.toNativeSensorPosition(2.0 * Math.PI), kTol);
    assertEquals(-1.0, controller.toNativeSensorPosition(-2.0 * Math.PI), kTol);

    // From native sensor position
    assertEquals(0.0, controller.fromNativeSensorPosition(0.0), kTol);
    assertEquals(2.0 * Math.PI, controller.fromNativeSensorPosition(1.0), kTol);
    assertEquals(-2.0 * Math.PI, controller.fromNativeSensorPosition(-1.0), kTol);

    // To native sensor velocity
    assertEquals(0.0, controller.toNativeSensorVelocity(0.0), kTol);
    assertEquals(60.0, controller.toNativeSensorVelocity(2.0 * Math.PI), kTol);
    assertEquals(-60.0, controller.toNativeSensorVelocity(-2.0 * Math.PI), kTol);

    // From native sensor velocity
    assertEquals(0.0, controller.fromNativeSensorVelocity(0.0), kTol);
    assertEquals(2.0 * Math.PI, controller.fromNativeSensorVelocity(60.0), kTol);
    assertEquals(-2.0 * Math.PI, controller.fromNativeSensorVelocity(-60.0), kTol);
  }

  @Test
  public void testGetInverted() {
    assertFalse(controller.getInverted());
    assertFalse(controller_follower.getInverted());
    assertTrue(controller_inverted.getInverted());
  }

  @Test
  public void testSetGetPercentOutput() throws InterruptedException {
    // Zero
    controller.setPercentOutput(0.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getPercentOutput(), 0.0, kTol);
    assertEquals(controller_follower.getPercentOutput(), 0.0, kTol);

    // // Positive value
    // controller.setPercentOutput(0.3);
    // Thread.sleep(200); // Wait for CAN delay
    // assertEquals(controller.getPercentOutput(), 0.3, kTol);
    // assertEquals(controller_follower.getPercentOutput(), 0.3, kTol);

    // // Negative value
    // controller.setPercentOutput(-0.5);
    // Thread.sleep(200); // Wait for CAN delay
    // assertEquals(controller.getPercentOutput(), -0.5, kTol);
    // assertEquals(controller_follower.getPercentOutput(), -0.5, kTol);

    // // > 1.0
    // controller.setPercentOutput(1.5);
    // Thread.sleep(200); // Wait for CAN delay
    // assertEquals(controller.getPercentOutput(), 1.0, kTol);
    // assertEquals(controller_follower.getPercentOutput(), 1.0, kTol);

    // // < -1.0
    // controller.setPercentOutput(-10.0);
    // Thread.sleep(200); // Wait for CAN delay
    // assertEquals(controller.getPercentOutput(), -1.0, kTol);
    // assertEquals(controller_follower.getPercentOutput(), -1.0, kTol);
  }

  @Test
  public void testSetGetPercentOutputInverted() throws InterruptedException {
    // Zero
    controller_inverted.setPercentOutput(0.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller_inverted.getPercentOutput(), 0.0, kTol);

    // // Positive value
    // controller_inverted.setPercentOutput(0.3);
    // Thread.sleep(200); // Wait for CAN delay
    // assertEquals(controller_inverted.getPercentOutput(), 0.3, kTol);

    // // Negative value
    // controller_inverted.setPercentOutput(-0.5);
    // Thread.sleep(200); // Wait for CAN delay
    // assertEquals(controller_inverted.getPercentOutput(), -0.5, kTol);

    // // > 1.0
    // controller_inverted.setPercentOutput(1.5);
    // Thread.sleep(200); // Wait for CAN delay
    // assertEquals(controller_inverted.getPercentOutput(), 1.0, kTol);

    // // < -1.0
    // controller_inverted.setPercentOutput(-10.0);
    // Thread.sleep(200); // Wait for CAN delay
    // assertEquals(controller_inverted.getPercentOutput(), -1.0, kTol);
  }

  @Test
  public void testEncoderSetPosition() throws InterruptedException {
    // Set positive position
    controller.setSensorPosition(1.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getSensorPosition(), 1.0, kTol);

    // Set negative position
    controller.setSensorPosition(-15.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getSensorPosition(), -15.0, kTol);

    // Set zero
    controller.zeroSensorPosition();
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller.getSensorPosition(), 0.0, kTol);

    // Set positive position
    controller_inverted.setSensorPosition(1.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller_inverted.getSensorPosition(), 1.0, kTol);

    // Set negative position
    controller_inverted.setSensorPosition(-15.0);
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller_inverted.getSensorPosition(), -15.0, kTol);

    // Set zero
    controller_inverted.zeroSensorPosition();
    Thread.sleep(200); // Wait for CAN delay
    assertEquals(controller_inverted.getSensorPosition(), 0.0, kTol);
  }

  //   @Test
  //   public void testSimSensor() throws InterruptedException {
  //     // Positive speed
  //     controller.setSimVelocity(100.0, 0.02);
  //     Thread.sleep(200); // Wait for CAN delay
  //     assertEquals(controller.getSensorVelocity(), 100.0, kTol);
  //     assertEquals(controller.getSensorPosition(), 2.0, kTol);

  //     // Negative speed
  //     controller.setSimVelocity(-100.0, 0.02);
  //     Thread.sleep(200); // Wait for CAN delay
  //     assertEquals(controller.getSensorVelocity(), -100.0, kTol);
  //     assertEquals(controller.getSensorPosition(), 0.0, kTol);
  //   }

  //   @Test
  //   public void testSimSensorInverted() throws InterruptedException {
  //     // Positive speed
  //     controller_inverted.setSimVelocity(100.0, 0.02);
  //     Thread.sleep(200); // Wait for CAN delay
  //     assertEquals(controller_inverted.getSensorVelocity(), 100.0, kTol);
  //     assertEquals(controller_inverted.getSensorPosition(), 2.0, kTol);

  //     // Negative speed
  //     controller_inverted.setSimVelocity(-100.0, 0.02);
  //     Thread.sleep(200); // Wait for CAN delay
  //     assertEquals(controller_inverted.getSensorVelocity(), -100.0, kTol);
  //     assertEquals(controller_inverted.getSensorPosition(), 0.0, kTol);
  //   }
}
