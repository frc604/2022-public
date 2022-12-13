package frc.quixlib.devices;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.*;

public class QuixPigeonTest {
  static final double kTol = 5e-2;

  @Test
  public void testYaw() throws InterruptedException {
    QuixPigeon pigeon = new QuixPigeon(0);

    assertEquals(0.0, pigeon.getContinuousYaw(), kTol);

    // Simulate continuous yaw.
    for (double yaw = 0.0; yaw < 6.0 * Math.PI; yaw += 0.1) {
      pigeon.setSimContinuousYaw(yaw);
      Thread.sleep(20); // Wait for CAN delay
    }
    pigeon.setSimContinuousYaw(6.0 * Math.PI);
    Thread.sleep(20); // Wait for CAN delay
    assertEquals(6.0 * Math.PI, pigeon.getContinuousYaw(), kTol);

    // Test zeroing.
    pigeon.zeroContinuousYaw();
    assertEquals(0.0, pigeon.getContinuousYaw(), kTol);

    // Test zero to offset.
    pigeon.setContinuousYaw(Math.PI);
    assertEquals(Math.PI, pigeon.getContinuousYaw(), kTol);

    // Simulate continuous yaw.
    for (double yaw = 6.0 * Math.PI; yaw < 12.0 * Math.PI; yaw += 0.1) {
      pigeon.setSimContinuousYaw(yaw);
      Thread.sleep(20); // Wait for CAN delay
    }
    pigeon.setSimContinuousYaw(12.0 * Math.PI);
    Thread.sleep(20); // Wait for CAN delay
    assertEquals(7.0 * Math.PI, pigeon.getContinuousYaw(), kTol);
  }
}
