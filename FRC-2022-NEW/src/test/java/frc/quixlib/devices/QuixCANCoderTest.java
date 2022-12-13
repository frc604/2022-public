package frc.quixlib.devices;

import static org.junit.jupiter.api.Assertions.*;

import frc.quixlib.motorcontrol.MechanismRatio;
import org.junit.jupiter.api.*;

public class QuixCANCoderTest {
  static final double kTol = 5e-2;

  @Test
  public void testBasic() throws InterruptedException {
    QuixCANCoder cancoder = new QuixCANCoder(0, new MechanismRatio());
    assertEquals(0.0, cancoder.getPosition(), kTol);
    assertEquals(0.0, cancoder.getAbsPosition(), kTol);
  }

  @Test
  public void testSim() throws InterruptedException {
    QuixCANCoder cancoder = new QuixCANCoder(1, new MechanismRatio());
    cancoder.setSimSensorVelocity(12.3, 0.1);
    Thread.sleep(500); // Wait for CAN delay
    assertEquals(1.23, cancoder.getPosition(), kTol);
    assertEquals(1.23, cancoder.getAbsPosition(), kTol);
    assertEquals(12.3, cancoder.getVelocity(), kTol);
  }

  // @Test
  // public void testZero() throws InterruptedException {
  //   QuixCANCoder cancoder = new QuixCANCoder(2, new MechanismRatio());
  //   assertEquals(0.0, cancoder.getPosition(), kTol);
  //   assertEquals(0.0, cancoder.getAbsPosition(), kTol);

  //   cancoder.setPosition(12.3);
  //   Thread.sleep(500); // Wait for CAN delay
  //   assertEquals(12.3, cancoder.getPosition(), kTol);
  //   assertEquals(0.0, cancoder.getAbsPosition(), kTol);

  //   cancoder.zero();
  //   Thread.sleep(500); // Wait for CAN delay
  //   assertEquals(0.0, cancoder.getAbsPosition(), kTol);
  //   assertEquals(0.0, cancoder.getAbsPosition(), kTol);
  // }
}
