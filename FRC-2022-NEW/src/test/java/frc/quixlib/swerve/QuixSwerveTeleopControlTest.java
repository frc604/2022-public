package frc.quixlib.swerve;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.*;

public class QuixSwerveTeleopControlTest {
  static final double kTol = 1e-6;

  @Test
  public void testDeadband() {
    final double kDeadband = 0.15;
    // Zero is still zero
    assertEquals(0.0, QuixSwerveTeleopControl.applyDeadband(0.0, kDeadband), kTol);
    // One is still one
    assertEquals(1.0, QuixSwerveTeleopControl.applyDeadband(1.0, kDeadband), kTol);
    // Output at deadband is zero
    assertEquals(0.0, QuixSwerveTeleopControl.applyDeadband(0.15, kDeadband), kTol);
    // Check other values
    assertEquals(0.25, QuixSwerveTeleopControl.applyDeadband(0.3625, kDeadband), kTol);
    assertEquals(0.5, QuixSwerveTeleopControl.applyDeadband(0.575, kDeadband), kTol);
    assertEquals(0.75, QuixSwerveTeleopControl.applyDeadband(0.7875, kDeadband), kTol);
  }
}
