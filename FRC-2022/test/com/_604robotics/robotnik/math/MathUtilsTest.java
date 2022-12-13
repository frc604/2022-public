package com._604robotics.robotnik.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;

public class MathUtilsTest {
    public static final double DELTA = 1e-6; // Allowable floating point error.

    @Test
    public void unwrapZero() {
        assertEquals(MathUtils.unwrapAngle(0, 0), 0, DELTA);
    }

    @Test
    public void unwrapPositiveToZero() {
        assertEquals(MathUtils.unwrapAngle(2.5 * Math.PI, 0), 0.5 * Math.PI, DELTA);
    }

    @Test
    public void unwrapNegativeToZero() {
        assertEquals(MathUtils.unwrapAngle(-2.5 * Math.PI, 0), -0.5 * Math.PI, DELTA);
    }

    @Test
    public void unwrapToPositive() {
        assertEquals(MathUtils.unwrapAngle(0.1 * Math.PI, 5.0 * Math.PI), 4.1 * Math.PI, DELTA);
    }

    @Test
    public void unwrapToNegative() {
        assertEquals(MathUtils.unwrapAngle(0.1 * Math.PI, -5.0 * Math.PI), -5.9 * Math.PI, DELTA);
    }

    @Test
    public void testRotateVector() {
        var v1 = Matrix.mat(Nat.N2(), Nat.N1()).fill(1, 1);
        var rotated = MathUtils.rotateVector(v1, 0.5 * Math.PI);
        assertEquals(rotated.get(0, 0), -1, DELTA);
        assertEquals(rotated.get(1, 0), 1, DELTA);
    }
}
