package com._604robotics.robot2022;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.Supplier;

import com._604robotics.robot2022.balls.BallManager;
import com._604robotics.robot2022.balls.BallManager.BallPathState;
import com._604robotics.robot2022.balls.BallManager.BallState;

import org.junit.jupiter.api.Test;

public class BallManagerTest {
    private class SettableSupplier<T> {
        private T value;

        public SettableSupplier(T defaultValue) {
            this.value = defaultValue;
        }

        public T get() {
            return value;
        }

        public void set(T newValue) {
            value = newValue;
        }
    }

    // @Test
    // public void testDefaultState() {
    //     SettableSupplier<BallState> topQueue = new SettableSupplier<BallState>(BallState.NONE);
    //     SettableSupplier<BallState> bottomQueue = new SettableSupplier<BallState>(BallState.NONE);
    //     SettableSupplier<Boolean> intakeButtonState = new SettableSupplier<Boolean>(false);
    //     SettableSupplier<Boolean> shootButtonState = new SettableSupplier<Boolean>(false);
    //     SettableSupplier<Boolean> purgeButtonState = new SettableSupplier<Boolean>(false);

    //     BallManager bm = new BallManager(topQueue::get, bottomQueue::get, intakeButtonState::get, shootButtonState::get, purgeButtonState::get);
    //     assertFalse(bm.shouldRunIntake());
    //     assertFalse(bm.shouldRunIntakeAndMoveToTopQueue());
    //     assertFalse(bm.shouldEjectFromIntake());
    //     assertFalse(bm.shouldEjectFromLauncher());
    //     assertFalse(bm.shouldShootFromLauncher());
    //     assertFalse(bm.shouldPurgeFromLauncher());
    // }

    // @Test
    // public void testLatchedIntake() {
    //     SettableSupplier<BallState> topQueue = new SettableSupplier<BallState>(BallState.NONE);
    //     SettableSupplier<BallState> bottomQueue = new SettableSupplier<BallState>(BallState.NONE);
    //     SettableSupplier<Boolean> intakeButtonState = new SettableSupplier<Boolean>(false);
    //     SettableSupplier<Boolean> shootButtonState = new SettableSupplier<Boolean>(false);
    //     SettableSupplier<Boolean> purgeButtonState = new SettableSupplier<Boolean>(false);

    //     BallManager bm = new BallManager(topQueue::get, bottomQueue::get, intakeButtonState::get, shootButtonState::get, purgeButtonState::get);
    //     assertFalse(bm.shouldRunIntake());
    //     assertFalse(bm.shouldRunIntakeAndMoveToTopQueue());
    //     assertFalse(bm.shouldEjectFromIntake());
    //     assertFalse(bm.shouldEjectFromLauncher());
    //     assertFalse(bm.shouldShootFromLauncher());

    //     // Simulate intake button pressed.
    //     intakeButtonState.set(true);
    //     bm.periodic();

    //     // Expect to run intake.
    //     assertTrue(bm.shouldRunIntake());

    //     // Simulate intake button released.
    //     intakeButtonState.set(false);
    //     bm.periodic();

    //     // Expect intake to stay running.
    //     assertTrue(bm.shouldRunIntake());
    // }


    // @Test
    // public void testPurge() {
    //     SettableSupplier<BallState> topQueue = new SettableSupplier<BallState>(BallState.CORRECT);
    //     SettableSupplier<BallState> bottomQueue = new SettableSupplier<BallState>(BallState.CORRECT);
    //     SettableSupplier<Boolean> intakeButtonState = new SettableSupplier<Boolean>(false);
    //     SettableSupplier<Boolean> shootButtonState = new SettableSupplier<Boolean>(false);
    //     SettableSupplier<Boolean> purgeButtonState = new SettableSupplier<Boolean>(false);

    //     BallManager bm = new BallManager(topQueue::get, bottomQueue::get, intakeButtonState::get, shootButtonState::get, purgeButtonState::get);
    //     assertFalse(bm.shouldRunIntake());
    //     assertFalse(bm.shouldRunIntakeAndMoveToTopQueue());
    //     assertFalse(bm.shouldEjectFromIntake());
    //     assertFalse(bm.shouldEjectFromLauncher());
    //     assertFalse(bm.shouldShootFromLauncher());

    //     // Simulate purge button pressed.
    //     purgeButtonState.set(true);
    //     bm.periodic();

    //     // Expect to purge.
    //     assertTrue(bm.shouldPurgeFromLauncher());
    //     assertTrue(bm.getCurrentState() == BallPathState.EMPTY_EMPTY);

    //     // Simulate purgeButtonState button released.
    //     purgeButtonState.set(false);
    //     intakeButtonState.set(true);
    //     bm.periodic();

    //     // Expect to stop pruging.
    //     assertTrue(bm.shouldRunIntake());
    // }
}
