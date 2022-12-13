package com._604robotics.robotnik.swerve;

import java.util.Arrays;
import java.util.Collections;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Helper class that converts a chassis velocity (dx, dy, and dtheta components) into individual
 * module states (speed and angle).
 *
 * <p>The inverse kinematics (converting from a desired chassis velocity to individual module
 * states) uses the relative locations of the modules with respect to the center of rotation. The
 * center of rotation for inverse kinematics is also variable. This means that you can set your set
 * your center of rotation in a corner of the robot to perform special evasion maneuvers.
 *
 * <p>Forward kinematics (converting an array of module states into the overall chassis motion) is
 * performs the exact opposite of what inverse kinematics does. Since this is an overdetermined
 * system (more equations than variables), we use a least-squares approximation.
 *
 * <p>The inverse kinematics: [moduleStates] = [moduleLocations] * [chassisSpeeds] We take the
 * Moore-Penrose pseudoinverse of [moduleLocations] and then multiply by [moduleStates] to get our
 * chassis speeds.
 *
 * <p>Forward kinematics is also used for odometry -- determining the position of the robot on the
 * field using encoders and a gyro.
 */
public class QuixSwerveDriveKinematics {
    private SwerveDriveKinematics kinematics;

    /**
     * Constructs a swerve drive kinematics object. This takes in a variable number of wheel locations
     * as Translation2ds. The order in which you pass in the wheel locations is the same order that
     * you will receive the module states when performing inverse kinematics. It is also expected that
     * you pass in the module states in the same order when calling the forward kinematics methods.
     *
     * @param wheelsMeters The locations of the wheels relative to the physical center of the robot.
     */
    public QuixSwerveDriveKinematics(Translation2d... wheelsMeters) {
        kinematics = new SwerveDriveKinematics(wheelsMeters);
    }

    /**
     * Performs inverse kinematics to return the module states from a desired chassis velocity. This
     * method is often used to convert joystick values into module speeds and angles.
     *
     * <p>This function also supports variable centers of rotation. During normal operations, the
     * center of rotation is usually the same as the physical center of the robot; therefore, the
     * argument is defaulted to that use case. However, if you wish to change the center of rotation
     * for evasive maneuvers, vision alignment, or for any other use case, you can do so.
     *
     * @param chassisSpeeds The desired chassis speed.
     * @param centerOfRotationMeters The center of rotation. For example, if you set the center of
     *     rotation at one corner of the robot and provide a chassis speed that only has a dtheta
     *     component, the robot will rotate around that corner.
     * @return An array containing the module states. Use caution because these module states are not
     *     normalized. Sometimes, a user input may cause one of the module speeds to go above the
     *     attainable max velocity. Use the {@link #desaturateWheelSpeeds(QuixSwerveModuleState[], double)
     *     DesaturateWheelSpeeds} function to rectify this issue.
     */
    public  QuixSwerveModuleState[] toQuixSwerveModuleStates(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
        return swerveModuleStatesToQuixSwerveModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotationMeters));
    }

    /**
     * Performs inverse kinematics. See {@link #toQuixSwerveModuleStates(ChassisSpeeds, Translation2d)}
     * toQuixSwerveModuleStates for more information.
     *
     * @param chassisSpeeds The desired chassis speed.
     * @return An array containing the module states.
     */
    public QuixSwerveModuleState[] toQuixSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        return toQuixSwerveModuleStates(chassisSpeeds, new Translation2d());
    }

    /**
     * Performs forward kinematics to return the resulting chassis state from the given module states.
     * This method is often used for odometry -- determining the robot's position on the field using
     * data from the real-world speed and angle of each module on the robot.
     *
     * @param wheelStates The state of the modules (as a QuixSwerveModuleState type) as measured from
     *     respective encoders and gyros. The order of the swerve module states should be same as
     *     passed into the constructor of this class.
     * @return The resulting chassis speed.
     */
    public ChassisSpeeds toChassisSpeeds(QuixSwerveModuleState... wheelStates) {
        return kinematics.toChassisSpeeds(quixSwerveModuleStatesToSwerveModuleStates(wheelStates));
    }

    /**
     * Renormalizes the wheel speeds if any individual speed is above the specified maximum.
     *
     * <p>Sometimes, after inverse kinematics, the requested speed from one or more modules may be
     * above the max attainable speed for the driving motor on that module. To fix this issue, one can
     * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
     * absolute threshold, while maintaining the ratio of speeds between modules.
     *
     * @param moduleStates Reference to array of module states. The array will be mutated with the
     *     normalized speeds!
     * @param attainableMaxSpeedMetersPerSecond The absolute max speed that a module can reach.
     */
    public static void desaturateWheelSpeeds(
        QuixSwerveModuleState[] moduleStates, double attainableMaxSpeedMetersPerSecond) {
        double realMaxSpeed = Collections.max(Arrays.asList(moduleStates)).speedMetersPerSecond;
        if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
            for (QuixSwerveModuleState moduleState : moduleStates) {
            moduleState.speedMetersPerSecond =
                moduleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
            }
        }
    }

    public SwerveDriveKinematics getInternalSwerverDriveKinematics() {
        return kinematics;
    }

    public static SwerveModuleState[] quixSwerveModuleStatesToSwerveModuleStates(QuixSwerveModuleState[] wheelStates) {
        SwerveModuleState[] states = new SwerveModuleState[wheelStates.length];

        for (int i = 0; i < wheelStates.length; i++) {
            QuixSwerveModuleState state = wheelStates[i];
            states[i] = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
        }

        return states;
    }

    public static QuixSwerveModuleState[] swerveModuleStatesToQuixSwerveModuleStates(SwerveModuleState[] wheelStates) {
        QuixSwerveModuleState[] states = new QuixSwerveModuleState[wheelStates.length];

        for (int i = 0; i < wheelStates.length; i++) {
            SwerveModuleState state = wheelStates[i];
            states[i] = new QuixSwerveModuleState(state.speedMetersPerSecond, state.angle);
        }

        return states;
    }

    public static SwerveModuleState quixSwerveModuleStateToSwerveModuleState(QuixSwerveModuleState wheelState) {
        return new SwerveModuleState(wheelState.speedMetersPerSecond, wheelState.angle);
    }
}
