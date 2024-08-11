// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.trajectory;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.constraint.DifferentialDriveVoltageConstraint;
import java.util.ArrayList;
import java.util.Collections;
import org.junit.jupiter.api.Test;

class DifferentialDriveVoltageConstraintTest {
    @Test
    void testDifferentialDriveVoltageConstraint() {
        // Pick an unreasonably large kA to ensure the constraint has to do some work
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 1, 3);
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.5);
        double maxVoltage = 10;
        DifferentialDriveVoltageConstraint constraint =
                new DifferentialDriveVoltageConstraint(feedforward, kinematics, maxVoltage);

        Trajectory trajectory =
                TrajectoryGeneratorTest.getTrajectory(Collections.singletonList(constraint));

        double duration = trajectory.getTotalTimeSeconds();
        double t = 0.0;
        double dt = 0.02;

        while (t < duration) {
            Trajectory.State point = trajectory.sample(t);
            ChassisSpeeds chassisSpeeds =
                    new ChassisSpeeds(
                            point.velocityMetersPerSecond,
                            0,
                            point.velocityMetersPerSecond * point.curvatureRadPerMeter);

            DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

            t += dt;

            // Not really a strictly-correct test as we're using the chassis accel instead of the
            // wheel accel, but much easier than doing it "properly" and a reasonable check anyway
            assertAll(
                    () ->
                            assertTrue(
                                    feedforward.calculate(
                                                    wheelSpeeds.leftMetersPerSecond, point.accelerationMetersPerSecondSq)
                                            <= maxVoltage + 0.05),
                    () ->
                            assertTrue(
                                    feedforward.calculate(
                                                    wheelSpeeds.leftMetersPerSecond, point.accelerationMetersPerSecondSq)
                                            >= -maxVoltage - 0.05),
                    () ->
                            assertTrue(
                                    feedforward.calculate(
                                                    wheelSpeeds.rightMetersPerSecond, point.accelerationMetersPerSecondSq)
                                            <= maxVoltage + 0.05),
                    () ->
                            assertTrue(
                                    feedforward.calculate(
                                                    wheelSpeeds.rightMetersPerSecond, point.accelerationMetersPerSecondSq)
                                            >= -maxVoltage - 0.05));
        }
    }

    @Test
    void testEndpointHighCurvature() {
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 1, 3);

        // Large trackwidth - need to test with radius of curvature less than half of trackwidth
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(3);
        double maxVoltage = 10;
        DifferentialDriveVoltageConstraint constraint =
                new DifferentialDriveVoltageConstraint(feedforward, kinematics, maxVoltage);

        TrajectoryConfig config = new TrajectoryConfig(12, 12).addConstraint(constraint);

        // Radius of curvature should be ~1 meter.
        assertDoesNotThrow(
                () ->
                        TrajectoryGenerator.generateTrajectory(
                                new Pose2d(1, 0, Rotation2d.fromDegrees(90)),
                                new ArrayList<>(),
                                new Pose2d(0, 1, Rotation2d.fromDegrees(180)),
                                config));

        assertDoesNotThrow(
                () ->
                        TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 1, Rotation2d.fromDegrees(180)),
                                new ArrayList<>(),
                                new Pose2d(1, 0, Rotation2d.fromDegrees(90)),
                                config.setReversed(true)));
    }
}
