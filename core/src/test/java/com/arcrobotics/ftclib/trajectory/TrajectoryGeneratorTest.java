// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.trajectory;

import static com.arcrobotics.ftclib.util.Units.feetToMeters;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.constraint.TrajectoryConstraint;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;

class TrajectoryGeneratorTest {
    static Trajectory getTrajectory(List<? extends TrajectoryConstraint> constraints) {
        final double maxVelocity = feetToMeters(12.0);
        final double maxAccel = feetToMeters(12);

        // 2018 cross scale auto waypoints.
        Pose2d sideStart =
                new Pose2d(feetToMeters(1.54), feetToMeters(23.23), Rotation2d.fromDegrees(-180));
        Pose2d crossScale =
                new Pose2d(feetToMeters(23.7), feetToMeters(6.8), Rotation2d.fromDegrees(-160));

        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(sideStart);
        waypoints.add(
                sideStart.plus(
                        new Transform2d(
                                new Translation2d(feetToMeters(-13), feetToMeters(0)), new Rotation2d())));
        waypoints.add(
                sideStart.plus(
                        new Transform2d(
                                new Translation2d(feetToMeters(-19.5), feetToMeters(5)),
                                Rotation2d.fromDegrees(-90))));
        waypoints.add(crossScale);

        TrajectoryConfig config =
                new TrajectoryConfig(maxVelocity, maxAccel).setReversed(true).addConstraints(constraints);

        return TrajectoryGenerator.generateTrajectory(waypoints, config);
    }

    @Test
    void testGenerationAndConstraints() {
        Trajectory trajectory = getTrajectory(new ArrayList<>());

        double duration = trajectory.getTotalTimeSeconds();
        double t = 0.0;
        double dt = 0.02;

        while (t < duration) {
            Trajectory.State point = trajectory.sample(t);
            t += dt;
            assertAll(
                    () -> assertTrue(Math.abs(point.velocityMetersPerSecond) < feetToMeters(12.0) + 0.05),
                    () ->
                            assertTrue(
                                    Math.abs(point.accelerationMetersPerSecondSq) < feetToMeters(12.0) + 0.05));
        }
    }

    @Test
    void testMalformedTrajectory() {
        Trajectory traj =
                TrajectoryGenerator.generateTrajectory(
                        Arrays.asList(
                                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                                new Pose2d(1, 0, Rotation2d.fromDegrees(180))),
                        new TrajectoryConfig(feetToMeters(12), feetToMeters(12)));

        assertEquals(traj.getStates().size(), 1);
        assertEquals(traj.getTotalTimeSeconds(), 0);
    }

    @Test
    void testQuinticCurvatureOptimization() {
        Trajectory t =
                TrajectoryGenerator.generateTrajectory(
                        Arrays.asList(
                                new Pose2d(1, 0, Rotation2d.fromDegrees(90)),
                                new Pose2d(0, 1, Rotation2d.fromDegrees(180)),
                                new Pose2d(-1, 0, Rotation2d.fromDegrees(270)),
                                new Pose2d(0, -1, Rotation2d.fromDegrees(360)),
                                new Pose2d(1, 0, Rotation2d.fromDegrees(90))),
                        new TrajectoryConfig(2, 2));

        for (int i = 1; i < t.getStates().size() - 1; ++i) {
            assertNotEquals(0, t.getStates().get(i).curvatureRadPerMeter);
        }
    }
}