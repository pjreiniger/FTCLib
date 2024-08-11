// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class TrajectoryTransformTest {
    @Test
    void testTransformBy() {
        TrajectoryConfig config = new TrajectoryConfig(3, 3);
        Trajectory trajectory =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(), new ArrayList<>(), new Pose2d(1, 1, Rotation2d.fromDegrees(90)), config);

        Trajectory transformedTrajectory =
                trajectory.transformBy(
                        new Transform2d(new Translation2d(1, 2), Rotation2d.fromDegrees(30)));

        // Test initial pose.
        assertEquals(
                new Pose2d(1, 2, Rotation2d.fromDegrees(30)), transformedTrajectory.sample(0).poseMeters);

        testSameShapedTrajectory(trajectory.getStates(), transformedTrajectory.getStates());
    }

    @Test
    void testRelativeTo() {
        TrajectoryConfig config = new TrajectoryConfig(3, 3);
        Trajectory trajectory =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(1, 2, Rotation2d.fromDegrees(30.0)),
                        new ArrayList<>(),
                        new Pose2d(5, 7, Rotation2d.fromDegrees(90)),
                        config);

        Trajectory transformedTrajectory =
                trajectory.relativeTo(new Pose2d(1, 2, Rotation2d.fromDegrees(30)));

        // Test initial pose.
        assertEquals(new Pose2d(), transformedTrajectory.sample(0).poseMeters);

        testSameShapedTrajectory(trajectory.getStates(), transformedTrajectory.getStates());
    }

    void testSameShapedTrajectory(List<Trajectory.State> statesA, List<Trajectory.State> statesB) {
        for (int i = 0; i < statesA.size() - 1; i++) {
            Pose2d a1 = statesA.get(i).poseMeters;
            Pose2d a2 = statesA.get(i + 1).poseMeters;

            Pose2d b1 = statesB.get(i).poseMeters;
            Pose2d b2 = statesB.get(i + 1).poseMeters;

            assertEquals(a2.relativeTo(a1), b2.relativeTo(b1));
        }
    }
}
