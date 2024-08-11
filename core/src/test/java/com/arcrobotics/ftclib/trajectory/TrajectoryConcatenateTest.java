// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import java.util.ArrayList;
import org.junit.jupiter.api.Test;

class TrajectoryConcatenateTest {
    @Test
    void testStates() {
        Trajectory t1 =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(),
                        new ArrayList<>(),
                        new Pose2d(1, 1, new Rotation2d()),
                        new TrajectoryConfig(2, 2));

        Trajectory t2 =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(1, 1, new Rotation2d()),
                        new ArrayList<>(),
                        new Pose2d(2, 2, Rotation2d.fromDegrees(45)),
                        new TrajectoryConfig(2, 2));

        Trajectory t = t1.concatenate(t2);

        double time = -1.0;
        for (int i = 0; i < t.getStates().size(); ++i) {
            Trajectory.State state = t.getStates().get(i);

            // Make sure that the timestamps are strictly increasing.
            assertTrue(state.timeSeconds > time);
            time = state.timeSeconds;

            // Ensure that the states in t are the same as those in t1 and t2.
            if (i < t1.getStates().size()) {
                assertEquals(state, t1.getStates().get(i));
            } else {
                Trajectory.State st = t2.getStates().get(i - t1.getStates().size() + 1);
                st.timeSeconds += t1.getTotalTimeSeconds();
                assertEquals(state, st);
            }
        }
    }
}
