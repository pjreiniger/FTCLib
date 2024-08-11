// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.trajectory;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.arcrobotics.ftclib.trajectory.constraint.CentripetalAccelerationConstraint;
import com.arcrobotics.ftclib.util.Units;
import java.util.Collections;
import org.junit.jupiter.api.Test;

class CentripetalAccelerationConstraintTest {
    @Test
    void testCentripetalAccelerationConstraint() {
        double maxCentripetalAcceleration = Units.feetToMeters(7.0); // 7 feet per second squared
        CentripetalAccelerationConstraint constraint =
                new CentripetalAccelerationConstraint(maxCentripetalAcceleration);

        Trajectory trajectory =
                TrajectoryGeneratorTest.getTrajectory(Collections.singletonList(constraint));

        double duration = trajectory.getTotalTimeSeconds();
        double t = 0.0;
        double dt = 0.02;

        while (t < duration) {
            Trajectory.State point = trajectory.sample(t);
            double centripetalAcceleration =
                    Math.pow(point.velocityMetersPerSecond, 2) * point.curvatureRadPerMeter;

            t += dt;
            assertTrue(centripetalAcceleration <= maxCentripetalAcceleration + 0.05);
        }
    }
}
