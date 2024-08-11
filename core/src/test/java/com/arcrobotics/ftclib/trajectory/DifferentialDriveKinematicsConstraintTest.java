// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.trajectory;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import com.arcrobotics.ftclib.util.Units;
import java.util.Collections;
import org.junit.jupiter.api.Test;

class DifferentialDriveKinematicsConstraintTest {
    @Test
    void testDifferentialDriveKinematicsConstraint() {
        double maxVelocity = Units.feetToMeters(12.0); // 12 feet per second
        DifferentialDriveKinematics kinematics =
                new DifferentialDriveKinematics(Units.inchesToMeters(27));
        DifferentialDriveKinematicsConstraint constraint =
                new DifferentialDriveKinematicsConstraint(kinematics, maxVelocity);

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
            assertAll(
                    () -> assertTrue(wheelSpeeds.leftMetersPerSecond <= maxVelocity + 0.05),
                    () -> assertTrue(wheelSpeeds.rightMetersPerSecond <= maxVelocity + 0.05));
        }
    }
}
