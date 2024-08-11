// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class Transform2dTest {
    private static final double kEpsilon = 1E-9;

    @Test
    void testInverse() {
        Pose2d initial = new Pose2d(new Translation2d(1.0, 2.0), Rotation2d.fromDegrees(45.0));
        Transform2d transform =
                new Transform2d(new Translation2d(5.0, 0.0), Rotation2d.fromDegrees(5.0));

        Pose2d transformed = initial.plus(transform);
        Pose2d untransformed = transformed.plus(transform.inverse());

        assertEquals(initial, untransformed);
    }

    @Test
    void testComposition() {
        Pose2d initial = new Pose2d(new Translation2d(1.0, 2.0), Rotation2d.fromDegrees(45.0));
        Transform2d transform1 =
                new Transform2d(new Translation2d(5.0, 0.0), Rotation2d.fromDegrees(5.0));
        Transform2d transform2 =
                new Transform2d(new Translation2d(0.0, 2.0), Rotation2d.fromDegrees(5.0));

        Pose2d transformedSeparate = initial.plus(transform1).plus(transform2);
        Pose2d transformedCombined = initial.plus(transform1.plus(transform2));

        assertEquals(transformedSeparate, transformedCombined);
    }
}
