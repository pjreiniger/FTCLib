// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.Test;

class Twist2dTest {
    @Test
    void testStraight() {
        Twist2d straight = new Twist2d(5.0, 0.0, 0.0);
        Pose2d straightPose = new Pose2d().exp(straight);

        Pose2d expected = new Pose2d(5.0, 0.0, new Rotation2d());
        assertEquals(expected, straightPose);
    }

    @Test
    void testQuarterCirle() {
        Twist2d quarterCircle = new Twist2d(5.0 / 2.0 * Math.PI, 0, Math.PI / 2.0);
        Pose2d quarterCirclePose = new Pose2d().exp(quarterCircle);

        Pose2d expected = new Pose2d(5.0, 5.0, Rotation2d.fromDegrees(90.0));
        assertEquals(expected, quarterCirclePose);
    }

    @Test
    void testDiagonalNoDtheta() {
        Twist2d diagonal = new Twist2d(2.0, 2.0, 0.0);
        Pose2d diagonalPose = new Pose2d().exp(diagonal);

        Pose2d expected = new Pose2d(2.0, 2.0, new Rotation2d());
        assertEquals(expected, diagonalPose);
    }

    @Test
    void testEquality() {
        Twist2d one = new Twist2d(5, 1, 3);
        Twist2d two = new Twist2d(5, 1, 3);
        assertEquals(one, two);
    }

    @Test
    void testInequality() {
        Twist2d one = new Twist2d(5, 1, 3);
        Twist2d two = new Twist2d(5, 1.2, 3);
        assertNotEquals(one, two);
    }

    @Test
    void testPose2dLog() {
        final Pose2d start = new Pose2d();
        final Pose2d end = new Pose2d(5.0, 5.0, Rotation2d.fromDegrees(90.0));

        final Twist2d twist = start.log(end);

        Twist2d expected = new Twist2d(5.0 / 2.0 * Math.PI, 0.0, Math.PI / 2.0);
        assertEquals(expected, twist);

        // Make sure computed twist gives back original end pose
        final Pose2d reapplied = start.exp(twist);
        assertEquals(end, reapplied);
    }
}
