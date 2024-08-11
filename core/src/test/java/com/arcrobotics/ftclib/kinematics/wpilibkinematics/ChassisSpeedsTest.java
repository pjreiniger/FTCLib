// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.kinematics.wpilibkinematics;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import org.junit.jupiter.api.Test;

class ChassisSpeedsTest {
    private static final double kEpsilon = 1E-9;

    @Test
    void testDiscretize() {
        final ChassisSpeeds target = new ChassisSpeeds(1.0, 0.0, 0.5);
        final double duration = 1.0;
        final double dt = 0.01;

        final ChassisSpeeds speeds = ChassisSpeeds.discretize(target, duration);
        final Twist2d twist =
                new Twist2d(
                        speeds.vxMetersPerSecond * dt,
                        speeds.vyMetersPerSecond * dt,
                        speeds.omegaRadiansPerSecond * dt);

        Pose2d pose = new Pose2d();
        for (double time = 0; time < duration; time += dt) {
            pose = pose.exp(twist);
        }

        final Pose2d result = pose; // For lambda capture
        assertAll(
                () -> assertEquals(target.vxMetersPerSecond * duration, result.getX(), kEpsilon),
                () -> assertEquals(target.vyMetersPerSecond * duration, result.getY(), kEpsilon),
                () ->
                        assertEquals(
                                target.omegaRadiansPerSecond * duration,
                                result.getRotation().getRadians(),
                                kEpsilon));
    }

    @Test
    void testFromFieldRelativeSpeeds() {
        final ChassisSpeeds chassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(1.0, 0.0, 0.5, Rotation2d.fromDegrees(-90.0));

        assertAll(
                () -> assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, kEpsilon),
                () -> assertEquals(1.0, chassisSpeeds.vyMetersPerSecond, kEpsilon),
                () -> assertEquals(0.5, chassisSpeeds.omegaRadiansPerSecond, kEpsilon));
    }

    @Test
    void testFromRobotRelativeSpeeds() {
        final ChassisSpeeds chassisSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(1.0, 0.0, 0.5, Rotation2d.fromDegrees(45.0));

        assertAll(
                () -> assertEquals(1.0 / Math.sqrt(2.0), chassisSpeeds.vxMetersPerSecond, kEpsilon),
                () -> assertEquals(1.0 / Math.sqrt(2.0), chassisSpeeds.vyMetersPerSecond, kEpsilon),
                () -> assertEquals(0.5, chassisSpeeds.omegaRadiansPerSecond, kEpsilon));
    }

    @Test
    void testPlus() {
        final ChassisSpeeds left = new ChassisSpeeds(1.0, 0.5, 0.75);
        final ChassisSpeeds right = new ChassisSpeeds(2.0, 1.5, 0.25);

        final ChassisSpeeds chassisSpeeds = left.plus(right);

        assertAll(
                () -> assertEquals(3.0, chassisSpeeds.vxMetersPerSecond),
                () -> assertEquals(2.0, chassisSpeeds.vyMetersPerSecond),
                () -> assertEquals(1.0, chassisSpeeds.omegaRadiansPerSecond));
    }

    @Test
    void testMinus() {
        final ChassisSpeeds left = new ChassisSpeeds(1.0, 0.5, 0.75);
        final ChassisSpeeds right = new ChassisSpeeds(2.0, 0.5, 0.25);

        final ChassisSpeeds chassisSpeeds = left.minus(right);

        assertAll(
                () -> assertEquals(-1.0, chassisSpeeds.vxMetersPerSecond),
                () -> assertEquals(0.0, chassisSpeeds.vyMetersPerSecond),
                () -> assertEquals(0.5, chassisSpeeds.omegaRadiansPerSecond));
    }

    @Test
    void testUnaryMinus() {
        final ChassisSpeeds chassisSpeeds = (new ChassisSpeeds(1.0, 0.5, 0.75)).unaryMinus();

        assertAll(
                () -> assertEquals(-1.0, chassisSpeeds.vxMetersPerSecond),
                () -> assertEquals(-0.5, chassisSpeeds.vyMetersPerSecond),
                () -> assertEquals(-0.75, chassisSpeeds.omegaRadiansPerSecond));
    }

    @Test
    void testMultiplication() {
        final ChassisSpeeds chassisSpeeds = (new ChassisSpeeds(1.0, 0.5, 0.75)).times(2.0);

        assertAll(
                () -> assertEquals(2.0, chassisSpeeds.vxMetersPerSecond),
                () -> assertEquals(1.0, chassisSpeeds.vyMetersPerSecond),
                () -> assertEquals(1.5, chassisSpeeds.omegaRadiansPerSecond));
    }

    @Test
    void testDivision() {
        final ChassisSpeeds chassisSpeeds = (new ChassisSpeeds(1.0, 0.5, 0.75)).div(2.0);

        assertAll(
                () -> assertEquals(0.5, chassisSpeeds.vxMetersPerSecond),
                () -> assertEquals(0.25, chassisSpeeds.vyMetersPerSecond),
                () -> assertEquals(0.375, chassisSpeeds.omegaRadiansPerSecond));
    }
}
