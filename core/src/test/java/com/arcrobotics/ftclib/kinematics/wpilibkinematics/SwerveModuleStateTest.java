// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.kinematics.wpilibkinematics;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class SwerveModuleStateTest {
    private static final double kEpsilon = 1E-9;

    @Test
    void testOptimize() {
        Rotation2d angleA = Rotation2d.fromDegrees(45);
        SwerveModuleState refA = new SwerveModuleState(-2.0, Rotation2d.fromDegrees(180));
        SwerveModuleState optimizedA = SwerveModuleState.optimize(refA, angleA);

        assertAll(
                () -> assertEquals(2.0, optimizedA.speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, optimizedA.angle.getDegrees(), kEpsilon));

        Rotation2d angleB = Rotation2d.fromDegrees(-50);
        SwerveModuleState refB = new SwerveModuleState(4.7, Rotation2d.fromDegrees(41));
        SwerveModuleState optimizedB = SwerveModuleState.optimize(refB, angleB);

        assertAll(
                () -> assertEquals(-4.7, optimizedB.speedMetersPerSecond, kEpsilon),
                () -> assertEquals(-139.0, optimizedB.angle.getDegrees(), kEpsilon));
    }

    @Test
    void testNoOptimize() {
        Rotation2d angleA = Rotation2d.fromDegrees(0);
        SwerveModuleState refA = new SwerveModuleState(2.0, Rotation2d.fromDegrees(89));
        SwerveModuleState optimizedA = SwerveModuleState.optimize(refA, angleA);

        assertAll(
                () -> assertEquals(2.0, optimizedA.speedMetersPerSecond, kEpsilon),
                () -> assertEquals(89.0, optimizedA.angle.getDegrees(), kEpsilon));

        Rotation2d angleB = Rotation2d.fromDegrees(0);
        SwerveModuleState refB = new SwerveModuleState(-2.0, Rotation2d.fromDegrees(-2));
        SwerveModuleState optimizedB = SwerveModuleState.optimize(refB, angleB);

        assertAll(
                () -> assertEquals(-2.0, optimizedB.speedMetersPerSecond, kEpsilon),
                () -> assertEquals(-2.0, optimizedB.angle.getDegrees(), kEpsilon));
    }
}
