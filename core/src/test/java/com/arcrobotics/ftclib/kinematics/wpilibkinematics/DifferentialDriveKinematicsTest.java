// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.kinematics.wpilibkinematics;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class DifferentialDriveKinematicsTest {
    private static final double kEpsilon = 1E-9;
    private final DifferentialDriveKinematics m_kinematics =
            new DifferentialDriveKinematics(0.381 * 2);

    @Test
    void testInverseKinematicsForZeros() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);

        assertAll(
                () -> assertEquals(0.0, wheelSpeeds.leftMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, wheelSpeeds.rightMetersPerSecond, kEpsilon));
    }

    @Test
    void testForwardKinematicsForZeros() {
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds();
        ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(wheelSpeeds);

        assertAll(
                () -> assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.vyMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.omegaRadiansPerSecond, kEpsilon));
    }

    @Test
    void testInverseKinematicsForStraightLine() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(3, 0, 0);
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);

        assertAll(
                () -> assertEquals(3.0, wheelSpeeds.leftMetersPerSecond, kEpsilon),
                () -> assertEquals(3.0, wheelSpeeds.rightMetersPerSecond, kEpsilon));
    }

    @Test
    void testForwardKinematicsForStraightLine() {
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(3, 3);
        ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(wheelSpeeds);

        assertAll(
                () -> assertEquals(3.0, chassisSpeeds.vxMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.vyMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.omegaRadiansPerSecond, kEpsilon));
    }

    @Test
    void testInverseKinematicsForRotateInPlace() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, Math.PI);
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);

        assertAll(
                () -> assertEquals(-0.381 * Math.PI, wheelSpeeds.leftMetersPerSecond, kEpsilon),
                () -> assertEquals(+0.381 * Math.PI, wheelSpeeds.rightMetersPerSecond, kEpsilon));
    }

    @Test
    void testForwardKinematicsForRotateInPlace() {
        DifferentialDriveWheelSpeeds wheelSpeeds =
                new DifferentialDriveWheelSpeeds(+0.381 * Math.PI, -0.381 * Math.PI);
        ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(wheelSpeeds);

        assertAll(
                () -> assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.vyMetersPerSecond, kEpsilon),
                () -> assertEquals(-Math.PI, chassisSpeeds.omegaRadiansPerSecond, kEpsilon));
    }
}
