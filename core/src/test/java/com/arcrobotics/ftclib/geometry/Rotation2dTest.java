// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.geometry;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.Test;

class Rotation2dTest {
    private static final double kEpsilon = 1E-9;

    @Test
    void testRadiansToDegrees() {
        Rotation2d rot1 = Rotation2d.fromRadians(Math.PI / 3);
        Rotation2d rot2 = Rotation2d.fromRadians(Math.PI / 4);

        assertAll(
                () -> assertEquals(60.0, rot1.getDegrees(), kEpsilon),
                () -> assertEquals(45.0, rot2.getDegrees(), kEpsilon));
    }

    @Test
    void testRadiansAndDegrees() {
        Rotation2d rot1 = Rotation2d.fromDegrees(45.0);
        Rotation2d rot2 = Rotation2d.fromDegrees(30.0);

        assertAll(
                () -> assertEquals(Math.PI / 4.0, rot1.getRadians(), kEpsilon),
                () -> assertEquals(Math.PI / 6.0, rot2.getRadians(), kEpsilon));
    }

    @Test
    void testRotateByFromZero() {
        Rotation2d zero = new Rotation2d();
        Rotation2d rotated = zero.rotateBy(Rotation2d.fromDegrees(90.0));

        assertAll(
                () -> assertEquals(Math.PI / 2.0, rotated.getRadians(), kEpsilon),
                () -> assertEquals(90.0, rotated.getDegrees(), kEpsilon));
    }

    @Test
    void testRotateByNonZero() {
        Rotation2d rot = Rotation2d.fromDegrees(90.0);
        rot = rot.plus(Rotation2d.fromDegrees(30.0));

        assertEquals(120.0, rot.getDegrees(), kEpsilon);
    }

    @Test
    void testMinus() {
        Rotation2d rot1 = Rotation2d.fromDegrees(70.0);
        Rotation2d rot2 = Rotation2d.fromDegrees(30.0);

        assertEquals(40.0, rot1.minus(rot2).getDegrees(), kEpsilon);
    }

    @Test
    void testUnaryMinus() {
        Rotation2d rot = Rotation2d.fromDegrees(20.0);

        assertEquals(-20.0, rot.unaryMinus().getDegrees(), kEpsilon);
    }

    @Test
    void testMultiply() {
        Rotation2d rot = Rotation2d.fromDegrees(10.0);

        assertEquals(30.0, rot.times(3.0).getDegrees(), kEpsilon);
        assertEquals(410.0, rot.times(41.0).getDegrees(), kEpsilon);
    }

    @Test
    void testEquality() {
        Rotation2d rot1 = Rotation2d.fromDegrees(43.0);
        Rotation2d rot2 = Rotation2d.fromDegrees(43.0);
        assertEquals(rot1, rot2);

        rot1 = Rotation2d.fromDegrees(-180.0);
        rot2 = Rotation2d.fromDegrees(180.0);
        assertEquals(rot1, rot2);
    }

    @Test
    void testInequality() {
        Rotation2d rot1 = Rotation2d.fromDegrees(43.0);
        Rotation2d rot2 = Rotation2d.fromDegrees(43.5);
        assertNotEquals(rot1, rot2);
    }

    @Test
    void testInterpolate() {
        // 50 + (70 - 50) * 0.5 = 60
        Rotation2d rot1 = Rotation2d.fromDegrees(50);
        Rotation2d rot2 = Rotation2d.fromDegrees(70);
        Rotation2d interpolated = rot1.interpolate(rot2, 0.5);
        assertEquals(60.0, interpolated.getDegrees(), kEpsilon);

        // -160 minus half distance between 170 and -160 (15) = -175
        rot1 = Rotation2d.fromDegrees(170);
        rot2 = Rotation2d.fromDegrees(-160);
        interpolated = rot1.interpolate(rot2, 0.5);
        assertEquals(-175.0, interpolated.getDegrees(), kEpsilon);
    }
}
