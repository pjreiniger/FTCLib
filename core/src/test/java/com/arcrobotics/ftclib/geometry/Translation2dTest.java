// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.geometry;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import java.util.Arrays;
import org.junit.jupiter.api.Test;

class Translation2dTest {
    private static final double kEpsilon = 1E-9;

    @Test
    void testSum() {
        Translation2d one = new Translation2d(1.0, 3.0);
        Translation2d two = new Translation2d(2.0, 5.0);

        Translation2d sum = one.plus(two);

        assertAll(
                () -> assertEquals(3.0, sum.getX(), kEpsilon),
                () -> assertEquals(8.0, sum.getY(), kEpsilon));
    }

    @Test
    void testDifference() {
        Translation2d one = new Translation2d(1.0, 3.0);
        Translation2d two = new Translation2d(2.0, 5.0);

        Translation2d difference = one.minus(two);

        assertAll(
                () -> assertEquals(-1.0, difference.getX(), kEpsilon),
                () -> assertEquals(-2.0, difference.getY(), kEpsilon));
    }

    @Test
    void testRotateBy() {
        Translation2d another = new Translation2d(3.0, 0.0);
        Translation2d rotated = another.rotateBy(Rotation2d.fromDegrees(90.0));

        assertAll(
                () -> assertEquals(0.0, rotated.getX(), kEpsilon),
                () -> assertEquals(3.0, rotated.getY(), kEpsilon));
    }

    @Test
    void testMultiplication() {
        Translation2d original = new Translation2d(3.0, 5.0);
        Translation2d mult = original.times(3);

        assertAll(
                () -> assertEquals(9.0, mult.getX(), kEpsilon),
                () -> assertEquals(15.0, mult.getY(), kEpsilon));
    }

    @Test
    void testDivision() {
        Translation2d original = new Translation2d(3.0, 5.0);
        Translation2d div = original.div(2);

        assertAll(
                () -> assertEquals(1.5, div.getX(), kEpsilon),
                () -> assertEquals(2.5, div.getY(), kEpsilon));
    }

    @Test
    void testNorm() {
        Translation2d one = new Translation2d(3.0, 5.0);
        assertEquals(Math.hypot(3.0, 5.0), one.getNorm(), kEpsilon);
    }

    @Test
    void testDistance() {
        Translation2d one = new Translation2d(1, 1);
        Translation2d two = new Translation2d(6, 6);
        assertEquals(5.0 * Math.sqrt(2.0), one.getDistance(two), kEpsilon);
    }

    @Test
    void testUnaryMinus() {
        Translation2d original = new Translation2d(-4.5, 7);
        Translation2d inverted = original.unaryMinus();

        assertAll(
                () -> assertEquals(4.5, inverted.getX(), kEpsilon),
                () -> assertEquals(-7.0, inverted.getY(), kEpsilon));
    }

    @Test
    void testEquality() {
        Translation2d one = new Translation2d(9, 5.5);
        Translation2d two = new Translation2d(9, 5.5);
        assertEquals(one, two);
    }

    @Test
    void testInequality() {
        Translation2d one = new Translation2d(9, 5.5);
        Translation2d two = new Translation2d(9, 5.7);
        assertNotEquals(one, two);
    }

    @Test
    void testPolarConstructor() {
        Translation2d one = new Translation2d(Math.sqrt(2), Rotation2d.fromDegrees(45.0));
        Translation2d two = new Translation2d(2, Rotation2d.fromDegrees(60.0));
        assertAll(
                () -> assertEquals(1.0, one.getX(), kEpsilon),
                () -> assertEquals(1.0, one.getY(), kEpsilon),
                () -> assertEquals(1.0, two.getX(), kEpsilon),
                () -> assertEquals(Math.sqrt(3.0), two.getY(), kEpsilon));
    }

    @Test
    void testNearest() {
        Translation2d origin = new Translation2d();

        // each translationX is X units away from the origin at a random angle.
        Translation2d translation1 = new Translation2d(1, Rotation2d.fromDegrees(45));
        Translation2d translation2 = new Translation2d(2, Rotation2d.fromDegrees(90));
        Translation2d translation3 = new Translation2d(3, Rotation2d.fromDegrees(135));
        Translation2d translation4 = new Translation2d(4, Rotation2d.fromDegrees(180));
        Translation2d translation5 = new Translation2d(5, Rotation2d.fromDegrees(270));

        assertEquals(
                origin.nearest(Arrays.asList(translation5, translation3, translation4)), translation3);
        assertEquals(
                origin.nearest(Arrays.asList(translation1, translation2, translation3)), translation1);
        assertEquals(
                origin.nearest(Arrays.asList(translation4, translation2, translation3)), translation2);
    }
}
