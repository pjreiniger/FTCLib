// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.spline;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.spline.SplineParameterizer.MalformedSplineException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;

class CubicHermiteSplineTest {
    private static final double kMaxDx = 0.127;
    private static final double kMaxDy = 0.00127;
    private static final double kMaxDtheta = 0.0872;

    private void run(Pose2d a, List<Translation2d> waypoints, Pose2d b) {
        // Start the timer.
        // var start = System.nanoTime();

        // Generate and parameterize the spline.
        Spline.ControlVector[] controlVectors =
                SplineHelper.getCubicControlVectorsFromWaypoints(
                        a, waypoints.toArray(new Translation2d[0]), b);
        CubicHermiteSpline[] splines =
                SplineHelper.getCubicSplinesFromControlVectors(
                        controlVectors[0], waypoints.toArray(new Translation2d[0]), controlVectors[1]);

        ArrayList<PoseWithCurvature> poses = new ArrayList<PoseWithCurvature>();

        poses.add(splines[0].getPoint(0.0));

        for (CubicHermiteSpline spline : splines) {
            poses.addAll(SplineParameterizer.parameterize(spline));
        }

        // End the timer.
        // var end = System.nanoTime();

        // Calculate the duration (used when benchmarking)
        // var durationMicroseconds = (end - start) / 1000.0;

        for (int i = 0; i < poses.size() - 1; i++) {
            PoseWithCurvature p0 = poses.get(i);
            PoseWithCurvature p1 = poses.get(i + 1);

            // Make sure the twist is under the tolerance defined by the Spline class.
            Twist2d twist = p0.poseMeters.log(p1.poseMeters);
            assertAll(
                    () -> assertTrue(Math.abs(twist.dx) < kMaxDx),
                    () -> assertTrue(Math.abs(twist.dy) < kMaxDy),
                    () -> assertTrue(Math.abs(twist.dtheta) < kMaxDtheta));
        }

        // Check first point
        assertAll(
                () -> assertEquals(a.getX(), poses.get(0).poseMeters.getX(), 1E-9),
                () -> assertEquals(a.getY(), poses.get(0).poseMeters.getY(), 1E-9),
                () ->
                        assertEquals(
                                a.getRotation().getRadians(),
                                poses.get(0).poseMeters.getRotation().getRadians(),
                                1E-9));

        // Check interior waypoints
        boolean interiorsGood = true;
        for (Translation2d waypoint : waypoints) {
            boolean found = false;
            for (PoseWithCurvature state : poses) {
                if (waypoint.getDistance(state.poseMeters.getTranslation()) == 0) {
                    found = true;
                }
            }
            interiorsGood &= found;
        }

        assertTrue(interiorsGood);

        // Check last point
        assertAll(
                () -> assertEquals(b.getX(), poses.get(poses.size() - 1).poseMeters.getX(), 1E-9),
                () -> assertEquals(b.getY(), poses.get(poses.size() - 1).poseMeters.getY(), 1E-9),
                () ->
                        assertEquals(
                                b.getRotation().getRadians(),
                                poses.get(poses.size() - 1).poseMeters.getRotation().getRadians(),
                                1E-9));
    }

    @Test
    void testStraightLine() {
        run(new Pose2d(), new ArrayList<>(), new Pose2d(3, 0, new Rotation2d()));
    }

    @Test
    void testSCurve() {
        Pose2d start = new Pose2d(0, 0, Rotation2d.fromDegrees(90.0));
        ArrayList<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(new Translation2d(1, 1));
        waypoints.add(new Translation2d(2, -1));
        Pose2d end = new Pose2d(3, 0, Rotation2d.fromDegrees(90.0));

        run(start, waypoints, end);
    }

    @Test
    void testOneInterior() {
        Pose2d start = new Pose2d(0, 0, Rotation2d.fromDegrees(0.0));
        ArrayList<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(new Translation2d(2.0, 0.0));
        Pose2d end = new Pose2d(4, 0, Rotation2d.fromDegrees(0.0));

        run(start, waypoints, end);
    }

    @Test
    void testWindyPath() {
        final Pose2d start = new Pose2d(0, 0, Rotation2d.fromDegrees(0.0));
        final ArrayList<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(new Translation2d(0.5, 0.5));
        waypoints.add(new Translation2d(0.5, 0.5));
        waypoints.add(new Translation2d(1.0, 0.0));
        waypoints.add(new Translation2d(1.5, 0.5));
        waypoints.add(new Translation2d(2.0, 0.0));
        waypoints.add(new Translation2d(2.5, 0.5));
        final Pose2d end = new Pose2d(3.0, 0.0, Rotation2d.fromDegrees(0.0));

        run(start, waypoints, end);
    }

    @Test
    void testMalformed() {
        assertThrows(
                MalformedSplineException.class,
                () ->
                        run(
                                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                                new ArrayList<>(),
                                new Pose2d(1, 0, Rotation2d.fromDegrees(180))));
        assertThrows(
                MalformedSplineException.class,
                () ->
                        run(
                                new Pose2d(10, 10, Rotation2d.fromDegrees(90)),
                                Arrays.asList(new Translation2d(10, 10.5)),
                                new Pose2d(10, 11, Rotation2d.fromDegrees(-90))));
    }
}
