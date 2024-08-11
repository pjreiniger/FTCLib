// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.controller.wpilibcontroller;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.util.MathUtil;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class RamseteControllerTest {
    private static final double kTolerance = 1 / 12.0;
    private static final double kAngularTolerance = Math.toRadians(2);

    @Test
    void testReachesReference() {
        final RamseteController controller = new RamseteController(2.0, 0.7);
        Pose2d robotPose = new Pose2d(2.7, 23.0, Rotation2d.fromDegrees(0.0));

        final List<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(new Pose2d(2.75, 22.521, new Rotation2d(0)));
        waypoints.add(new Pose2d(24.73, 19.68, new Rotation2d(5.846)));
        TrajectoryConfig config = new TrajectoryConfig(8.8, 0.1);
        final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);

        final double kDt = 0.02;
        final double totalTime = trajectory.getTotalTimeSeconds();
        for (int i = 0; i < (totalTime / kDt); ++i) {
            Trajectory.State state = trajectory.sample(kDt * i);

            ChassisSpeeds output = controller.calculate(robotPose, state);
            robotPose =
                    robotPose.exp(
                            new Twist2d(output.vxMetersPerSecond * kDt, 0, output.omegaRadiansPerSecond * kDt));
        }

        final List<Trajectory.State> states = trajectory.getStates();
        final Pose2d endPose = states.get(states.size() - 1).poseMeters;

        // Java lambdas require local variables referenced from a lambda expression
        // must be final or effectively final.
        final Pose2d finalRobotPose = robotPose;
        assertAll(
                () -> assertEquals(endPose.getX(), finalRobotPose.getX(), kTolerance),
                () -> assertEquals(endPose.getY(), finalRobotPose.getY(), kTolerance),
                () ->
                        assertEquals(
                                0.0,
                                MathUtil.angleModulus(
                                        endPose.getRotation().getRadians() - finalRobotPose.getRotation().getRadians()),
                                kAngularTolerance));
    }
}
