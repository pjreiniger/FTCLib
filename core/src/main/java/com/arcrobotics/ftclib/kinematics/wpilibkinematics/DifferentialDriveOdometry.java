// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.kinematics.wpilibkinematics;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

/**
 * Class for differential drive odometry. Odometry allows you to track the robot's position on the
 * field over the course of a match using readings from 2 encoders and a gyroscope.
 *
 * <p>Teams can use odometry during the autonomous period for complex tasks like path following.
 * Furthermore, odometry can be used for latency compensation when using computer-vision systems.
 *
 * <p>It is important that you reset your encoders to zero before using this class. Any subsequent
 * pose resets also require the encoders to be reset to zero.
 */
public class DifferentialDriveOdometry extends Odometry<DifferentialDriveWheelPositions> {
    /**
     * Constructs a DifferentialDriveOdometry object.
     *
     * @param gyroAngle The angle reported by the gyroscope.
     * @param leftDistanceMeters The distance traveled by the left encoder.
     * @param rightDistanceMeters The distance traveled by the right encoder.
     * @param initialPoseMeters The starting position of the robot on the field.
     */
    public DifferentialDriveOdometry(
            Rotation2d gyroAngle,
            double leftDistanceMeters,
            double rightDistanceMeters,
            Pose2d initialPoseMeters) {
        super(
                new DifferentialDriveKinematics(1),
                gyroAngle,
                new DifferentialDriveWheelPositions(leftDistanceMeters, rightDistanceMeters),
                initialPoseMeters);
    }

    /**
     * Constructs a DifferentialDriveOdometry object.
     *
     * @param gyroAngle The angle reported by the gyroscope.
     * @param leftDistanceMeters The distance traveled by the left encoder.
     * @param rightDistanceMeters The distance traveled by the right encoder.
     */
    public DifferentialDriveOdometry(
            Rotation2d gyroAngle, double leftDistanceMeters, double rightDistanceMeters) {
        this(gyroAngle, leftDistanceMeters, rightDistanceMeters, new Pose2d());
    }

    /**
     * Resets the robot's position on the field.
     *
     * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
     * automatically takes care of offsetting the gyro angle.
     *
     * @param gyroAngle The angle reported by the gyroscope.
     * @param leftDistanceMeters The distance traveled by the left encoder.
     * @param rightDistanceMeters The distance traveled by the right encoder.
     * @param poseMeters The position on the field that your robot is at.
     */
    public void resetPosition(
            Rotation2d gyroAngle,
            double leftDistanceMeters,
            double rightDistanceMeters,
            Pose2d poseMeters) {
        super.resetPosition(
                gyroAngle,
                new DifferentialDriveWheelPositions(leftDistanceMeters, rightDistanceMeters),
                poseMeters);
    }

    /**
     * Updates the robot position on the field using distance measurements from encoders. This method
     * is more numerically accurate than using velocities to integrate the pose and is also
     * advantageous for teams that are using lower CPR encoders.
     *
     * @param gyroAngle The angle reported by the gyroscope.
     * @param leftDistanceMeters The distance traveled by the left encoder.
     * @param rightDistanceMeters The distance traveled by the right encoder.
     * @return The new pose of the robot.
     */
    public Pose2d update(
            Rotation2d gyroAngle, double leftDistanceMeters, double rightDistanceMeters) {
        return super.update(
                gyroAngle, new DifferentialDriveWheelPositions(leftDistanceMeters, rightDistanceMeters));
    }
}
