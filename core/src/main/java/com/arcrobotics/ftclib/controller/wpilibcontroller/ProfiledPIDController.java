// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arcrobotics.ftclib.controller.wpilibcontroller;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid profile. Users should
 * call reset() when they first start running the controller to avoid unwanted behavior.
 */
public class ProfiledPIDController {
    private static int instances;

    private PIDController m_controller;
    private double m_minimumInput;
    private double m_maximumInput;

    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile m_profile;
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and Kd.
     *
     * @param Kp The proportional coefficient.
     * @param Ki The integral coefficient.
     * @param Kd The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     */
    public ProfiledPIDController(
            double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints) {
        this(Kp, Ki, Kd, constraints, 0.02);
    }

    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and Kd.
     *
     * @param Kp The proportional coefficient.
     * @param Ki The integral coefficient.
     * @param Kd The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     * @param period The period between controller updates in seconds. The default is 0.02 seconds.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     * @throws IllegalArgumentException if period &lt;= 0
     */
    @SuppressWarnings("this-escape")
    public ProfiledPIDController(
            double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints, double period) {
        m_controller = new PIDController(Kp, Ki, Kd);
        m_constraints = constraints;
        m_profile = new TrapezoidProfile(m_constraints);
        instances++;
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>Sets the proportional, integral, and differential coefficients.
     *
     * @param Kp The proportional coefficient. Must be &gt;= 0.
     * @param Ki The integral coefficient. Must be &gt;= 0.
     * @param Kd The differential coefficient. Must be &gt;= 0.
     */
    public void setPID(double Kp, double Ki, double Kd) {
        m_controller.setPID(Kp, Ki, Kd);
    }

    /**
     * Sets the proportional coefficient of the PID controller gain.
     *
     * @param Kp The proportional coefficient. Must be &gt;= 0.
     */
    public void setP(double Kp) {
        m_controller.setP(Kp);
    }

    /**
     * Sets the integral coefficient of the PID controller gain.
     *
     * @param Ki The integral coefficient. Must be &gt;= 0.
     */
    public void setI(double Ki) {
        m_controller.setI(Ki);
    }

    /**
     * Sets the differential coefficient of the PID controller gain.
     *
     * @param Kd The differential coefficient. Must be &gt;= 0.
     */
    public void setD(double Kd) {
        m_controller.setD(Kd);
    }

    /**
     * Gets the proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return m_controller.getP();
    }

    /**
     * Gets the integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return m_controller.getI();
    }

    /**
     * Gets the differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return m_controller.getD();
    }

    /**
     * Gets the period of this controller.
     *
     * @return The period of the controller.
     */
    public double getPeriod() {
        return m_controller.getPeriod();
    }

    /**
     * Returns the position tolerance of this controller.
     *
     * @return the position tolerance of the controller.
     */
    public double getPositionTolerance() {
        return m_controller.getTolerance()[0];
    }

    /**
     * Returns the velocity tolerance of this controller.
     *
     * @return the velocity tolerance of the controller.
     */
    public double getVelocityTolerance() {
        return m_controller.getTolerance()[1];
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal state.
     */
    public void setGoal(TrapezoidProfile.State goal) {
        m_goal = goal;
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal position.
     */
    public void setGoal(double goal) {
        m_goal = new TrapezoidProfile.State(goal, 0);
    }

    /**
     * Gets the goal for the ProfiledPIDController.
     *
     * @return The goal.
     */
    public TrapezoidProfile.State getGoal() {
        return m_goal;
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return True if the error is within the tolerance of the error.
     */
    public boolean atGoal() {
        return atSetpoint() && m_goal.equals(m_setpoint);
    }

    /**
     * Set velocity and acceleration constraints for goal.
     *
     * @param constraints Velocity and acceleration constraints for goal.
     */
    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        m_constraints = constraints;
        m_profile = new TrapezoidProfile(m_constraints);
    }

    /**
     * Get the velocity and acceleration constraints for this controller.
     *
     * @return Velocity and acceleration constraints.
     */
    public TrapezoidProfile.Constraints getConstraints() {
        return m_constraints;
    }

    /**
     * Returns the current setpoint of the ProfiledPIDController.
     *
     * @return The current setpoint.
     */
    public TrapezoidProfile.State getSetpoint() {
        return m_setpoint;
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return True if the error is within the tolerance of the error.
     */
    public boolean atSetpoint() {
        return m_controller.atSetPoint();
    }

    /**
     * Sets the minimum and maximum values for the integrator.
     *
     * <p>When the cap is reached, the integrator value is added to the controller output rather than
     * the integrator value times the integral gain.
     *
     * @param minimumIntegral The minimum value of the integrator.
     * @param maximumIntegral The maximum value of the integrator.
     */
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        m_controller.setIntegrationBounds(minimumIntegral, maximumIntegral);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        m_controller.setTolerance(positionTolerance, velocityTolerance);
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    public double getPositionError() {
        return m_controller.getPositionError();
    }

    /**
     * Returns the change in error per second.
     *
     * @return The change in error per second.
     */
    public double getVelocityError() {
        return m_controller.getVelocityError();
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The controller's next output.
     */
    public double calculate(double measurement) {
        m_setpoint = m_profile.calculate(getPeriod(), m_setpoint, m_goal);
        return m_controller.calculate(measurement, m_setpoint.position);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal The new goal of the controller.
     * @return The controller's next output.
     */
    public double calculate(double measurement, TrapezoidProfile.State goal) {
        setGoal(goal);
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PIDController.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal The new goal of the controller.
     * @return The controller's next output.
     */
    public double calculate(double measurement, double goal) {
        setGoal(goal);
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal The new goal of the controller.
     * @param constraints Velocity and acceleration constraints for goal.
     * @return The controller's next output.
     */
    public double calculate(
            double measurement, TrapezoidProfile.State goal, TrapezoidProfile.Constraints constraints) {
        setConstraints(constraints);
        return calculate(measurement, goal);
    }

    /** Reset the previous error, the integral term, and disable the controller. */
    public void reset() {
        m_controller.reset();
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measurement The current measured State of the system.
     */
    public void reset(TrapezoidProfile.State measurement) {
        m_controller.reset();
        m_setpoint = measurement;
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system.
     * @param measuredVelocity The current measured velocity of the system.
     */
    public void reset(double measuredPosition, double measuredVelocity) {
        reset(new TrapezoidProfile.State(measuredPosition, measuredVelocity));
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system. The velocity is assumed to
     *     be zero.
     */
    public void reset(double measuredPosition) {
        reset(measuredPosition, 0.0);
    }
}
