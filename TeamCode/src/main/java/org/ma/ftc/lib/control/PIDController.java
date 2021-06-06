package org.ma.ftc.lib.control; // MA FTC 18548

import com.qualcomm.robotcore.util.ElapsedTime;

import org.ma.ftc.lib.MathUtil;
import org.ma.ftc.lib.command.RobotMap;

/**
 * Implements a PID control loop.
 */
public class PIDController {
    private final ElapsedTime timer = RobotMap.getInstance().getRuntime();

    private double setpoint = 0d;
    private double tolerance = 0d;

    private double error = 0d;
    private double integral = 0d;

    private double derivative = 0d;
    private double previous_error = 0d;

    private double dt;
    private double previous_time;

    private double Kp, Ki, Kd;
    private double Kf = 0d;

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd and a default setpoint and tolerance.
     *
     * @param setpoint  The period between controller updates in seconds
     * @param tolerance The period between controller updates in seconds..
     * @param Kp        The proportional coefficient.
     * @param Ki        The integral coefficient.
     * @param Kd        The derivative coefficient.
     * @param Kf        the feed forward coefficient.
     */
    public PIDController(double setpoint, double tolerance, double Kp, double Ki, double Kd, double Kf) {
        this.previous_time = timer.seconds();

        this.setpoint = setpoint;
        this.tolerance = tolerance;

        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        this.Kf = Kf;
    }

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd and a default setpoint and tolerance.
     *
     * @param setpoint  The period between controller updates in seconds
     * @param tolerance The period between controller updates in seconds..
     * @param Kp        The proportional coefficient.
     * @param Ki        The integral coefficient.
     * @param Kd        The derivative coefficient.
     */
    public PIDController(double setpoint, double tolerance, double Kp, double Ki, double Kd) {
        this.previous_time = timer.seconds();
        this.setpoint = setpoint;
        this.tolerance = tolerance;

        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd and a default setpoint.
     *
     * @param setpoint The control loop's setpoint.
     * @param Kp       The proportional coefficient.
     * @param Ki       The integral coefficient.
     * @param Kd       The derivative coefficient.
     */
    public PIDController(double setpoint, double Kp, double Ki, double Kd) {
        this.setpoint = setpoint;
        this.previous_time = timer.seconds();

        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd.
     *
     * @param Kp The proportional coefficient.
     * @param Ki The integral coefficient.
     * @param Kd The derivative coefficient.
     */
    public PIDController(double Kp, double Ki, double Kd) {
        this.previous_time = timer.seconds();

        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void reset()
    {
        this.previous_time = timer.seconds();
        this.integral = 0;
        this.previous_error = this.error;
    }

    @Override
    public String toString() {
        return
                "\n\tsetpoint - " + setpoint +
                ",\n\ttolerance - " + tolerance +
                ",\n\terror - " + error;
                // ",\n\tintegral - " + integral +
                // ",\n\tderivative - " + derivative +
                // ",\n\tkp - " + Kp +
                // ",\n\tki - " + Ki +
                // ",\n\tkd - " + Kd +
                // ",\n\tkf - " + Kf;
    }

    /**
     * Returns the next output of the PID controller using only proportional gain.
     *
     * @param input the current input of the control loop
     * @return result of the control loop
     */
    public double getP(double input) {
        return (this.error = input - this.setpoint) * this.Kp;
    }

    /**
     * Returns the next output of the PID controller using only integral gain.
     *
     * @param input the current input of the control loop
     * @return result of the control loop
     */
    public double getI(double input) {
        this.error = input - this.setpoint;

        this.dt = timer.milliseconds() - this.previous_time;
        this.previous_time = timer.milliseconds();

        this.integral = this.integral + this.error * this.dt;

        return this.integral * this.Ki;
    }

    /**
     * Returns the next output of the PID controller using only derivative gain.
     *
     * @param input the current input of the control loop
     * @return result of the control loop
     */
    public double getD(double input) {
        this.error = input - this.setpoint;

        this.derivative = this.error - this.previous_error;
        this.dt = timer.milliseconds() - this.previous_time;

        this.previous_error = this.error;
        this.previous_time = timer.milliseconds();

        return (this.derivative / this.dt) * this.Kd;
    }

    /**
     * Returns the feed forward of the system.
     *
     * @return feed forward
     */
    public double getF() {
        return this.Kf;
    }

    /**
     * Returns the next output of the PID controller using both proportional and integral gain.
     *
     * @param input the current input of the control loop
     * @return result of the control loop
     */
    public double getPI(double input) {
        return MathUtil.clamp(getP(input) + getI(input), -1, 1);
    }

    /**
     * Returns the next output of the PID controller using both proportional and derivative gain.
     *
     * @param input the current input of the control loop
     * @return result of the control loop
     */
    public double getPD(double input) { return MathUtil.clamp(getP(input) + getD(input), -1, 1); }

    /**
     * Returns the next output of the PID controller using both proportional and feed forward gain.
     *
     * @param input the current input of the control loop
     * @return result of the control loop
     */
    public double getPF(double input) {
        return MathUtil.clamp(getP(input) + getF(), -1, 1);
    }

    /**
     * Returns the next output of the PID controller using proportional, integral and derivative gain.
     *
     * @param input the current input of the control loop
     * @return result of the control loop
     */
    public double getPID(double input) {
        this.error = this.setpoint - input;
        this.dt = timer.milliseconds() - this.previous_time;

        this.integral = this.integral + this.error * this.dt;
        this.derivative = (this.error - this.previous_error) / this.dt;

        this.previous_time = timer.milliseconds();
        this.previous_error = this.error;
        return MathUtil.clamp(this.error * this.Kp + this.integral * this.Ki + this.derivative * this.Kd, 1, -1);
    }

    /**
     * Returns the next output of the PID controller using both proportional, integral, derivative and feed forward gain.
     *
     * @param input the current input of the control loop
     * @return result of the control loop
     */
    public double getPIDF(double input) {
        return MathUtil.clamp(getPID(input) + getKf(), 1, -1);
    }

    /**
     * Returns true if the error is within the tolerance of the setpoint.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetpoint() {
        return Math.abs(this.error) <= this.tolerance;
    }

    /**
     * Sets the setpoint for the PIDController.
     *
     * @param setpoint The desired setpoint.
     */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param tolerance Position error which is tolerable.
     */
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    /**
     * Sets the Proportional coefficient of the PID controller gain.
     *
     * @param Kp proportional coefficient
     */
    public void setKp(double Kp) {
        this.Kp = Kp;
    }

    /**
     * Sets the Integral coefficient of the PID controller gain.
     *
     * @param Ki integral coefficient
     */
    public void setKi(double Ki) {
        this.Ki = Ki;
    }

    /**
     * Sets the Differential coefficient of the PID controller gain.
     *
     * @param Kd differential coefficient
     */
    public void setKd(double Kd) {
        this.Kd = Kd;
    }

    /**
     * Sets the Feed Forward coefficient of the PID controller gain.
     *
     * @param Kf feed forward coefficient
     */
    public void setKf(double Kf) {
        this.Kf = Kf;
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>Set the proportional, integral, and differential coefficients.
     *
     * @param Kp The proportional coefficient.
     * @param Ki The integral coefficient.
     * @param Kd The derivative coefficient.
     */
    public void setAll(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    /**
     * Returns the current setpoint of the PIDController.
     *
     * @return The current setpoint.
     */
    public double getSetpoint() {
        return this.setpoint;
    }

    /**
     * Returns the current tolerance of the PIDController.
     *
     * @return The current tolerannce.
     */
    public double getTolerance() {
        return this.tolerance;
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    public double getError() {
        return this.error;
    }

    /**
     * Get the Proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getKp() {
        return this.Kp;
    }

    /**
     * Get the Integral coefficient.
     *
     * @return integral coefficient
     */
    public double getKi() {
        return this.Ki;
    }

    /**
     * Get the Differential coefficient.
     *
     * @return differential coefficient
     */
    public double getKd() {
        return this.Kd;
    }

    /**
     * Get the Feed Forward coefficient.
     *
     * @return feed forward coefficient
     */
    public double getKf() {
        return this.Kf;
    }
}