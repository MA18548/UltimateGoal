package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController
{
    private ElapsedTime timer;

    private double setpoint = 0d;
    private double tolerance = 0d;

    private double error = 0d;
    private double integral = 0d;

    private double derivative = 0d; 
    private double previous_error = 0d;
    
    private double dt;
    private double previous_time = 0d;

    private double Kp, Ki, Kd;
    private double Kf = 0d;

    public PIDController(double setpoint, double tolerance, double Kp, double Ki, double Kd, double Kf) 
    {
        this.timer = new ElapsedTime();

        this.setpoint = setpoint;
        this.tolerance = tolerance;

        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        this.Kf = Kf;
    }

    public PIDController(double setpoint, double tolerance, double Kp, double Ki, double Kd) 
    {
        this.setpoint = setpoint;
        
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public PIDController(double setpoint, double Kp, double Ki, double Kd) 
    {
        this.setpoint = setpoint;
        
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public PIDController(double Kp, double Ki, double Kd) 
    {        
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double getP(double input)
    {
        return (this.error = input - this.setpoint) * this.Kp;
    }
    
    public double getI(double input)
    {
        this.error = input - this.setpoint;
        
        this.dt = timer.milliseconds() - this.previous_time;
        this.previous_time = timer.milliseconds();

        return (this.integral = this.integral + this.error) * this.Ki * this.dt;
    }
    
    public double getD(double input)
    {
        this.error = input - this.setpoint;

        this.derivative = this.error - this.previous_error;
        this.dt = timer.milliseconds() - this.previous_time;

        this.previous_error = this.error;
        this.previous_time = timer.milliseconds();

        return (this.derivative / this.dt) * this.Kd;
    }

    public double getF()
    {
        return this.Kf;
    }
    
    public double getPI(double input)
    {
        
        return clamp(getP(input) + getI(input), -1, 1);
    }

    public double getPD(double input)
    {
        return clamp(getP(input) + getD(input), -1, 1);
    }

    public double getPF(double input)
    {
        return clamp(getP(input) + getF(), -1, 1);
    }

    public double getPID(double input)
    {
        return clamp(getP(input) + getI(input) + getD(input), -1, 1);
    }

    public double getPIDF(double input)
    {
        return clamp(getP(input) + getI(input) + getD(input) + getF(), -1, 1);
    }

    public boolean atSetpoint()
    {
        return Math.abs(this.error) <= this.tolerance; 
    }

    public void setSetpoint(double setpoint)
    {
        this.setpoint = setpoint;
    }
    
    public void setTolerance(double tolerance)
    {
        this.tolerance = tolerance;
    }

    public void setKp(double Kp)
    {
        this.Kp = Kp;
    }

    public void setKi(double Ki)
    {
        this.Ki = Ki;
    }

    public void setKd(double Kd)
    {
        this.Kd = Kd;
    }

    public void setKf(double Kf)
    {
        this.Kf = Kf;
    }

    public void setAll(double Kp, double Ki, double Kd)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;    
    }

    public double getSetpoint() 
    {
        return this.setpoint;
    }

    public double getTolerance() 
    {
        return this.tolerance;
    }

    public double getKp() 
    {
        return this.Kp;
    }

    public double getKi() 
    {
        return this.Ki;
    }

    public double getKd() 
    {
        return this.Kd;
    }

    public double getKf() 
    {
        return this.Kf;
    }

    public static double clamp(double value, double low, double high) 
    {
        return Math.max(low, Math.min(value, high));
    }
}