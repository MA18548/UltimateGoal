// Your First Program

class PIDController
{
    private double setpoint = 0;
    private double tolerance = 0;

    private double error = 0;
    private double integral = 0;
    private double derivative = 0, 
    private double previous_error = 0;
    
    private double Kp, Ki, Kd;
    private double Kf = 0;

    public PIDController(double setpoint, double tolerance, double Kp, double Ki, double Kd, double Kf) 
    {
        this.setpoint = setpoint;
        this.tolerance = tolerance;

        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        this.Kf = Kf;
    }

    public PIDController(double setpoint, double, tolerance, double Kp, double Ki, double Kd) 
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
        this.error = input - this.setpoint
        
        return (this.integral = this.integral + this.error) * this.Ki;
    }
    
    public double getD(double input)
    {
        this.error = input - this.setpoint

        this.derivative = this.error - this.previous_error;
        this.previous_error = this.error;

        return this.derivative * this.Kd;
    }

    public double getF()
    {
        return this.Kf;
    }
    
    public double getPI(double input)
    {
        return MathUtil.clamp(getP(input) + getI(input), -1, 1);
    }

    public double getPD(double input)
    {
        return MathUtil.clamp(getP(input) + getD(input), -1, 1);
    }

    public double getPF(double input)
    {
        return MathUtil.clamp(getP(input) + getF(input), -1, 1);
    }

    public double getPID(double input)
    {
        return MathUtil.clamp(getP(input) + getI(input) + getD(input), -1, 1);
    }

    public double getPIDF(double input)
    {
        return MathUtil.clamp(getP(input) + getI(input) + getD(input) + getF(), -1, 1);
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
}