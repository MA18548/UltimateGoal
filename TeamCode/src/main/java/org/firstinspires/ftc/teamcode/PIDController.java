// Your First Program

class PIDController
{
    double setpoint, tolerance;

    double error;
    double integral;
    double derivative, previous_error;
    
    double Kp, Ki, Kd, Kf;

    public PIDController(double setpoint, double tolerance, double Kp, double Ki, double Kd, double Kf) 
    {
        self.setpoint = setpoint;
        self.tolerance = tolerance;

        self.Kp = Kp;
        self.Ki = Ki;
        self.Kd = Kd;

        self.Kf = Kf;
    }

    public PIDController(double setpoint, double Kp, double Ki, double Kd) 
    {
        self.setpoint = setpoint;
        
        self.Kp = Kp;
        self.Ki = Ki;
        self.Kd = Kd;
    }

    public PIDController(double Kp, double Ki, double Kd) 
    {        
        self.Kp = Kp;
        self.Ki = Ki;
        self.Kd = Kd;
    }

    public double getP(double input)
    {
        return (self.error = input - self.setpoint) * self.Kp;
    }
    
    public double getI()
    {
        return (self.integral = self.integral + self.error) * self.Ki;
    }
    
    public double getD()
    {
        self.derivative = self.error - self.previous_error;
        self.previous_error = self.error;

        return self.derivative * self.Kd;
    }

    public double getF()
    {
        return self.Kf;
    }
    
    public double getPI()
    {
        return MathUtil.clamp(getP() + getI(), -1, 1);
    }

    public double getPD()
    {
        return MathUtil.clamp(getP() + getD(), -1, 1);
    }

    public double getPF()
    {
        return MathUtil.clamp(getP() + getF(), -1, 1);
    }

    public double getPID()
    {
        return MathUtil.clamp(getP() + getI() + getD(), -1, 1);
    }

    public double getPIDF()
    {
        return MathUtil.clamp(getP() + getI() + getD() + getF(), -1, 1);
    }

    public boolean atSetpoint()
    {
        return Math.abs(self.error) <= self.tolerance; 
    }

    public void setSetpoint(double setpoint)
    {
        self.setpoint = setpoint;
    }
    
    public void setTolerance(double tolerance)
    {
        self.tolerance = tolerance;
    }

    public void setKp(double Kp)
    {
        self.Kp = Kp;
    }

    public void setKi(double Ki)
    {
        self.Ki = Ki;
    }

    public void setKd(double Kd)
    {
        self.Kd = Kd;
    }

    public void setKf(double Kf)
    {
        self.Kf = Kf;
    }

    public void setAll(double Kp, double Ki, double Kd)
    {
        self.Kp = Kp;
        self.Ki = Ki;
        self.Kd = Kd;    
    }

    public double getSetpoint() 
    {
        return self.setpoint;
    }

    public double getTolerance() 
    {
        return self.tolerance;
    }

    public double getKp() 
    {
        return self.Kp;
    }

    public double getKi() 
    {
        return self.Ki;
    }

    public double getKd() 
    {
        return self.Kd;
    }

    public double getKf() 
    {
        return self.Kf;
    }
}