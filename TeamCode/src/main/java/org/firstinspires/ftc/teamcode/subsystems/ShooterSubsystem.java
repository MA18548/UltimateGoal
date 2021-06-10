package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.ma.ftc.lib.command.RobotMap;
import org.ma.ftc.lib.command.SubsystemBase;
import org.ma.ftc.lib.control.PIDController;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem shooterSubsystem = null;

    private final DcMotorEx flyWheel;
    private final Servo servo;
    private final DistanceSensor irSensor;

    private final PIDController velocityPID;

    private final double MAX_SERVO_POS = 0.55; // TODO
    private final double MIN_SERVO_POS = 0; // TODO
    private boolean servoReset;

    public int SETPOINT = -2095;
    public boolean VISION = true;

    private ShooterSubsystem() {
        registerSubsystem();

        flyWheel = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        velocityPID = new PIDController(0, 130, 0.003, 0, 0., 0.4);

        flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo = hardwareMap.get(Servo.class, "shooter_servo");
        irSensor = hardwareMap.get(DistanceSensor.class, "shooter_ir");
    }

    public double getPID()
    {
        return velocityPID.getPIDF(getRPM());
    }

    public void setSetpoint(double setpoint)
    {
        velocityPID.setKf((1/6000d) * setpoint);
        velocityPID.setSetpoint(setpoint);
    }

    public double getRPMByDistance(double distance)
    {
        return distance > 2650 ? -2095 : -1 * (0.0015 * (Math.pow(distance, 2)) - 6.5231 * distance + 8920.4);
    }

    public void setMotor(double power) {
        flyWheel.setPower(power);
    }

    public void stopMotor()
    {
        flyWheel.setPower(0);
    }

    public void runRPM()
    {
        setMotor(getPID());
    }

    public double getRPM()
    {
        return toRPM(getSpeed());
    }

    public double getSpeed() {
        return flyWheel.getVelocity(AngleUnit.DEGREES) * 3;
    }

    public double getMotorPower()
    {
        return flyWheel.getPower();
    }

    public boolean atSetpoint() {
        return velocityPID.atSetpoint();
    }

    public double getRPMFromDistance(double distance)
    {
        return 0;
    }
    
    public void loadToFlywheel() {
        servo.setPosition(MIN_SERVO_POS);
        servoReset = false;
    }

    public boolean isReset()
    {
        return servoReset;
    }

    public double getServo()
    {
        return servo.getPosition();
    }

    public boolean isServoReset()
    {
        return Math.abs(servo.getPosition() - MAX_SERVO_POS) <= 0.05;
    }

    public boolean isServoMax()
    {
        return Math.abs(servo.getPosition() - MIN_SERVO_POS) <= 0.05;
    }

    public void resetServo() {
        servo.setPosition(MAX_SERVO_POS);
        servoReset = true;
    }

    public boolean getIR()
    {
        return irSensor.getDistance(DistanceUnit.CM) < 5.50;
    }

    public static double toRPM(double radiansPerSecond)
    {
        return radiansPerSecond * ((2*Math.PI)/60d) * 10;
    }

    public static double toRadiansPerSecond(double rpm)
    {
        return (60d/(2*Math.PI)) * rpm;
    }

    public static ShooterSubsystem getInstance() {
        if (shooterSubsystem == null) {
            shooterSubsystem = new ShooterSubsystem();
        }
        return shooterSubsystem;
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter KF: ", velocityPID.getKf());
        telemetry.addData("Shooter Setpoint: ", SETPOINT);

    }
}
