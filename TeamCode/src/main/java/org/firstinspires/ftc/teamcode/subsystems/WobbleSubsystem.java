package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.ma.ftc.lib.command.RobotMap;
import org.ma.ftc.lib.command.SubsystemBase;

public class WobbleSubsystem extends SubsystemBase {

    private static WobbleSubsystem wobbleSubsystem;

    private final DcMotor motor;
    private final Servo servoClaw;
    private final Servo servoLock;

    private boolean isOpen;

    private final static double SERVO_OPEN = 0;
    private final static double SERVO_CLOSE = 0.5;
    private final static double LOCK_OPEN = 0;
    private final static double LOCK_CLOSE = 0.5;

    private WobbleSubsystem() {
        registerSubsystem();

        motor = RobotMap.getInstance().getMap().get(DcMotor.class, "wobble_motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servoClaw = RobotMap.getInstance().getMap().get(Servo.class, "wobble_servo");
        servoLock = RobotMap.getInstance().getMap().get(Servo.class, "wobble_lock");

        isOpen = false;
    }

    public void setMotor(double power) {
        motor.setPower(power);
    }

    public boolean isOpen()
    {
        return isOpen;
    }

    public void openClaw()
    {
        servoClaw.setPosition(SERVO_OPEN);
        isOpen = true;
    }

    public void closeClaw()
    {
        servoClaw.setPosition(SERVO_CLOSE);
        isOpen = false;
    }

    public void openLock()
    {
        servoLock.setPosition(LOCK_OPEN);
        isOpen = true;
    }

    public void closeLock()
    {
        servoLock.setPosition(LOCK_CLOSE);
        isOpen = false;
    }

    public static WobbleSubsystem getInstance() {
        if (wobbleSubsystem == null) {
            wobbleSubsystem = new WobbleSubsystem();
        }
        return wobbleSubsystem;
    }
}