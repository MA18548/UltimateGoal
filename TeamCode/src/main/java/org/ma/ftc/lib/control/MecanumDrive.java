package org.ma.ftc.lib.control;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.ma.ftc.lib.MathUtil;
import org.ma.ftc.lib.geometry.Vector2D;

public class MecanumDrive {
    public enum MovementType
    {
        TANK,
        STRAFE
    };

    private final DcMotor leftFront;
    private final DcMotor leftBack;
    private final DcMotor rightFront;
    private final DcMotor rightBack;

    public MecanumDrive(DcMotor leftFront, DcMotor leftBack,
                        DcMotor rightFront, DcMotor rightBack) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
    }

    public void driveCartesian(double ySpeed, double xSpeed, double rotation) {
        driveCartesian(ySpeed, xSpeed, rotation, 0f);
    }

    public void driveCartesian(double ySpeed, double xSpeed, double rotation, double gyroAngle) {
        ySpeed = MathUtil.clamp(ySpeed, 1f, -1f);
        xSpeed = MathUtil.clamp(xSpeed, 1f, -1f);

        Vector2D input = new Vector2D(xSpeed, ySpeed);
        input.rotate(-gyroAngle);

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = input.getX() + input.getY() + rotation;
        wheelSpeeds[2] = -input.getX() + input.getY() - rotation;
        wheelSpeeds[1] = -input.getX() + input.getY() + rotation;
        wheelSpeeds[3] = input.getX() + input.getY() - rotation;
        MathUtil.normalize(wheelSpeeds);

        setMotors(wheelSpeeds[0], wheelSpeeds[1],
                wheelSpeeds[2], wheelSpeeds[3]);
    }

    public void drivePolar(double mag, double angle, double rotation) {
        driveCartesian(mag * Math.sin(angle),
                mag * Math.cos(angle),
                rotation);
    }

    public void setMotors(double leftF, double leftB, double rightF, double rightB) {
        leftFront.setPower(leftF);
        leftBack.setPower(leftB);
        rightFront.setPower(rightF);
        rightBack.setPower(rightB);
    }

    public void stop() {
        tankDrive(0, 0);
    }

    public void tankDrive(double leftPower, double rightPower) {
        setMotors(leftPower, leftPower,
                rightPower, rightPower);
    }

    public void arcadeDrive(double angle, double distance) {
        distance *= -1;
        double w = (100 - Math.abs(angle * 100)) * (distance) + distance * 100;
        double v = (100 - Math.abs(distance * 100)) * (angle) + angle * 100;

        tankDrive((-(v + w) / 200), ((v - w) / 200));
    }
}

