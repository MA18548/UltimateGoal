package org.ma.ftc.lib.control;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.ma.ftc.lib.control.MecanumDrive.MovementType;

public class Movement {

    private MovementType movementType;
    private double distanceSetpoint;
    private double angleSetpoint;
    private double distanceTolerance;
    private double angleTolerance;
    private double distanceSpeed;
    private double angleSpeed;
    private double waitTime;

    public Movement(MovementType movementType, double distanceSetpoint, double angleSetpoint,
                    double distanceTolerance, double robotAngleTolerance, double distanceSpeed, double robotAngleSpeed, double waitTime) {
        this.setMovementType(movementType);
        this.setDistanceSetpoint(distanceSetpoint);
        this.setAngleSetpoint(angleSetpoint);
        this.setDistanceTolerance(distanceTolerance);
        this.setAngleTolerance(robotAngleTolerance);
        this.setDistanceSpeed(distanceSpeed);
        this.setAngleSpeed(robotAngleSpeed);
        this.waitTime = waitTime;
    }

    public Movement(MovementType movementType, double distanceSetpoint, double angleSetpoint,
                    double distanceTolerance, double robotAngleTolerance, double distanceSpeed, double robotAngleSpeed) {
        this.setMovementType(movementType);
        this.setDistanceSetpoint(distanceSetpoint);
        this.setAngleSetpoint(angleSetpoint);
        this.setDistanceTolerance(distanceTolerance);
        this.setAngleTolerance(robotAngleTolerance);
        this.setDistanceSpeed(distanceSpeed);
        this.setAngleSpeed(robotAngleSpeed);
        this.waitTime = 0;
    }

    @Override
    public String toString() {
        return
                "distanceSetpoint=" + distanceSetpoint +
                ",\n\tangleSetpoint=" + angleSetpoint +
                ",\n\tdistanceTolerance=" + distanceTolerance +
                ",\n\tangleTolerance=" + angleTolerance +
                ",\n\tdistanceSpeed=" + distanceSpeed +
                ",\n\tangleSpeed=" + angleSpeed;
    }

    public void setMovementType(MovementType movementType)
    {
        this.movementType = movementType;
    }

    public MovementType getMovementType()
    {
        return this.movementType;
    }

    public double getDistanceSetpoint() {
        return distanceSetpoint;
    }

    public void setDistanceSetpoint(double distanceSetpoint) {
        this.distanceSetpoint = distanceSetpoint * MecanumDriveSubsystem.getInstance().TICKS_PER_METER;
    }

    public double getAngleSetpoint() {
        return angleSetpoint;
    }

    public void setAngleSetpoint(double robotAngleSetpoint) {
        this.angleSetpoint = robotAngleSetpoint;
    }

    public double getDistanceTolerance() {
        return distanceTolerance;
    }

    public void setDistanceTolerance(double distanceTolerance) {
        this.distanceTolerance = distanceTolerance;
    }

    public double getAngleTolerance() {
        return angleTolerance;
    }

    public void setAngleTolerance(double angleTolerance) {
        this.angleTolerance = angleTolerance;
    }

    public double getDistanceSpeed() {
        return distanceSpeed;
    }

    public void setDistanceSpeed(double distanceSpeed) {
        this.distanceSpeed = distanceSpeed;
    }

    public double getAngleSpeed() {
        return angleSpeed;
    }

    public void setAngleSpeed(double angleSpeed) {
        this.angleSpeed = angleSpeed;
    }

    public double getWaitTime()
    {
        return waitTime;
    }

    public boolean isStrafe()
    {
        return movementType == MovementType.STRAFE;
    }

    public boolean isTank()
    {
        return movementType == MovementType.TANK;
    }

    public boolean isMainCrossMotors()
    {
        return (angleSetpoint >= 0 && angleSetpoint <= 90) || (angleSetpoint >= 180 && angleSetpoint <= 270);
    }

    public boolean isSecondaryCrossMotors()
    {
        return !isMainCrossMotors();
    }

    public int getDirection()
    {
        int out = 1;
        if (isStrafe())
        {
            out = getIsPositive() ? 1 : -1;
        }

        return out;
    }

    public boolean getIsPositive()
    {
        return this.angleSetpoint >= 0 && this.angleSetpoint < 180;
    }
}
