package org.ma.ftc.lib.control;

import java.util.ArrayList;

public class PathBuilder {

    private ArrayList<Movement> path;

    public PathBuilder()
    {
        path = new ArrayList<>();
    }

    public PathBuilder driveTank(double distance, double angle, double waitTime)
    {
        path.add(new Movement(MecanumDrive.MovementType.TANK,distance, angle, 0.1, 3,
                1, 1, waitTime));
        return this;
    }

    public PathBuilder driveTank(double distance, double angle)
    {
        return driveTank(distance, angle, 0);
    }

    public PathBuilder driveForward(double distance, double waitTime)
    {
        path.add(new Movement(MecanumDrive.MovementType.TANK,distance, 0, 0.1, 3,
                1, 1, waitTime));
        return this;
    }

    public PathBuilder driveForward(double distance)
    {
        return driveForward(distance, 0);
    }

    public PathBuilder turnInPlace(double angle, double waitTime)
    {
        path.add(new Movement(MecanumDrive.MovementType.TANK,0, angle, 0.1, 3,
                1, 1, waitTime));
        return this;
    }
    public PathBuilder turnInPlace(double angle)
    {
        return turnInPlace(angle, 0);
    }

    public PathBuilder strafeAngle(double distance, double angle, double waitTime)
    {
        path.add(new Movement(MecanumDrive.MovementType.STRAFE, distance, angle, 0.1, 3,
                1, 1, waitTime));
        return this;
    }

    public PathBuilder strafeAngle(double distance, double angle)
    {
        return strafeAngle(distance, angle, 0);
    }

    public PathBuilder strafeLeft(double distance, double waitTime)
    {
        path.add(new Movement(MecanumDrive.MovementType.STRAFE, distance, 0, 0.1, 3,
                1, 1, waitTime));
        return this;
    }

    public PathBuilder strafeLeft(double distance)
    {
        return strafeLeft(distance, 0);
    }

    public PathBuilder strafeRight(double distance, double waitTime)
    {
        path.add(new Movement(MecanumDrive.MovementType.STRAFE, distance, 180, 0.1, 3,
                1, 1, waitTime));
        return this;
    }

    public PathBuilder strafeRight(double distance)
    {
        return strafeRight(distance, 0);
    }

    public Movement[] getPath()
    {
        Movement[] output = new Movement[path.size()];
        output = path.toArray(output);
        path.clear();
        return output;
    }
}
