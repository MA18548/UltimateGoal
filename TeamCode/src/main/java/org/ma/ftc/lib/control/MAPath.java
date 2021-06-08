/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ma.ftc.lib.control;

/**
 * @author MA 18548
 */

import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.ma.ftc.lib.command.CommandBase;
import org.ma.ftc.lib.command.RobotMap;

public class MAPath extends CommandBase {
    /**
     * Creates a new MAPath.
     */

    private final MecanumDriveSubsystem chassis;
    private final ElapsedTime timer;
    public static int stage = 0;
    public static int pathnum = 0;
    private double lastTimeOnTarget;
    private final double waitTime;

    private boolean isFinished;

    private Movement[][] paths = {Path.testPath, Path.testPath2};

    public MAPath(double waitTime) {
        this.waitTime = waitTime;
        chassis = MecanumDriveSubsystem.getInstance();
        timer = RobotMap.getInstance().getRuntime();
        addRequirements(chassis);
        Path.mainPath = Path.testPath;
    }

    @Override
    public String toString() {
        return "MAPath:" +
                "\tstage - " + stage +
                ",\tpathnum - " + lastTimeOnTarget +
                ",\tcurrent path - " + chassis.getCurrentMovement();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
//    chassis.rampRate(RobotConstants.Ramp_Rate_Auto);
//    chassis.setidilmodeBrake(false);

        stage = 0;
        isFinished = false;
        chassis.reset();
        chassis.setMovementBySetpoint(Path.mainPath[0]);
        lastTimeOnTarget = timer.seconds();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotMap.getInstance().getTelemtry().addData("stage", stage);

        RobotMap.getInstance().getTelemtry().addData("distance", chassis.getDistance());
        RobotMap.getInstance().getTelemtry().addData("angle", chassis.getAngle());

        RobotMap.getInstance().getTelemtry().addData("angle done", chassis.isAngleDone());
        RobotMap.getInstance().getTelemtry().addData("distance done", chassis.isDistanceDone());

        RobotMap.getInstance().getTelemtry().addData("angle error", chassis.getAngleError());
        RobotMap.getInstance().getTelemtry().addData("distance error", chassis.getDistanceError());

        RobotMap.getInstance().getTelemtry().addData("motor power", chassis.leftFrontMotor.getPower());


        RobotMap.getInstance().getTelemtry().addData("distance", chassis.getDistancePID());
        chassis.runMovement();

        //&& chassis.isAngleDone()
        if (chassis.isDistanceDone() && chassis.isAngleDone() && timer.seconds() - lastTimeOnTarget >= Path.mainPath[stage].getWaitTime())
        {
            if (stage < Path.mainPath.length - 1) {
                stage++;
                chassis.setMovementBySetpoint(Path.mainPath[stage]);
                lastTimeOnTarget = timer.seconds();
            }
            else
            {
                isFinished = true;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            //      chassis.setidilmodeBrake(true);
        } else {
            pathnum++;
        }
        chassis.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
//        if (!(Math.abs(chassis.getDistanceError()) < Path.mainPath[Path.mainPath.length - 1].getDistanceTolerance() * chassis.getTicksPerMinute()
//                && Math.abs(chassis.getAngleError()) < Path.mainPath[Path.mainPath.length - 1].getAngleTolerance()
//                && stage == Path.mainPath.length)) {
//            lastTimeOnTarget = RobotMap.getInstance().getRuntime().seconds();
//        }
//        return Math.abs(chassis.getDistanceError()) < Path.mainPath[Path.mainPath.length - 1].getDistanceTolerance() * chassis.getTicksPerMinute()
//                && Math.abs(chassis.getAngleError()) < Path.mainPath[Path.mainPath.length - 1].getAngleTolerance() && stage == Path.mainPath.length
//                && RobotMap.getInstance().getRuntime().seconds() - lastTimeOnTarget > waitTime;
    }
}