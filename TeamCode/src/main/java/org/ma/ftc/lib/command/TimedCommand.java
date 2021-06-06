package org.ma.ftc.lib.command;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.ma.ftc.lib.command.CommandBase;
import org.ma.ftc.lib.command.CommandScheduler;
import org.ma.ftc.lib.command.RobotMap;

public class TimedCommand extends CommandBase {
    private final CommandBase command;
    private final ElapsedTime timer;

    private final double duration;
    private double startTime;

    public TimedCommand(CommandBase command, double duration) {
        this.command = command;
        this.timer = RobotMap.getInstance().getRuntime();
        this.duration = duration;
    }

    @Override
    public void initialize() {
        this.startTime = timer.milliseconds();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished() || (timer.milliseconds() - startTime > duration);
    }
}
