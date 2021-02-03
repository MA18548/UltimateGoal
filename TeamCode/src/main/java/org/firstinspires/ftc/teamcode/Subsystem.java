package org.firstinspires.ftc.teamcode;

public abstract class Subsystem {
    void periodic() {}

    void registerSubsystem()
    {
        CommandScheduler.getInstance().registerSubsystem(this);
    }
}
