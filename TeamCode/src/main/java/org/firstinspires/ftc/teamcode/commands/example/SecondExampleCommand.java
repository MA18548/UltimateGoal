//package org.firstinspires.ftc.teamcode;
//
//import org.ma.ftc.lib.command.CommandBase;
//import org.ma.ftc.lib.command.RobotMap;
//
//public class SecondExampleCommand extends CommandBase {
//
//    private ExampleSubsystem exampleSubsystem;
//    private double startTime;
//
//
//    public SecondExampleCommand()
//    {
//        exampleSubsystem = ExampleSubsystem.getInstance();
//
//        addRequirements(exampleSubsystem);
//    }
//
//
//    public void init()
//    {
//        startTime = RobotMap.getInstance().getRuntime().seconds();
//    }
//
//    public void execute()
//    {
//        exampleSubsystem.setMotor(1);
//    }
//
//    public void end(boolean interrupted)
//    {
//        exampleSubsystem.setMotor(0);
//    }
//
//    public boolean isFinished()
//    {
//        return (RobotMap.getInstance().getRuntime().seconds() - startTime) >= 5;
//    }
//}
