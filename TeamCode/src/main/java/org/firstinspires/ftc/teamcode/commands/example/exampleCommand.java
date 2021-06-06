//package org.firstinspires.ftc.teamcode;
//
//import org.ma.ftc.lib.command.CommandBase;
//
//public class exampleCommand extends CommandBase {
//
//    private ExampleSubsystem exampleSubsystem;
//    private double power;
//
//    public exampleCommand(double power)
//    {
//        exampleSubsystem = ExampleSubsystem.getInstance();
//        this.power = power;
//
//        addRequirements(exampleSubsystem);
//    }
//
//    public void init()
//    {
//
//    }
//
//    public void execute()
//    {
//        exampleSubsystem.setMotor(this.power);
//    }
//
//    public void end(boolean interrupted)
//    {
//        exampleSubsystem.setMotor(0);
//    }
//}
