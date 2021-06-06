package org.ma.ftc.lib.command; // MA FTC 18548

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A robot subsystem. Subsystems are the basic unit of robot organization in the Command-based
 * framework; they encapsulate low-level hardware objects (motor controllers, sensors, etc) and
 * provide methods through which they can be used by {@link CommandBase}s. Subsystems are used by the
 * {@link CommandScheduler}'s resource management system to ensure multiple robot actions are not
 * "fighting" over the same hardware; Commands that use a subsystem should include that subsystem in
 * their {@link CommandBase#getRequirements()} method, and resources used within a subsystem should
 * generally remain encapsulated and not be shared by other parts of the robot.
 *
 * <p>Subsystems must be registered with the scheduler with the {@link
 * CommandScheduler#registerSubsystem(SubsystemBase...)} method in order for the {@link
 * SubsystemBase#periodic()} method and default commands to be called.
 */
public abstract class SubsystemBase {
    protected static final HardwareMap hardwareMap = RobotMap.getInstance().getMap();
    protected static final Telemetry telemetry = RobotMap.getInstance().getTelemtry();

    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link CommandBase}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    public void periodic() {
    }

    /**
     * Registers this subsystem with the {@link CommandScheduler}, allowing its {@link
     * SubsystemBase#periodic()} method to be called when the scheduler runs.
     */
    protected void registerSubsystem() {
        CommandScheduler.getInstance().registerSubsystem(this);
    }
}
