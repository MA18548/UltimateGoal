/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.autonomous.StraightPathAutonomousCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveTrainCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.RingStackCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.ShooterVisionCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeMotorCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterPIDCommand;
import org.firstinspires.ftc.teamcode.commands.wobble.WobbleMotorCommand;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.ma.ftc.lib.Runnable;
import org.ma.ftc.lib.command.CommandScheduler;
import org.ma.ftc.lib.command.MAGamepad;
import org.ma.ftc.lib.command.RobotMap;
import org.ma.ftc.lib.control.MAPath;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Main", group = "Iterative Opmode")
public class MainOpmode extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();


    private MAGamepad driveGamepad;
    private MAGamepad systemGamepad;

    /*
     * Code to run ONCE when the driver hits INIT
     */


    public void init() {
        driveGamepad = new MAGamepad(gamepad1);
        systemGamepad = new MAGamepad(gamepad2);
        RobotMap.getInstance().init(hardwareMap, telemetry, runtime, driveGamepad, systemGamepad);

        CameraSubsystem.cameraSubsystem = null;
        CommandScheduler.getInstance().clearButtons();
//        CommandScheduler.getInstance().clearSubsystems();
        CommandScheduler.getInstance().clearCommands();
        CameraSubsystem.getInstance();
        MecanumDriveSubsystem.getInstance().reset();

        systemGamepad.X.whileActiveOnce(new IntakeMotorCommand(0.69));
        systemGamepad.B.whileActiveOnce(new IntakeMotorCommand(-0.69));
        systemGamepad.A.whileActiveOnce(new ShooterPIDCommand(-2600));
        systemGamepad.Y.whenActive(new Runnable() {
            @Override
            public void run() {
                ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
                shooterSubsystem.setMotor(.3);
            }
        }).whenInactive(new Runnable() {
            @Override
            public void run() {
                ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
                shooterSubsystem.setMotor(.3);
            }
        });

        systemGamepad.DPAD_RIGHT.whileActiveContinuous(new RingStackCommand());

        systemGamepad.DPAD_DOWN.whenActive(new Runnable() {
            @Override
            public void run() {
                ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
                if (shooterSubsystem.isReset()) {
                    shooterSubsystem.loadToFlywheel();
                } else {
                    shooterSubsystem.resetServo();
                }
            }
        });

        systemGamepad.DPAD_UP.whenActive(new Runnable() {
            @Override
            public void run() {
                WobbleSubsystem wobbleSubsystem = WobbleSubsystem.getInstance();
                if (wobbleSubsystem.isOpen()) {
                    wobbleSubsystem.closeClaw();
                    wobbleSubsystem.closeLock();
                } else {
                    wobbleSubsystem.openLock();
                    wobbleSubsystem.openClaw();
                }
            }
        });

        systemGamepad.RIGHT_BUMPER.whenActive(new Runnable() {
            @Override
            public void run() {
                ShooterSubsystem.getInstance().SETPOINT += 25;
                ShooterSubsystem.getInstance().setSetpoint(ShooterSubsystem.getInstance().SETPOINT);
            }
        });

        systemGamepad.DPAD_LEFT.whenActive(new Runnable() {
            @Override
            public void run() {
                ShooterSubsystem.getInstance().VISION = !ShooterSubsystem.getInstance().VISION;
            }
        });

        systemGamepad.LEFT_BUMPER.whenActive(new Runnable() {
            @Override
            public void run() {
                ShooterSubsystem.getInstance().SETPOINT -= 25;
                ShooterSubsystem.getInstance().setSetpoint(ShooterSubsystem.getInstance().SETPOINT);
            }
        });

        driveGamepad.DPAD_DOWN.whileActiveOnce(new MAPath(0.7));
        driveGamepad.Y.whileActiveOnce(new StraightPathAutonomousCommand());
        driveGamepad.A.whileActiveOnce(new ShooterVisionCommand());

        CommandScheduler.getInstance().setDefaultCommand(new DriveTrainCommand());
        CommandScheduler.getInstance().setDefaultCommand(new WobbleMotorCommand());

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        CameraSubsystem.getInstance().targetsUltimateGoal.activate();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        CameraSubsystem.getInstance().periodic();
        telemetry.update();
        telemetry.clearAll();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        CameraSubsystem.getInstance().targetsUltimateGoal.deactivate();
    }

}
