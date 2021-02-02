package org.firstinspires.ftc.teamcode;

import java.util.function.BooleanSupplier;

import com.qualcomm.robotcore.hardware.Gamepad;


public class Gamepad_EX
{
    private Gamepad gamepad;

    public Trigger A;
    public Trigger B;
    public Trigger X;
    public Trigger Y;

    public Trigger LEFT_BUMPER;
    public Trigger RIGHT_BUMPER;

    public Trigger DPAD_UP;
    public Trigger DPAD_DOWN;
    public Trigger DPAD_LEFT;
    public Trigger DPAD_RIGHT;

    public Trigger START;
    public Trigger BACK;

    public Trigger LEFT_STICK;
    public Trigger RIGHT_STICK;

    public Joystick_EX(Gamepad gamepad)
    {
        this.gamepad = gamepad;

        this.A = new Trigger(() -> this.gamepad.a);
        this.B = new Trigger(() -> this.gamepad.b);
        this.X = new Trigger(() -> this.gamepad.x);
        this.Y = new Trigger(() -> this.gamepad.y);
    
        this.LEFT_BUMPER = new Trigger(() -> this.gamepad.left_bumper);
        this.RIGHT_BUMPER = new Trigger(() -> this.gamepad.right_bumper);
    
        this.DPAD_UP = new Trigger(() -> this.gamepad.dpad_up);
        this.DPAD_DOWN = new Trigger(() -> this.gamepad.dpad_down);
        this.DPAD_LEFT = new Trigger(() -> this.gamepad.dpad_left);
        this.DPAD_RIGHT = new Trigger(() -> this.gamepad.dpad_right);
    
        this.START = new Trigger(() -> this.gamepad.start);
        this.BACK = new Trigger(() -> this.gamepad.back);
    
        this.LEFT_STICK = new Trigger(() -> this.gamepad.left_stick_button);
        this.RIGHT_STICK = new Trigger(() -> this.gamepad.right_stick_button);
    }

    public double getLeftY()
    {
        return gamepad.left_stick_y;
    }

    public double getLeftX()
    {
        return gamepad.left_stick_x;
    }

    public double getRightY()
    {
        return gamepad.right_stick_y;
    }

    public double getRightX()
    {
        return gamepad.right_stick_x;
    }


}