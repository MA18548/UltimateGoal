package org.firstinspires.ftc.teamcode;

import java.util.function.BooleanSupplier;

import jdk.jfr.internal.cmd.Command;

public class Trigger 
{

    private BooleanSupplier isActive;

    public Trigger(BooleanSupplier isActive)
    {
        this.isActive = isActive;
    }

    public boolean get()
    {
        return isActive.getAsBoolean();
    }

    public Trigger whenActive(CommandBase command)
    {
        CommandScheduler.getInstance()
        .addButton(
            new Runnable() {
                private boolean m_pressedLast = get();

                @Override
                public void run() {
                boolean pressed = get();

                if (!m_pressedLast && pressed) {
                    command.schedule();
                }

                m_pressedLast = pressed;
                }
            });

        return this;
    }

    public Trigger whileActiveContinuous(CommandBase command)
    {
        CommandScheduler.getInstance()
        .addButton(
            new Runnable() {
                private boolean m_pressedLast = get();

                @Override
                public void run() {
                boolean pressed = get();

                if (pressed) {
                    command.schedule();
                } else if (m_pressedLast) {
                    command.cancel();
                }

                m_pressedLast = pressed;
                }
            });
        return this;
    }

    public Trigger whileActiveOnce(CommandBase command)
    {
        CommandScheduler.getInstance()
        .addButton(
            new Runnable() {
            private boolean m_pressedLast = get();

            @Override
            public void run() {
                boolean pressed = get();

                if (!m_pressedLast && pressed) {
                command.schedule();
                } else if (m_pressedLast && !pressed) {
                command.cancel();
                }

                m_pressedLast = pressed;
            }
            });
        return this;
    }

    public Trigger whenInactive(CommandBase command) 
    {
        CommandScheduler.getInstance()
            .addButton(
                new Runnable() {
                  private boolean m_pressedLast = get();
    
                  @Override
                  public void run() {
                    boolean pressed = get();
    
                    if (m_pressedLast && !pressed) {
                      command.schedule();
                    }
    
                    m_pressedLast = pressed;
                  }
                });
        return this;
    }

    public Trigger cancelWhenActive(CommandBase command)
    {
        CommandScheduler.getInstance()
            .addButton(
                new Runnable() {
                    private boolean m_pressedLast = get();

                    @Override
                    public void run() {
                    boolean pressed = get();

                    if (!m_pressedLast && pressed) {
                        command.cancel();
                    }

                    m_pressedLast = pressed;
                    }
                });
        return this;
    }
    
    public Trigger and(Trigger trigger) 
    {
        return new Trigger(() -> get() && trigger.get());
    }


    public Trigger or(Trigger trigger) 
    {
        return new Trigger(() -> get() || trigger.get());
    }


    public Trigger negate() 
    {
        return new Trigger(() -> !get());
    }
}