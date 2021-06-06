package org.ma.ftc.lib.command; // MA FTC 18548

import org.ma.ftc.lib.BooleanSupplier;
import org.ma.ftc.lib.Runnable;

/**
 * This class provides an easy way to link commands to inputs.
 *
 * <p>It is very easy to link a button to a command. For instance, you could link the trigger button
 * of a joystick to a "score" command.
 *
 * <p>It is encouraged that teams write a subclass of Trigger if they want to have something unusual
 * (for instance, if they want to react to the user holding a button while the robot is reading a
 * certain sensor input). For this, they only have to write the {@link Trigger#get()} method to get
 * the full functionality of the Trigger class.
 */

public class Trigger {
    private final BooleanSupplier isActive;

    /**
     * Creates a new trigger with the given condition determining whether it is active.
     *
     * @param isActive returns whether or not the trigger should be active
     */
    public Trigger(BooleanSupplier isActive) {
        this.isActive = isActive;
    }

    /**
     * Returns whether or not the trigger is active.
     *
     * <p>This method will be called repeatedly a command is linked to the Trigger.
     *
     * @return whether or not the trigger condition is active.
     */
    public boolean get() {
        return isActive.getAsBoolean();
    }

    /**
     * Starts the given command whenever the trigger just becomes active.
     *
     * @param command       the command to start
     * @param interruptible whether the command is interruptible
     * @return this trigger, so calls can be chained
     */
    public Trigger whenActive(final CommandBase command, final boolean interruptible) {
        CommandScheduler.getInstance()
                .addButton(
                        new Runnable() {
                            private boolean m_pressedLast = get();

                            @Override
                            public void run() {
                                boolean pressed = get();

                                if (!m_pressedLast && pressed) {
                                    command.schedule(interruptible);
                                }

                                m_pressedLast = pressed;
                            }
                        });

        return this;
    }

    /**
     * Starts the given command whenever the trigger just becomes active. The command is set to be
     * interruptible.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public Trigger whenActive(final CommandBase command) {
        return whenActive(command, true);
    }

    /**
     * Runs the given runnable whenever the trigger just becomes active.
     *
     * @param toRun        the runnable to run
     * @param requirements the required subsystems
     * @return this trigger, so calls can be chained
     */
    public Trigger whenActive(Runnable toRun, SubsystemBase... requirements) {
        return whenActive(new InstantCommand(toRun, requirements));
    }

    /**
     * Constantly starts the given command while the button is held.
     *
     * <p>{@link CommandBase#schedule(boolean)} will be called repeatedly while the trigger is active, and
     * will be canceled when the trigger becomes inactive.
     *
     * @param command       the command to start
     * @param interruptible whether the command is interruptible
     * @return this trigger, so calls can be chained
     */
    public Trigger whileActiveContinuous(final CommandBase command, final boolean interruptible) {
        CommandScheduler.getInstance()
                .addButton(
                        new Runnable() {
                            private boolean m_pressedLast = get();

                            @Override
                            public void run() {
                                boolean pressed = get();

                                if (pressed) {
                                    command.schedule(interruptible);
                                } else if (m_pressedLast) {
                                    command.cancel();
                                }

                                m_pressedLast = pressed;
                            }
                        });
        return this;
    }

    /**
     * Constantly starts the given command while the button is held.
     *
     * <p>{@link CommandBase#schedule(boolean)} will be called repeatedly while the trigger is active, and
     * will be canceled when the trigger becomes inactive. The command is set to be interruptible.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public Trigger whileActiveContinuous(final CommandBase command) {
        return whileActiveContinuous(command, true);
    }

    /**
     * Constantly runs the given runnable while the button is held.
     *
     * @param toRun        the runnable to run
     * @param requirements the required subsystems
     * @return this trigger, so calls can be chained
     */
    public Trigger whileActiveContinuous(Runnable toRun, SubsystemBase... requirements) {
        return whileActiveContinuous(new InstantCommand(toRun, requirements));
    }

    /**
     * Starts the given command when the trigger initially becomes active, and ends it when it becomes
     * inactive, but does not re-start it in-between.
     *
     * @param command       the command to start
     * @param interruptible whether the command is interruptible
     * @return this trigger, so calls can be chained
     */
    public Trigger whileActiveOnce(final CommandBase command, final boolean interruptible) {
        CommandScheduler.getInstance()
                .addButton(
                        new Runnable() {
                            private boolean m_pressedLast = get();

                            @Override
                            public void run() {
                                boolean pressed = get();

                                if (!m_pressedLast && pressed) {
                                    command.schedule(interruptible);
                                } else if (m_pressedLast && !pressed) {
                                    command.cancel();
                                }

                                m_pressedLast = pressed;
                            }
                        });
        return this;
    }

    /**
     * Starts the given command when the trigger initially becomes active, and ends it when it becomes
     * inactive, but does not re-start it in-between. The command is set to be interruptible.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public Trigger whileActiveOnce(final CommandBase command) {
        return whileActiveOnce(command, true);
    }

    /**
     * Starts the command when the trigger becomes inactive.
     *
     * @param command       the command to start
     * @param interruptible whether the command is interruptible
     * @return this trigger, so calls can be chained
     */
    public Trigger whenInactive(final CommandBase command, final boolean interruptible) {
        CommandScheduler.getInstance()
                .addButton(
                        new Runnable() {
                            private boolean m_pressedLast = get();

                            @Override
                            public void run() {
                                boolean pressed = get();

                                if (m_pressedLast && !pressed) {
                                    command.schedule(interruptible);
                                }

                                m_pressedLast = pressed;
                            }
                        });
        return this;
    }

    /**
     * Starts the command when the trigger becomes inactive. The command is set to be interruptible.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public Trigger whenInactive(final CommandBase command) {
        return whenInactive(command, true);
    }

    /**
     * Runs the given runnable when the trigger becomes inactive.
     *
     * @param toRun        the runnable to run
     * @param requirements the required subsystems
     * @return this trigger, so calls can be chained
     */
    public Trigger whenInactive(Runnable toRun, SubsystemBase... requirements) {
        return whenInactive(new InstantCommand(toRun, requirements));
    }

    /**
     * Toggles a command when the trigger becomes active.
     *
     * @param command       the command to toggle
     * @param interruptible whether the command is interruptible
     * @return this trigger, so calls can be chained
     */
    public Trigger toggleWhenActive(final CommandBase command, final boolean interruptible) {
        CommandScheduler.getInstance()
                .addButton(
                        new Runnable() {
                            private boolean m_pressedLast = get();

                            @Override
                            public void run() {
                                boolean pressed = get();

                                if (!m_pressedLast && pressed) {
                                    if (command.isScheduled()) {
                                        command.cancel();
                                    } else {
                                        command.schedule(interruptible);
                                    }
                                }

                                m_pressedLast = pressed;
                            }
                        });
        return this;
    }

    /**
     * Toggles a command when the trigger becomes active. The command is set to be interruptible.
     *
     * @param command the command to toggle
     * @return this trigger, so calls can be chained
     */
    public Trigger toggleWhenActive(final CommandBase command) {
        return toggleWhenActive(command, true);
    }

    /**
     * Cancels a command when the trigger becomes active.
     *
     * @param command the command to cancel
     * @return this trigger, so calls can be chained
     */
    public Trigger cancelWhenActive(final CommandBase command) {
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

    /**
     * Composes this trigger with another trigger, returning a new trigger that is active when both
     * triggers are active.
     *
     * @param trigger the trigger to compose with
     * @return the trigger that is active when both triggers are active
     */
    public Trigger and(final Trigger trigger) {
        return new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return get() && trigger.get();
            }
        });
    }

    /**
     * Composes this trigger with another trigger, returning a new trigger that is active when either
     * trigger is active.
     *
     * @param trigger the trigger to compose with
     * @return the trigger that is active when either trigger is active
     */
    public Trigger or(final Trigger trigger) {
        return new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return get() || trigger.get();
            }
        });
    }

    /**
     * Creates a new trigger that is active when this trigger is inactive, i.e. that acts as the
     * negation of this trigger.
     *
     * @return the negated trigger
     */
    public Trigger negate() {
        return new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !get();
            }
        });
    }
}