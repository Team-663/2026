// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;

/**
 * Autonomously runs the agitator at a fixed speed for a fixed duration.
 *
 * <p>Drives the agitator motor at {@code speedPct} (range -1.0 to 1.0) for
 * {@code durationSeconds}, then stops it. Intended for use as a PathPlanner
 * NamedCommand to jostle balls before or during a shooting sequence.
 *
 * <p>Usage example:
 * <pre>
 *   AutoRunAgitator.create(m_shooter, 0.5, 1.0)  // 50% for 1 second
 * </pre>
 */
public class AutoRunAgitator {

    /**
     * Builds the timed agitator command.
     *
     * @param shooter         the Shooter subsystem
     * @param speedPct        agitator motor output (-1.0 to 1.0)
     * @param durationSeconds how long to run the agitator
     * @return a command that runs the agitator at the given speed and stops after
     *         the given number of seconds
     */
    public static Command create(Shooter shooter, double speedPct, double durationSeconds) {
        return Commands.run(() -> shooter.setAgitatorPctOutput(speedPct), shooter)
                .withTimeout(durationSeconds)
                .finallyDo(() -> shooter.setAgitatorPctOutput(0.0))
                .withName("AutoRunAgitator");
    }
}
