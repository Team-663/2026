// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;

/**
 * Autonomously spins up the shooter and feeds balls for a fixed duration.
 *
 * <p>Runs {@link SpinShooter} and {@link ShootBalls} in parallel, then stops
 * both after {@code durationSeconds} have elapsed. Because ball detection is
 * unavailable, the timer acts as the only termination condition.
 *
 * <p>Usage example:
 * <pre>
 *   new AutoShootBalls(m_shooter, 3000.0, 15.0, 2.5)
 * </pre>
 */
public class AutoShootBalls {

    /**
     * Builds the timed shoot command. Returns a {@link Command} rather than
     * extending {@code Command} directly, because the behaviour is fully
     * expressed by composing existing commands.
     *
     * @param shooter         the Shooter subsystem
     * @param wheelSpeedRPM   desired flywheel speed in RPM
     * @param hoodAngleDeg    desired hood angle in degrees of the 200-tooth gear
     * @param durationSeconds how long to run the shooter and feed motors
     * @return a command that spins the shooter, feeds balls, and ends after the
     *         given number of seconds
     */
    public static Command create(Shooter shooter,
                                 double wheelSpeedRPM,
                                 double hoodAngleDeg,
                                 double durationSeconds) {
        return Commands.parallel(
                new SpinShooter(shooter, wheelSpeedRPM, hoodAngleDeg),
                new ShootBalls(shooter))
            .withTimeout(durationSeconds);
    }
}
