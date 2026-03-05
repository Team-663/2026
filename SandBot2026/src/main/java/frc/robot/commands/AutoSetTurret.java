// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;

/**
 * Rotates the turret to a target angle and waits until it arrives, with a
 * timeout to prevent auto routines from stalling if the turret is slightly
 * off target and never fully satisfies the at-setpoint condition.
 *
 * <p>The command:
 * <ol>
 *   <li>Sends the angle to the turret PID ({@code setTurretAngle}).</li>
 *   <li>Waits (polling {@code isTurretAtTarget()}) until the turret settles.</li>
 *   <li>Ends automatically once at target, OR after {@code timeoutSeconds} —
 *       whichever comes first.</li>
 * </ol>
 *
 * <p>Usage example:
 * <pre>
 *   AutoSetTurret.create(m_shooter, -45.0, 2.0)
 * </pre>
 */
public class AutoSetTurret {

    /**
     * Builds the turret-positioning command.
     *
     * @param shooter        the Shooter subsystem
     * @param angleDeg       desired turret angle in degrees (output shaft, 0 = straight ahead).
     *                       Clamped internally to the configured soft limits.
     * @param timeoutSeconds maximum time to wait for the turret to reach the target
     *                       before the command gives up and auto continues
     * @return a command that moves the turret and finishes when on-target or timed out
     */
    public static Command create(Shooter shooter, double angleDeg, double timeoutSeconds) {
        return Commands.sequence(
                // Step 1: kick off the PID — instant, no subsystem requirement
                Commands.runOnce(() -> shooter.setTurretAngle(angleDeg)),
                // Step 2: wait until the PID reports on-target, or until timeout
                Commands.waitUntil(shooter::isTurretAtTarget)
                        .withTimeout(timeoutSeconds))
            .withName("AutoSetTurret_" + angleDeg);
    }
}
