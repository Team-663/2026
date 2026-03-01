// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Sets the shooter flywheel to a target RPM and the hood to a desired angle (degrees
 * of the 200-tooth gear) for as long as the command is active.
 *
 * <p>On end (or interrupt), the flywheel is stopped and the hood returns to 0°.
 */
public class SpinShooter extends Command {

    private final Shooter m_shooter;
    private final double m_targetRPM;
    private final double m_hoodAngleDeg;

    /**
     * @param shooter      the Shooter subsystem
     * @param targetRPM    desired flywheel speed in RPM
     * @param hoodAngleDeg desired hood angle in degrees of the 200-tooth gear
     */
    public SpinShooter(Shooter shooter, double targetRPM, double hoodAngleDeg) {
        m_shooter = shooter;
        m_targetRPM = targetRPM;
        m_hoodAngleDeg = hoodAngleDeg;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        m_shooter.shooterSetRPM(m_targetRPM);
        m_shooter.setHoodAngle(m_hoodAngleDeg);
    }

    @Override
    public void execute() {
        // Setpoints are held by the motor controllers; nothing extra needed each loop.
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.shooterStop();
        m_shooter.setHoodAngle(0.0);
    }

    @Override
    public boolean isFinished() {
        // Runs until interrupted (e.g. button released).
        return false;
    }
}
