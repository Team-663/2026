// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

/**
 * Feeds balls into the shooter by running the agitator and kicker motors.
 *
 * <p>The motors are only driven when two conditions are both true:
 * <ol>
 *   <li>The flywheel is above {@link ShooterConstants#SHOOTER_RPM_MIN_FEED} — prevents
 *       jamming a stationary or barely-moving shooter.</li>
 *   <li>The flywheel is within {@link ShooterConstants#SHOOTER_ERROR_TOLERANCE_RPM} of
 *       its current setpoint — ensures a shot isn't wasted while the wheel is still
 *       spinning up to the desired speed.</li>
 * </ol>
 *
 * <p>A separate command (e.g. {@code shooterSetRPMCmd}) is responsible for setting the
 * flywheel target RPM before this command is scheduled.
 */
public class ShootBalls extends Command {

  private final Shooter m_shooter;

  /** Creates a new ShootBalls command. */
  public ShootBalls(Shooter shooter) {
    m_shooter = shooter;
    // NOTE: Do NOT add Shooter as a requirement here. SpinShooter (which controls
    // the flywheel and hood) runs concurrently with this command. Adding the same
    // requirement would cancel SpinShooter the moment the trigger is pressed.
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Shooter/Feeding", false);
  }

  @Override
  public void execute() {
    boolean readyToFeed = m_shooter.isShooterAtRPM() && m_shooter.isShooterAboveMinRPM();

    SmartDashboard.putBoolean("Shooter/Feeding", readyToFeed);

    if (readyToFeed) {
      m_shooter.setKickerPctOutput(ShooterConstants.KICKER_FEED_SPEED);
      m_shooter.setAgitatorPctOutput(ShooterConstants.AGITATOR_FEED_SPEED);
    } else {
      m_shooter.setKickerPctOutput(0.0);
      m_shooter.setAgitatorPctOutput(0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Always stop the feed motors when the command ends or is interrupted.
    m_shooter.setKickerPctOutput(0.0);
    m_shooter.setAgitatorPctOutput(0.0);
    SmartDashboard.putBoolean("Shooter/Feeding", false);
  }

  @Override
  public boolean isFinished() {
    // Runs until interrupted (e.g. button released).
    return false;
  }
}

