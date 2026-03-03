// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase
{
    // Intake has 2x SparkMax motors: 1 for arm, 1 for roller
    private final SparkMax m_armMotor = new SparkMax(CANConstants.INTAKE_ARM_CAN_ID, MotorType.kBrushless);
    private final SparkMax m_rollerMotor = new SparkMax(CANConstants.INTAKE_ROLLER_CAN_ID, MotorType.kBrushed);

    // Through Bore Encoder — absolute (duty-cycle) signal gives position within one revolution.
    private final DutyCycleEncoder m_absEncoder =
        new DutyCycleEncoder(IntakeConstants.INTAKE_ARM_ABS_DIO_PORT);

    // WPILib software PID — runs every periodic() call (~50 Hz)
    private final PIDController m_armPID = new PIDController(
        IntakeConstants.INTAKE_ARM_PID_P,
        IntakeConstants.INTAKE_ARM_PID_I,
        IntakeConstants.INTAKE_ARM_PID_D);

    private boolean m_armPIDEnabled = false;
    private double m_targetArmPosition = 0.0;

    // Multi-turn tracking: we track full revolutions by detecting when the
    // absolute encoder wraps around 0↔1. This is more reliable than the index
    // pulse because it works at any speed and doesn't require a separate counter.
    private int m_turnCount = 0;
    private double m_prevAbsPosition = 0.0;
    private boolean m_firstRead = true;

    // Runtime zero offset — starts from the constant but can be overwritten at
    // any time by pressing the rezero button without re-deploying code.
    private double m_absZeroOffset = IntakeConstants.INTAKE_ARM_ABS_ZERO_OFFSET;

    /** Creates a new Intake. */
    public Intake()
    {
        IntakeInitMotors();
        m_armPID.setTolerance(IntakeConstants.INTAKE_ARM_ERROR_TOLERANCE);
    }

    private void IntakeInitMotors()
    {
        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.inverted(false);
        armConfig.idleMode(IdleMode.kBrake);
        // No closed-loop or encoder config on the SparkMax — PID runs in software
        m_armMotor.configure(armConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.inverted(true);
        rollerConfig.idleMode(IdleMode.kCoast);
        m_rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void periodic()
    {
        // Update multi-turn tracking every cycle
        updateTurnCount();

        // Run the software PID loop if enabled
        if (m_armPIDEnabled)
        {
            double currentPosition = getArmPosition();
            double output = m_armPID.calculate(currentPosition, m_targetArmPosition);
            // Direction-aware output clamp
            if (output >= 0)
                output = MathUtil.clamp(output, 0, IntakeConstants.INTAKE_ARM_MAX_OUTPUT_FORWARD);
            else
                output = MathUtil.clamp(output, -IntakeConstants.INTAKE_ARM_MAX_OUTPUT_REVERSE, 0);
            output = applySoftLimits(output, currentPosition);
            m_armMotor.set(output);
        }

        updateSmartDashboard();
    }

    /**
     * Enforces software soft limits using the corrected position.
     * If the arm is at or past a limit, blocks output in the direction
     * that would move it further out of range.
     *
     * @param output   Requested motor output (-1.0 to 1.0)
     * @param position Current corrected arm position (rotations)
     * @return         Clamped output (0.0 if at a limit in the offending direction)
     */
    private double applySoftLimits(double output, double position)
    {
        if (position >= IntakeConstants.INTAKE_ARM_SOFT_LIMIT_MAX && output > 0) return 0.0;
        if (position <= IntakeConstants.INTAKE_ARM_SOFT_LIMIT_MIN && output < 0) return 0.0;
        return output;
    }

    /**
     * Re-zeros the arm encoder at the current position.
     * Captures the current raw absolute reading as the new zero offset,
     * then resets the turn counter so the arm reads 0.0 from this point.
     * The new offset is NOT saved to Constants — re-deploy to make it permanent.
     */
    private void rezeroArm()
    {
        m_absZeroOffset = m_absEncoder.get();
        m_turnCount = 0;
        m_firstRead = true; // force updateTurnCount() to re-seed on next cycle
        m_armPIDEnabled = false;
        SmartDashboard.putNumber("Intake/ArmZeroOffset", m_absZeroOffset);
    }

    public Command rezeroArmCmd()
    {
        return runOnce(this::rezeroArm).withName("ArmRezero");
    }

    /**
     * Returns the zero-offset-corrected absolute encoder reading, wrapped into
     * (-0.5, 0.5] so that the home position (0.0) is at the CENTER of the safe
     * range. This keeps the hardware rollover boundary as far from zero as
     * possible, preventing false turn-count increments due to encoder noise near
     * the zero point.
     */
    private double getCorrectedAbsPosition()
    {
        double raw = m_absEncoder.get();
        double corrected = raw - m_absZeroOffset;
        // Wrap into (-0.5, 0.5]
        corrected = corrected % 1.0;
        if (corrected > 0.5)  corrected -= 1.0;
        if (corrected <= -0.5) corrected += 1.0;
        return corrected;
    }

    /**
     * Detects wrap-around of the absolute encoder (0→1 or 1→0) to count
     * full revolutions. Called every periodic() cycle.
     */
    private void updateTurnCount()
    {
        double currentAbs = getCorrectedAbsPosition();

        if (m_firstRead)
        {
            m_prevAbsPosition = currentAbs;
            m_firstRead = false;
            return;
        }

        double delta = currentAbs - m_prevAbsPosition;

        // If the absolute value jumped by more than 0.5 in one cycle,
        // the encoder wrapped around (crossed the 0/1 boundary).
        // At 50 Hz, a genuine movement of >0.5 rotation per cycle = 30 RPM
        // at the output shaft, which is far beyond normal arm speed.
        if (delta > 0.5)
        {
            // Wrapped backwards: e.g., 0.05 → 0.95 means we crossed 0 going negative
            m_turnCount--;
        }
        else if (delta < -0.5)
        {
            // Wrapped forwards: e.g., 0.95 → 0.05 means we crossed 1.0 going positive
            m_turnCount++;
        }

        m_prevAbsPosition = currentAbs;
    }

    /**
     * Returns the arm position in output-shaft rotations, multi-turn capable.
     * Combines the absolute within-turn position with the revolution count.
     * e.g., 2.75 = 2 full turns + 0.75 of the current turn.
     */
    public double getArmPosition()
    {
        return m_turnCount + getCorrectedAbsPosition();
    }

    /** Returns the offset-corrected absolute encoder reading (0.0–1.0, within one revolution). */
    public double getArmAbsolutePosition()
    {
        return getCorrectedAbsPosition();
    }

    private void setArmPosition(double rotationsTarget)
    {
        m_targetArmPosition = rotationsTarget;
        m_armPID.setSetpoint(rotationsTarget);
        m_armPIDEnabled = true;
    }

    public boolean isArmAtTarget()
    {
        return m_armPID.atSetpoint();
    }

    public Command setArmPositionCmd(double rotationsTarget)
    {
        return runOnce(() -> setArmPosition(rotationsTarget))
            .withName("ArmPosition_" + rotationsTarget);
    }

    private void setArmPctOutput(double output)
    {
        // Disable the software PID so it doesn't fight manual control
        m_armPIDEnabled = false;
        // Direction-aware speed cap
        if (output >= 0)
            output = MathUtil.clamp(output, 0, IntakeConstants.INTAKE_ARM_MAX_OUTPUT_FORWARD);
        else
            output = MathUtil.clamp(output, -IntakeConstants.INTAKE_ARM_MAX_OUTPUT_REVERSE, 0);
        // Software soft limits
        output = applySoftLimits(output, getArmPosition());
        m_armMotor.set(output);
    }

    public Command setArmPctOutputCmd(DoubleSupplier output)
    {
        return run(() -> setArmPctOutput(output.getAsDouble()))
            .withName("ArmByXbox");
    }

    public Command armOffCmd()
    {
        return runOnce(() -> setArmPctOutput(0.0))
            .withName("ArmOff");
    }

    private void setRollerPctOutput(double output)
    {
        m_rollerMotor.set(output);
    }

    public Command setRollerPctOutputCmd(DoubleSupplier output)
    {
        return run(() -> setRollerPctOutput(output.getAsDouble()))
            .withName("RollerByXbox");
    }

    public Command rollerOffCmd()
    {
        return runOnce(() -> setRollerPctOutput(0.0))
            .withName("RollerOff");
    }

    public Command intakeOffCmd()
    {
        return runOnce(() -> setRollerPctOutput(0.0))
            .withName("IntakeOff");
    }

    public void setIntakeStatus(boolean enabled)
    {
        SmartDashboard.putString("Intake/Status", enabled ? "On" : "Off");
    }

    public Command intakeOffCommand()
    {
        return Commands.run(() -> setIntakeStatus(false), this).withName("IntakeOff");
    }

    private void updateSmartDashboard()
    {
        SmartDashboard.putNumber("Intake/ArmSpeed", m_armMotor.getAppliedOutput());
        SmartDashboard.putNumber("Intake/ArmPosition", getArmPosition());
        SmartDashboard.putNumber("Intake/ArmPositionAbs", getArmAbsolutePosition());
        SmartDashboard.putNumber("Intake/ArmPositionTarget", m_targetArmPosition);
        SmartDashboard.putNumber("Intake/ArmTurnCount", m_turnCount);
        SmartDashboard.putBoolean("Intake/ArmPIDEnabled", m_armPIDEnabled);
        SmartDashboard.putBoolean("Intake/ArmEncoderConnected", m_absEncoder.isConnected());

        SmartDashboard.putNumber("Intake/RollerSpeed", m_rollerMotor.getAppliedOutput());
        SmartDashboard.putNumber("Intake/RollerCurrent", m_rollerMotor.getOutputCurrent());
    }
}
