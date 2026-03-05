// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase
{
    // Shooter has 5 motors: 1x kicker, 1x turret, 1x hood, 2x flywheel (master/slave)
    private final SparkMax m_agitatorMotor = new SparkMax(CANConstants.INTAKE_AGITATOR_CAN_ID, MotorType.kBrushed);
    private final SparkMax m_kickerMotor = new SparkMax(CANConstants.SHOOTER_KICKER_CAN_ID, MotorType.kBrushless);
    private final SparkMax m_turretMotor = new SparkMax(CANConstants.SHOOTER_TURRET_CAN_ID, MotorType.kBrushless);
    private final TalonSRX m_hoodMotor = new TalonSRX(CANConstants.SHOOTER_HOOD_CAN_ID);
    private final TalonFX m_shooterMaster = new TalonFX(CANConstants.SHOOTER_MASTER_CAN_ID);
    private final TalonFX m_shooterSlave = new TalonFX(CANConstants.SHOOTER_SLAVE_CAN_ID);
    
    // Turret: REV Through Bore Encoder on roboRIO DIO — absolute duty-cycle signal.
    // The encoder sits on the 24t pulley shaft; the output pulley is 200t.
    // Gear ratio = 200/24 ≈ 8.333 encoder revolutions per output revolution.
    private final DutyCycleEncoder m_turretAbsEncoder =
        new DutyCycleEncoder(ShooterConstants.TURRET_ABS_DIO_PORT);

    // Software PID controller — runs in periodic() at ~50 Hz.
    private final PIDController m_turretPID = new PIDController(
        ShooterConstants.TURRET_PID_P,
        ShooterConstants.TURRET_PID_I,
        ShooterConstants.TURRET_PID_D);

    // Continuous turret position tracking.
    // m_turretPositionDeg holds the accumulated output-shaft position in degrees.
    // Updated every cycle by looking at how much the raw encoder moved (delta).
    private double  m_turretPositionDeg = 0.0;  // continuous output-shaft degrees, 0 = straight
    private double  m_turretPrevRaw     = 0.0;  // previous SHIFTED encoder reading
    private boolean m_turretFirstRead   = true;

    // Offset applied to every raw reading so that "home" maps to 0.5 instead of
    // whatever the encoder happens to read at boot. This pushes the 0↔1 wrap
    // boundary far away from normal operating range, preventing noise-induced
    // phantom wraps. Set by rezeroTurret().
    private double  m_turretRawOffset   = 0.0;

    // True while the software PID should be driving the turret motor.
    private boolean m_turretPIDEnabled  = false;
    // Last PID output value, stored for SmartDashboard (avoid calling calculate() twice)
    private double  m_turretPIDOutput   = 0.0;

    private final int m_talonTimeoutMs = 10;

    private double m_targetShooterRPM     = 0.0;
    private double m_targetHoodPosition   = 0.0;
    private double m_targetTurretPosition = 0.0; // degrees, output shaft, 0 = straight
    /** Creates a new Shooter. */
    public Shooter()
    {
        ShooterInitHood();
        ShooterInitTurret();
        ShooterInitKicker();
        ShooterInitAgitator();
        ShooterInitFlywheel();
        rezeroTurret(); // seed turn count and zero offset from current position
    }

    @Override
    public void periodic()
    {
        // Always update the turret position tracker
        updateTurretPosition();

        // Run software PID for the turret when enabled
        if (m_turretPIDEnabled)
        {
            double currentAngle = getTurretAngle();
            double output = m_turretPID.calculate(currentAngle, m_targetTurretPosition);

            m_turretPIDOutput = output;  // save for SmartDashboard

            output = MathUtil.clamp(output,
                -ShooterConstants.TURRET_MAX_OUTPUT,
                 ShooterConstants.TURRET_MAX_OUTPUT);

            // Soft-limit enforcement: cut output if past the limit in that direction
            if (currentAngle >= ShooterConstants.TURRET_SOFT_LIMIT_FORWARD && output > 0) output = 0;
            if (currentAngle <= ShooterConstants.TURRET_SOFT_LIMIT_REVERSE && output < 0) output = 0;

            m_turretMotor.set(output);
        }

        updateSmartDashboard();
    }

    private void ShooterInitFlywheel()
    {
        MotorOutputConfigs mconfig = new MotorOutputConfigs();

        mconfig.withInverted(InvertedValue.CounterClockwise_Positive);
        mconfig.withNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
        //fx_cfg.Feedback.SensorToMechanismRatio = 125.0 / 360.0;
        fx_cfg.Feedback.RotorToSensorRatio = 1.0;

        fx_cfg.Slot0.kP = ShooterConstants.SHOOTER_PID_P;
        fx_cfg.Slot0.kI = ShooterConstants.SHOOTER_PID_I;
        fx_cfg.Slot0.kD = ShooterConstants.SHOOTER_PID_D;
        fx_cfg.Slot0.kV = ShooterConstants.SHOOTER_PID_FF_KV;
        fx_cfg.Slot0.kS = ShooterConstants.SHOOTER_PID_FF_KS;

        fx_cfg.CurrentLimits.withStatorCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
        fx_cfg.CurrentLimits.withStatorCurrentLimitEnable(true);

        fx_cfg.withMotorOutput(mconfig);

        StatusCode status = m_shooterMaster.getConfigurator().apply(fx_cfg);

        if (!status.isOK()) {
            System.out.println("Shooter Master configuration failed: " + status);
        }

        TalonFXConfiguration configSlave = new TalonFXConfiguration();
        configSlave.CurrentLimits.SupplyCurrentLimitEnable = true;
        configSlave.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_CURRENT_LIMIT;

        // Setup Slave motor configuration
        m_shooterSlave.setNeutralMode(NeutralModeValue.Coast);
        m_shooterSlave.setControl(new Follower(m_shooterMaster.getDeviceID(), MotorAlignmentValue.Opposed));
        status = m_shooterSlave.getConfigurator().apply(configSlave);

        if (!status.isOK()) {
            System.out.println("Shooter Slave configuration failed: " + status);
        }
    }

    private void ShooterInitTurret()
    {
        // SparkMax drives the turret motor in open-loop; PID runs in software via periodic().
        SparkMaxConfig turretConfig = new SparkMaxConfig();
        turretConfig.inverted(ShooterConstants.TURRET_MOTOR_INVERTED);
        turretConfig.idleMode(IdleMode.kBrake);
        turretConfig.smartCurrentLimit(30); // stall protection for the turret

        m_turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        m_turretPID.setTolerance(ShooterConstants.TURRET_ERROR_TOLERANCE);
    }

    private void ShooterInitKicker()
    {
        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        kickerConfig.inverted(false);
        kickerConfig.idleMode(IdleMode.kBrake);
        m_kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    private void ShooterInitAgitator()
    {
        SparkMaxConfig agitatorConfig = new SparkMaxConfig();
        agitatorConfig.inverted(false);
        agitatorConfig.idleMode(IdleMode.kBrake);
        m_agitatorMotor.configure(agitatorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    private void ShooterInitHood()
    {
        // Hood motor: BAG motor on TalonSRX with SRX Mag Encoder on output shaft via data port
        m_hoodMotor.configFactoryDefault(m_talonTimeoutMs);

        // Feedback sensor: SRX Mag Encoder in relative mode — zeroes on boot
        m_hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, m_talonTimeoutMs);
        m_hoodMotor.setSensorPhase(true); // TODO: verify — if hood moves opposite to expected, flip this

        // Zero the encoder on init so "0" = wherever the hood is at power-on
        m_hoodMotor.setSelectedSensorPosition(0, 0, m_talonTimeoutMs);

        // PID gains (slot 0) — tuned for fine, slow hood control
        m_hoodMotor.config_kP(0, ShooterConstants.HOOD_PID_P, m_talonTimeoutMs);
        m_hoodMotor.config_kI(0, ShooterConstants.HOOD_PID_I, m_talonTimeoutMs);
        m_hoodMotor.config_kD(0, ShooterConstants.HOOD_PID_D, m_talonTimeoutMs);
        m_hoodMotor.config_kF(0, ShooterConstants.HOOD_PID_FF, m_talonTimeoutMs);
        m_hoodMotor.config_IntegralZone(0, ShooterConstants.HOOD_PID_IZONE, m_talonTimeoutMs);
        m_hoodMotor.configAllowableClosedloopError(0, ShooterConstants.HOOD_ERROR_TOLERANCE, m_talonTimeoutMs);

        // Clamp peak output for fine control — 0.2 in either direction
        m_hoodMotor.configClosedLoopPeakOutput(0, ShooterConstants.HOOD_MAX_OUTPUT, m_talonTimeoutMs);
        m_hoodMotor.configPeakOutputForward(ShooterConstants.HOOD_MAX_OUTPUT, m_talonTimeoutMs);
        m_hoodMotor.configPeakOutputReverse(-ShooterConstants.HOOD_MAX_OUTPUT, m_talonTimeoutMs);

        // Ramp rate for smooth motion
        m_hoodMotor.configClosedloopRamp(ShooterConstants.HOOD_CLOSED_LOOP_RAMP, m_talonTimeoutMs);

        // Soft limits — enforced by the TalonSRX firmware
        m_hoodMotor.configForwardSoftLimitThreshold(ShooterConstants.HOOD_SOFT_LIMIT_FORWARD, m_talonTimeoutMs);
        m_hoodMotor.configForwardSoftLimitEnable(true, m_talonTimeoutMs);
        m_hoodMotor.configReverseSoftLimitThreshold(ShooterConstants.HOOD_SOFT_LIMIT_REVERSE, m_talonTimeoutMs);
        m_hoodMotor.configReverseSoftLimitEnable(true, m_talonTimeoutMs);
    }

    private void setShooterPctOutput(double value)
    {
        m_shooterMaster.set(value);
    }

    public void shooterSetRPM(double rpm)
    {
        m_targetShooterRPM = rpm;
        m_shooterMaster.setControl(new VelocityVoltage(rpm / 60.0)); // convert RPM to RPS
    }

    public void shooterStop()
    {
        m_targetShooterRPM = 0.0;
        m_shooterMaster.stopMotor();
    }

    public boolean isShooterAtRPM()
    {
        double currentRPM = m_shooterMaster.getVelocity().getValueAsDouble() * 60.0; // convert RPS to RPM
        return Math.abs(currentRPM - m_targetShooterRPM) <= ShooterConstants.SHOOTER_ERROR_TOLERANCE_RPM;
    }

    /** Returns true when the flywheel is spinning fast enough to safely accept a ball. */
    public boolean isShooterAboveMinRPM()
    {
        double currentRPM = m_shooterMaster.getVelocity().getValueAsDouble() * 60.0;
        return currentRPM >= ShooterConstants.SHOOTER_RPM_MIN_FEED;
    }

    public Command setShooterPctOutputCmd(DoubleSupplier output)
    {
        return run( ()-> {
            setShooterPctOutput(output.getAsDouble());
            })
            .withName("ShooterByXbox");
    }

    public Command shooterSetRPMCmd(double rpm)
    {
        //return Commands.sequence(
        //        new InstantCommand(() -> shooterSetRPM(rpm)));
        return Commands.runOnce(() -> shooterSetRPM(rpm), this)
            .withName("ShooterSetRPM_" + rpm);
    }

    public Command shooterOffCmd()
    {
        return Commands.run(() -> shooterStop(), this).withName("ShooterOff");
    }

    /**
     * Commands the hood to a position in native sensor units (SRX Mag Encoder ticks).
     * 4096 ticks = 1 revolution of the encoder (16t gear) shaft.
     */
    private void setHoodPosition(double positionTicks)
    {
        m_targetHoodPosition = positionTicks;
        m_hoodMotor.set(ControlMode.Position, positionTicks);
    }

    /** Resets the hood encoder position to 0. Useful for re-zeroing during debugging. */
    public void resetHoodEncoder()
    {
        m_hoodMotor.setSelectedSensorPosition(0, 0, m_talonTimeoutMs);
        m_targetHoodPosition = 0.0;
    }

    /**
     * Convenience: set the hood position in degrees of the 200-tooth output gear.
     * Clamped to [0, HOOD_MAX_ANGLE_DEG] for safety.
     *
     * @param degrees angle of the 200t gear relative to its zero position
     */
    public void setHoodAngle(double degrees)
    {
        degrees = Math.max(ShooterConstants.HOOD_MIN_ANGLE_DEG,
                  Math.min(degrees, ShooterConstants.HOOD_MAX_ANGLE_DEG));
        setHoodPosition(degrees * ShooterConstants.HOOD_TICKS_PER_DEGREE);
    }

    /** Returns the current hood angle in degrees of the 200-tooth gear. */
    public double getHoodAngle()
    {
        return m_hoodMotor.getSelectedSensorPosition() / ShooterConstants.HOOD_TICKS_PER_DEGREE;
    }

    /** Returns true when the hood is within the allowable error of its target. */
    public boolean isHoodAtTarget()
    {
        return Math.abs(m_hoodMotor.getSelectedSensorPosition() - m_targetHoodPosition)
                <= ShooterConstants.HOOD_ERROR_TOLERANCE;
    }

    /** Command to move the hood to a specific angle of the 200-tooth gear (degrees). */
    public Command setHoodAngleCmd(double degrees)
    {
        return Commands.runOnce(() -> setHoodAngle(degrees), this)
            .withName("HoodAngle_" + degrees);
    }

    /** Command to move the hood to a specific position (native sensor ticks). */
    public Command setHoodPositionCmd(double positionTicks)
    {
        return Commands.runOnce(() -> setHoodPosition(positionTicks), this)
            .withName("HoodPosition_" + positionTicks);
    }

    private void setHoodPctOutput(double output)
    {
        output = output * ShooterConstants.HOOD_MANUAL_SPEED_SCALAR;
        output = Math.max(-ShooterConstants.HOOD_MAX_OUTPUT,
                 Math.min(output, ShooterConstants.HOOD_MAX_OUTPUT));
        m_hoodMotor.set(ControlMode.PercentOutput, output);
    }

    public Command setHoodPctOutputCmd(DoubleSupplier output)
    {
        return run( ()-> {setHoodPctOutput(output.getAsDouble());})
            .withName("HoodByXbox");
    }

    public Command hoodOffCmd()
    {
        return Commands.sequence(
                new InstantCommand(() -> setHoodPctOutput(0.0)));
    }

    // -------------------------------------------------------------------------
    // Turret — simple delta-based position tracking
    // -------------------------------------------------------------------------

    /**
     * Called every periodic() to accumulate encoder movement into m_turretPositionDeg.
     *
     * Reads the raw encoder, applies m_turretRawOffset so that the "home" position
     * sits at 0.5 (far from the 0/1 wrap boundary), computes the delta from the
     * previous shifted reading, unwraps it if needed, and accumulates.
     */
    private void updateTurretPosition()
    {
        // Shift raw reading so home ≈ 0.5.  Result stays in [0.0, 1.0).
        double shifted = (m_turretAbsEncoder.get() - m_turretRawOffset + 1.0) % 1.0;

        if (m_turretFirstRead)
        {
            m_turretPrevRaw   = shifted;
            m_turretFirstRead = false;
            return;
        }

        double delta = shifted - m_turretPrevRaw;

        // Unwrap: a jump > 0.5 in one 20 ms cycle means the encoder wrapped.
        if (delta > 0.5)       delta -= 1.0;
        else if (delta < -0.5) delta += 1.0;

        // Convert delta (encoder-shaft revolutions) to output-shaft degrees.
        double sign = ShooterConstants.TURRET_ENCODER_INVERTED ? -1.0 : 1.0;
        m_turretPositionDeg += (delta * 360.0 / ShooterConstants.TURRET_GEAR_RATIO) * sign;

        m_turretPrevRaw = shifted;
    }

    /** Returns the turret output-shaft position in degrees. 0 = wherever it was at startup. */
    public double getTurretAngle()
    {
        return m_turretPositionDeg;
    }

    /**
     * Zeros the turret position to 0 at the current physical location.
     * Also sets m_turretRawOffset so the current raw reading maps to 0.5,
     * keeping the wrap boundary as far away as possible from normal operation.
     */
    public void rezeroTurret()
    {
        // Offset = rawNow − 0.5, so (raw − offset) % 1.0 = 0.5 at this instant.
        m_turretRawOffset      = m_turretAbsEncoder.get() - 0.5;
        m_turretPrevRaw        = 0.5;  // shifted value right now
        m_turretFirstRead      = false;
        m_turretPositionDeg    = 0.0;
        m_targetTurretPosition = 0.0;
        m_turretPIDEnabled     = false;
        m_turretMotor.stopMotor();
    }

    /**
     * Commands the turret to a target angle (degrees of output shaft, 0 = straight).
     * Enables the software PID loop which runs in periodic().
     */
    public void setTurretAngle(double degrees)
    {
        degrees = MathUtil.clamp(degrees,
            ShooterConstants.TURRET_SOFT_LIMIT_REVERSE,
            ShooterConstants.TURRET_SOFT_LIMIT_FORWARD);
        m_targetTurretPosition = degrees;
        m_turretPIDEnabled     = true;
    }

    /** Returns true when the turret is within tolerance of its target angle. */
    public boolean isTurretAtTarget()
    {
        return m_turretPIDEnabled && m_turretPID.atSetpoint();
    }

    /** Stops the turret motor and disables the PID loop. */
    public void turretStop()
    {
        m_turretPIDEnabled = false;
        m_turretMotor.stopMotor();
    }

    /** Command to move the turret to a specific angle (degrees, output shaft). */
    public Command setTurretAngleCmd(double degrees)
    {
        return Commands.runOnce(() -> setTurretAngle(degrees))
            .withName("TurretAngle_" + degrees);
    }

    /** Snapshots the current angle and holds it with closed-loop. */
    public void holdTurretPosition()
    {
        setTurretAngle(getTurretAngle());
    }

    /** Command version — use as .onFalse() when releasing the manual stick. */
    public Command holdTurretPositionCmd()
    {
        return Commands.runOnce(() -> holdTurretPosition())
            .withName("TurretHold");
    }

    /** Command to re-zero the turret at its current position. */
    public Command rezeroTurretCmd()
    {
        return Commands.runOnce(() -> rezeroTurret())
            .withName("TurretRezero");
    }

    private void setTurretPctOutput(double output)
    {
        // Disable PID while in manual mode so it doesn't fight the stick.
        m_turretPIDEnabled = false;

        // Enforce soft limits: zero the output if already past a limit in that direction.
        double currentAngle = getTurretAngle();
        if (currentAngle >= ShooterConstants.TURRET_SOFT_LIMIT_FORWARD && output > 0) output = 0;
        if (currentAngle <= ShooterConstants.TURRET_SOFT_LIMIT_REVERSE && output < 0) output = 0;

        m_turretMotor.set(output);
    }

    public Command setTurretPctOutputCmd(DoubleSupplier output)
    {
        return Commands.run(() -> setTurretPctOutput(output.getAsDouble()))
            .withName("TurretByXbox");
    }

    public Command turretOffCmd()
    {
        return Commands.runOnce(() -> setTurretPctOutput(0.0));
    }

    public void setKickerPctOutput(double output)
    {
        m_kickerMotor.set(output);
    }

    public Command setKickerPctOutputCmd(DoubleSupplier output)
    {
        return run( ()-> {setKickerPctOutput(output.getAsDouble());})
            .withName("KickerByXbox");
    }

    public Command kickerOffCmd()
    {
        return Commands.sequence(
                new InstantCommand(() -> setKickerPctOutput(0.0)));
    }

    public void setAgitatorPctOutput(double output)
    {
        SmartDashboard.putNumber("Intake/AgitatorOutput", output);
        m_agitatorMotor.set(output);
    }

    public Command setAgitatorPctOutputCmd(DoubleSupplier output)
    {
        return run( ()-> {setAgitatorPctOutput(output.getAsDouble());})
            .withName("AgitatorByXbox");
    }

    public Command agitatorOffCmd()
    {
        return Commands.sequence(
                new InstantCommand(() -> setAgitatorPctOutput(0.0)));
    }

    private void updateSmartDashboard()
    {
        SmartDashboard.putNumber("Shooter/Setpoint", m_targetShooterRPM);
        SmartDashboard.putNumber("Shooter/RPM", m_shooterMaster.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putBoolean("Shooter/AtTarget", isShooterAtRPM());
        SmartDashboard.putNumber("Shooter/Voltage", m_shooterMaster.getMotorVoltage().getValueAsDouble());
        
        SmartDashboard.putNumber("Shooter/Error_RPM", m_targetShooterRPM - (m_shooterMaster.getVelocity().getValueAsDouble() * 60.0));
        SmartDashboard.putNumber("Shooter/Current_Amps", m_shooterMaster.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Supply_Amps", m_shooterMaster.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/ClosedLoopOutput", m_shooterMaster.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/ClosedLoopRef", m_shooterMaster.getClosedLoopReference().getValueAsDouble());
        
        
        SmartDashboard.putNumber("Hood/Setpoint", m_targetHoodPosition);
        SmartDashboard.putNumber("Hood/Setpoint_Deg", m_targetHoodPosition / ShooterConstants.HOOD_TICKS_PER_DEGREE);
        SmartDashboard.putNumber("Hood/Position", m_hoodMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Hood/Angle_Deg", getHoodAngle());
        SmartDashboard.putNumber("Hood/Error", m_targetHoodPosition - m_hoodMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Hood/Speed", m_hoodMotor.getMotorOutputPercent());
        SmartDashboard.putBoolean("Hood/AtTarget", isHoodAtTarget());

        SmartDashboard.putNumber("Turret/Setpoint_Deg", m_targetTurretPosition);
        SmartDashboard.putNumber("Turret/Angle_Deg", getTurretAngle());
        SmartDashboard.putNumber("Turret/AbsRaw", m_turretAbsEncoder.get());
        SmartDashboard.putBoolean("Turret/AtTarget", isTurretAtTarget());
        SmartDashboard.putBoolean("Turret/PIDEnabled", m_turretPIDEnabled);
        SmartDashboard.putNumber("Turret/Error_Deg", m_targetTurretPosition - getTurretAngle());
        SmartDashboard.putNumber("Turret/Output", m_turretMotor.getAppliedOutput());
        SmartDashboard.putNumber("Turret/PID_Output", m_turretPIDOutput);
        SmartDashboard.putBoolean("Turret/EncoderConnected", m_turretAbsEncoder.isConnected());

        SmartDashboard.putNumber("Kicker/Speed", m_kickerMotor.getAppliedOutput());

        // Aggitator speed and current
        SmartDashboard.putNumber("Agitator/Speed", m_agitatorMotor.getAppliedOutput());
        SmartDashboard.putNumber("Agitator/Current", m_agitatorMotor.getOutputCurrent());
    }
}