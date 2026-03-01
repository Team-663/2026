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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;;

public class Shooter extends SubsystemBase
{
    // Shooter has 5 motors: 1x kicker, 1x turret, 1x hood, 2x flywheel (master/slave)
    private final SparkMax m_agitatorMotor = new SparkMax(CANConstants.INTAKE_AGITATOR_CAN_ID, MotorType.kBrushed);
    private final SparkMax m_kickerMotor = new SparkMax(CANConstants.SHOOTER_KICKER_CAN_ID, MotorType.kBrushless);
    private final SparkMax m_turretMotor = new SparkMax(CANConstants.SHOOTER_TURRET_CAN_ID, MotorType.kBrushless);
    private final TalonSRX m_hoodMotor = new TalonSRX(CANConstants.SHOOTER_HOOD_CAN_ID);
    private final TalonFX m_shooterMaster = new TalonFX(CANConstants.SHOOTER_MASTER_CAN_ID);
    private final TalonFX m_shooterSlave = new TalonFX(CANConstants.SHOOTER_SLAVE_CAN_ID);
    
    //private final SparkClosedLoopController m_turretPID = m_shooterTurret.getClosedLoopController();

    //private final DigitalInput m_elevatorLimitLow = new DigitalInput(ArmConstants.ELEVATOR_LOW_LIMIT_SWITCH_PORT);
    // EXAMPLE OF TALON FX POSITION PID
    //private final TalonFX m_wrist = new TalonFX(Constants.ARM_WRIST_CAN_ID);
    //private final PositionVoltage m_wristVoltage = new PositionVoltage(0);
    private final int m_talonTimeoutMs = 10;

    private double m_targetShooterRPM = 0.0;
    private double m_targetHoodPosition = 0.0;
    private double m_targetTurretPosition = 0.0;
    /** Creates a new Shooter. */
    public Shooter()
    {
        ShooterInitHood();
        ShooterInitTurret();
        ShooterInitKicker();
        ShooterInitAgitator();
        ShooterInitFlywheel();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
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
        SparkMaxConfig turretConfig = new SparkMaxConfig();
        turretConfig.inverted(false);
        turretConfig.idleMode(IdleMode.kBrake);
        turretConfig.closedLoop.pid(ShooterConstants.TURRET_PID_P, ShooterConstants.TURRET_PID_I, ShooterConstants.TURRET_PID_D);

        turretConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        turretConfig.encoder.positionConversionFactor(ShooterConstants.TURRET_ENCODER_CONVERSION_FACTOR);
        turretConfig.closedLoop.maxOutput(ShooterConstants.TURRET_MAX_OUTPUT);

        turretConfig.softLimit.forwardSoftLimit(ShooterConstants.TURRET_SOFT_LIMIT_FORWARD);
        turretConfig.softLimit.forwardSoftLimitEnabled(false);
        turretConfig.softLimit.reverseSoftLimit(ShooterConstants.TURRET_SOFT_LIMIT_REVERSE);
        turretConfig.softLimit.reverseSoftLimitEnabled(false);

        m_turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
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

    private void setTurretPosition(double angle)
    {
        m_targetTurretPosition = angle;
        //m_turretMotor.getClosedLoopController().setSetpoint(angle, ControlType.kPosition);
    }

    private void setTurretPctOutput(double output)
    {
        m_turretMotor.set(output);
    }

    public Command setTurretPctOutputCmd(DoubleSupplier output)
    {
        return run( ()-> {setTurretPctOutput(output.getAsDouble());})
            .withName("TurretByXbox");
    }

    public Command turretOffCmd()
    {
        return Commands.sequence(
                new InstantCommand(() -> setTurretPctOutput(0.0)));
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

        SmartDashboard.putNumber("Turret/Setpoint", m_targetTurretPosition);
        SmartDashboard.putNumber("Turret/Position", m_turretMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Turret/Speed", m_turretMotor.getEncoder().getVelocity());

        SmartDashboard.putNumber("Kicker/Speed", m_kickerMotor.getAppliedOutput());

        // Aggitator speed and current
        SmartDashboard.putNumber("Agitator/Speed", m_agitatorMotor.getAppliedOutput());
        SmartDashboard.putNumber("Agitator/Current", m_agitatorMotor.getOutputCurrent());
    }
}