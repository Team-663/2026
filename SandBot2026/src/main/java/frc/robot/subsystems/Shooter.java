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
    private final SparkMax m_kickerMotor = new SparkMax(CANConstants.SHOOTER_KICKER_CAN_ID, MotorType.kBrushless);
    private final SparkMax m_turretMotor = new SparkMax(CANConstants.SHOOTER_TURRET_CAN_ID, MotorType.kBrushless);
    private final TalonSRX m_hoodMotor = new TalonSRX(CANConstants.SHOOTER_HOOD_CAN_ID);
    //private final SparkMax m_shooterMaster = new SparkMax(CANConstants.SHOOTER_MASTER_CAN_ID, MotorType.kBrushless);
    //private final SparkMax m_shooterSlave = new SparkMax(CANConstants.SHOOTER_SLAVE_CAN_ID, MotorType.kBrushless);
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

        mconfig.withInverted(InvertedValue.Clockwise_Positive);
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



        /* WHEN SHOOTER WAS NEO... */
        /*
        SparkMaxConfig masterConfig = new SparkMaxConfig();
        SparkMaxConfig slaveConfig = new SparkMaxConfig();

        FeedForwardConfig ffcfg = new FeedForwardConfig();
        ffcfg.kS(ShooterConstants.SHOOTER_PID_FF_KS);
        ffcfg.kV(ShooterConstants.SHOOTER_PID_FF_KV);

        masterConfig.closedLoop.pid(ShooterConstants.SHOOTER_PID_P, ShooterConstants.SHOOTER_PID_I, ShooterConstants.SHOOTER_PID_D);
        masterConfig.closedLoop.feedForward.apply(ffcfg);

        m_shooterMaster.configure(masterConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        
        slaveConfig.follow(m_shooterMaster);
        slaveConfig.inverted(true);

        m_shooterSlave.configure(slaveConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        */
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

    private void ShooterInitHood()
    {
        // Hood motor uses TalonSRX controller
        m_hoodMotor.configFactoryDefault();
        m_hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); // TODO: verify encoder type
        m_hoodMotor.setSensorPhase(true);
        m_hoodMotor.configAllowableClosedloopError(0, ShooterConstants.HOOD_ERROR_TOLERANCE, m_talonTimeoutMs);

        m_hoodMotor.configClosedLoopPeakOutput(0, ShooterConstants.HOOD_MAX_OUTPUT_UP, m_talonTimeoutMs);
        m_hoodMotor.config_kP(0, ShooterConstants.HOOD_PID_P);
        m_hoodMotor.config_kI(0, ShooterConstants.HOOD_PID_I);
        m_hoodMotor.config_kD(0, ShooterConstants.HOOD_PID_D);
        m_hoodMotor.config_kF(0, ShooterConstants.HOOD_PID_FF);

        //TODO: enable soft limits when we have hood position ranges from the encoder
        m_hoodMotor.configForwardSoftLimitThreshold(ShooterConstants.HOOD_SOFT_LIMIT_FORWARD, m_talonTimeoutMs);
        m_hoodMotor.configForwardSoftLimitEnable(false);
        m_hoodMotor.configReverseSoftLimitThreshold(ShooterConstants.HOOD_SOFT_LIMIT_REVERSE, m_talonTimeoutMs);
        m_hoodMotor.configReverseSoftLimitEnable(false);
    
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

    public Command setShooterPctOutputCmd(DoubleSupplier output)
    {
        return run( ()-> {
            setShooterPctOutput(output.getAsDouble());
            })
            .withName("ShooterByXbox");
    }

    public Command shooterSetRPMCmd(double rpm)
    {
        return Commands.sequence(
                new InstantCommand(() -> shooterSetRPM(rpm)));
    }

    public Command shooterOffCmd()
    {
        return Commands.run(() -> shooterStop(), this).withName("ShooterOff");
    }

    private void setHoodPosition(double angle)
    {
        // TODO: implement hood position control
        m_targetHoodPosition = angle;
        //m_shooterHood.set(ControlMode.Position, angle);
    }

    private void setHoodPctOutput(double output)
    {
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

    private void setKickerPctOutput(double output)
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

    private void updateSmartDashboard()
    {
        SmartDashboard.putNumber("Shooter/Setpoint", m_targetShooterRPM);
        SmartDashboard.putNumber("Shooter/RPM", m_shooterMaster.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putBoolean("Shooter/AtTarget", isShooterAtRPM());
        
        SmartDashboard.putNumber("Hood/Setpoint", m_targetHoodPosition);
        SmartDashboard.putNumber("Hood/Position", m_hoodMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Hood/Speed", m_hoodMotor.getMotorOutputPercent());

        SmartDashboard.putNumber("Turret/Setpoint", m_targetTurretPosition);
        SmartDashboard.putNumber("Turret/Position", m_turretMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Turret/Speed", m_turretMotor.getEncoder().getVelocity());

        SmartDashboard.putNumber("Kicker/Speed", m_kickerMotor.getAppliedOutput());
    }
}