// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
    private final SparkMax m_shooterKicker = new SparkMax(CANConstants.SHOOTER_KICKER_CAN_ID, MotorType.kBrushless);
    private final SparkMax m_shooterTurret = new SparkMax(CANConstants.SHOOTER_TURRET_CAN_ID, MotorType.kBrushless);
    private final TalonSRX m_shooterHood = new TalonSRX(CANConstants.SHOOTER_HOOD_CAN_ID);
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
        fx_cfg.Feedback.SensorToMechanismRatio = 125.0 / 360.0;
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
        turretConfig.closedLoop.pid(ShooterConstants.TURRET_PID_P, ShooterConstants.TURRET_PID_I, ShooterConstants.TURRET_PID_D);

        turretConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
        turretConfig.encoder.positionConversionFactor(ShooterConstants.TURRET_ENCODER_CONVERSION_FACTOR);
        turretConfig.closedLoop.maxOutput(ShooterConstants.TURRET_MAX_OUTPUT);

        turretConfig.softLimit.forwardSoftLimit(ShooterConstants.TURRET_SOFT_LIMIT_FORWARD);
        turretConfig.softLimit.forwardSoftLimitEnabled(false);
        turretConfig.softLimit.reverseSoftLimit(ShooterConstants.TURRET_SOFT_LIMIT_REVERSE);
        turretConfig.softLimit.reverseSoftLimitEnabled(false);

        m_shooterTurret.configure(turretConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    private void ShooterInitHood()
    {
        // Hood motor uses TalonSRX controller
        m_shooterHood.configFactoryDefault();
        m_shooterHood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); // TODO: verify encoder type
        m_shooterHood.setSensorPhase(true);
        m_shooterHood.configAllowableClosedloopError(0, ShooterConstants.HOOD_ERROR_TOLERANCE, m_talonTimeoutMs);

        m_shooterHood.configClosedLoopPeakOutput(0, ShooterConstants.HOOD_MAX_OUTPUT_UP, m_talonTimeoutMs);
        m_shooterHood.config_kP(0, ShooterConstants.HOOD_PID_P);
        m_shooterHood.config_kI(0, ShooterConstants.HOOD_PID_I);
        m_shooterHood.config_kD(0, ShooterConstants.HOOD_PID_D);
        m_shooterHood.config_kF(0, ShooterConstants.HOOD_PID_FF);

        //TODO: enable soft limits when we have hood position ranges from the encoder
        m_shooterHood.configForwardSoftLimitThreshold(ShooterConstants.HOOD_SOFT_LIMIT_FORWARD, m_talonTimeoutMs);
        m_shooterHood.configForwardSoftLimitEnable(false);
        m_shooterHood.configReverseSoftLimitThreshold(ShooterConstants.HOOD_SOFT_LIMIT_REVERSE, m_talonTimeoutMs);
        m_shooterHood.configReverseSoftLimitEnable(false);
    
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

    public Command shooterSetRPMCommand(double rpm)
    {
        return Commands.sequence(
                new InstantCommand(() -> shooterSetRPM(rpm)));
    }

    public Command shooterOffCommand()
    {
        return Commands.sequence(
                new InstantCommand(() -> shooterStop()));
    }

    private void updateSmartDashboard()
    {
        SmartDashboard.putNumber("Shooter/Setpoint", m_targetShooterRPM);
        SmartDashboard.putNumber("Shooter/RPM", m_shooterMaster.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putBoolean("Shooter/AtTarget", isShooterAtRPM());
    }
}