// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Intake extends SubsystemBase
{
    // Intake has 3x SparkMax motors: 1 for arm, 1 for agitator, 1 for roller
    private final SparkMax m_armMotor = new SparkMax(CANConstants.INTAKE_ARM_CAN_ID, MotorType.kBrushless);
    private final SparkMax m_agitatorMotor = new SparkMax(CANConstants.INTAKE_AGITATOR_CAN_ID, MotorType.kBrushed);
    private final SparkMax m_rollerMotor = new SparkMax(CANConstants.INTAKE_ROLLER_CAN_ID, MotorType.kBrushless);
    
    private double m_targetArmPosition = 0.0;
    
    
    /** Creates a new Intake. */
    public Intake()
    {
        IntakeInitMotors();
    }

    private void IntakeInitMotors()
    {
        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.inverted(false);
        armConfig.idleMode(IdleMode.kBrake);

        armConfig.closedLoop.pid(IntakeConstants.INTAKE_ARM_PID_P, IntakeConstants.INTAKE_ARM_PID_I, IntakeConstants.INTAKE_ARM_PID_D);

        //armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
        //armConfig.encoder.positionConversionFactor(ShooterConstants.TURRET_ENCODER_CONVERSION_FACTOR);
        armConfig.closedLoop.maxOutput(IntakeConstants.INTAKE_ARM_MAX_OUTPUT);

        armConfig.softLimit.forwardSoftLimit(0.0);
        armConfig.softLimit.forwardSoftLimitEnabled(false);
        armConfig.softLimit.reverseSoftLimit(0.0); 
        armConfig.softLimit.reverseSoftLimitEnabled(false);
        
        m_armMotor.configure(armConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        SparkMaxConfig agitatorConfig = new SparkMaxConfig();
        agitatorConfig.inverted(false);
        agitatorConfig.idleMode(IdleMode.kBrake);
        m_agitatorMotor.configure(agitatorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.inverted(false);
        rollerConfig.idleMode(IdleMode.kCoast);
        m_rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        updateSmartDashboard();
    }

    private void setArmPosition(double angle)
    {
        // TODO: implement hood position control
        m_targetArmPosition = angle;
        //m_shooterHood.set(ControlMode.Position, angle);
    }

    private void setArmPctOutput(double output)
    {
        m_armMotor.set(output);
    }

    public Command setArmPctOutputCmd(DoubleSupplier output)
    {
        return run( ()-> {setArmPctOutput(output.getAsDouble());})
            .withName("ArmByXbox");
    }

    public Command armOffCmd()
    {
        return Commands.runOnce(() -> setArmPctOutput(0.0), this).withName("ArmOff");
    }

    private void setAgitatorPctOutput(double output)
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
        return Commands.runOnce(() -> setAgitatorPctOutput(0.0), this).withName("AgitatorOff");
    }

    private void setRollerPctOutput(double output)
    {
        m_rollerMotor.set(output);
    }

    public Command setRollerPctOutputCmd(DoubleSupplier output)
    {
        return run( ()-> {setRollerPctOutput(output.getAsDouble());})
            .withName("RollerByXbox");
    }

    public Command rollerOffCmd()
    {
        return Commands.runOnce(() -> setRollerPctOutput(0.0), this).withName("RollerOff");
    }

    public void setIntakeStatus(boolean enabled)
    {
        if (enabled)
        {
            SmartDashboard.putString("Intake/Status", "On");
        }
        else
        {
            SmartDashboard.putString("Intake/Status", "Off");
        }
    }

    public Command intakeOffCommand()
    {
       // kill switch for the entire intake subsystem and update the SmartDashboard.
       //sets the power of all three motors to 0.0
       return Commands.runOnce(() -> {
            setIntakeStatus(false);
            setArmPctOutput(0.0);
            setAgitatorPctOutput(0.0);
            setRollerPctOutput(0.0);
        }, this).withName("IntakeOff");
    }



    private void updateSmartDashboard()
    {
        SmartDashboard.putNumber("Intake/ArmSpeed", m_armMotor.getAppliedOutput());
        SmartDashboard.putNumber("Intake/ArmPosition", m_armMotor.getEncoder().getPosition());

        SmartDashboard.putNumber("Intake/RollerSpeed", m_rollerMotor.getAppliedOutput());
        SmartDashboard.putNumber("Intake/AgitatorSpeed", m_agitatorMotor.getAppliedOutput());
    }
}
