// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
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

public class Intake extends SubsystemBase
{
    // Intake has 2x SparkMax motors: 1 for arm, 1 for roller
    private final SparkMax m_armMotor = new SparkMax(CANConstants.INTAKE_ARM_CAN_ID, MotorType.kBrushless);
    private final SparkMax m_rollerMotor = new SparkMax(CANConstants.INTAKE_ROLLER_CAN_ID, MotorType.kBrushed);
    
    // Closed-loop controller and encoder reference, retrieved after configuration
    private final SparkClosedLoopController m_armPID = m_armMotor.getClosedLoopController();

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

        // Scale the built-in encoder so 1 unit = 1 output-shaft rotation (16:1 gearbox)
        armConfig.encoder.positionConversionFactor(IntakeConstants.INTAKE_ARM_ENCODER_CONVERSION_FACTOR);

        armConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        armConfig.closedLoop.pid(IntakeConstants.INTAKE_ARM_PID_P, IntakeConstants.INTAKE_ARM_PID_I, IntakeConstants.INTAKE_ARM_PID_D);
        armConfig.closedLoop.maxOutput(IntakeConstants.INTAKE_ARM_MAX_OUTPUT);

        // TODO: enable soft limits once physical travel is measured (values in output-shaft rotations)
        armConfig.softLimit.forwardSoftLimit(IntakeConstants.INTAKE_ARM_SOFT_LIMIT_FORWARD);
        armConfig.softLimit.forwardSoftLimitEnabled(false);
        armConfig.softLimit.reverseSoftLimit(IntakeConstants.INTAKE_ARM_SOFT_LIMIT_REVERSE);
        armConfig.softLimit.reverseSoftLimitEnabled(false);
        
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
        // This method will be called once per scheduler run
        updateSmartDashboard();
    }

    private void setArmPosition(double outputShaftRotations)
    {
        m_targetArmPosition = outputShaftRotations;
        m_armPID.setSetpoint(outputShaftRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public boolean isArmAtTarget()
    {
        return Math.abs(m_armMotor.getEncoder().getPosition() - m_targetArmPosition)
                <= IntakeConstants.INTAKE_ARM_ERROR_TOLERANCE;
    }

    public Command setArmPositionCmd(double outputShaftRotations)
    {
        return runOnce(() -> setArmPosition(outputShaftRotations))
            .withName("ArmPosition_" + outputShaftRotations);
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
        return Commands.sequence(
                new InstantCommand(() -> setArmPctOutput(0.0)));
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
        return Commands.sequence(
                new InstantCommand(() -> setRollerPctOutput(0.0)));
    }

    // Turn off roller and agitator together
    public Command intakeOffCmd()
    {
        return Commands.sequence(
                new InstantCommand(() -> setRollerPctOutput(0.0))
            ).withName("IntakeOff");
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
        return Commands.run(() -> setIntakeStatus(false), this).withName("IntakeOff");
    }



    private void updateSmartDashboard()
    {
        SmartDashboard.putNumber("Intake/ArmSpeed", m_armMotor.getAppliedOutput());
        SmartDashboard.putNumber("Intake/ArmPosition", m_armMotor.getEncoder().getPosition());

        SmartDashboard.putNumber("Intake/RollerSpeed", m_rollerMotor.getAppliedOutput());
        // Log current of rollers
        SmartDashboard.putNumber("Intake/RollerCurrent", m_rollerMotor.getOutputCurrent());
    }
}
