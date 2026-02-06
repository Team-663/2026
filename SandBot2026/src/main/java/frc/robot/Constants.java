// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants 
{
    public static class CANConstants
    {   // PIGEON = 20
        // BackLeftDrive/Angle:    4,  5
        // BackRightDrive/Angle:   6,  7
        // FrontLeftDrive/Angle:   8,  9
        // FrontRightDrive/Angle: 10, 11

        public static final int INTAKE_ARM_CAN_ID      = 12;
        public static final int INTAKE_ROLLER_CAN_ID   = 13;

        public static final int SHOOTER_KICKER_CAN_ID  = 21;
        public static final int SHOOTER_TURRET_CAN_ID  = 22;
        public static final int SHOOTER_HOOD_CAN_ID    = 23;
        public static final int SHOOTER_MASTER_CAN_ID  = 24; // from turret perspective, LEFT motor if mounted outside
        public static final int SHOOTER_SLAVE_CAN_ID   = 25;
    }

    public static class DrivebaseConstants
    {
        public static final double MAX_SPEED = Units.feetToMeters(15.5);
    }

    public static class OperatorConstants
    {
        public static final int XBOX_DRIVER_PORT   = 0;
        public static final int XBOX_OPERATOR_PORT = 1;

        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }

    public static class ShooterConstants
    {
        public static final double TURRET_PID_P = 0.75;
        public static final double TURRET_PID_I = 0.0;
        public static final double TURRET_PID_D = 0.1;
        public static final double TURRET_PID_FF = 0.0;
        public static final double TURRET_ERROR_TOLERANCE = 1.0; // degrees
        public static final double TURRET_ENCODER_CONVERSION_FACTOR = 360.0 / 4096.0; // degrees per encoder tick
        public static final double TURRET_SOFT_LIMIT_FORWARD = 100.0; // degrees
        public static final double TURRET_SOFT_LIMIT_REVERSE = -100.0;

        public static final double TURRET_MAX_OUTPUT = 0.25;     
        public static final double TURRET_MAX_OUTPUT_XBOX = 1.0;

        public static final double HOOD_PID_P = 0.1;
        public static final double HOOD_PID_I = 0.0;
        public static final double HOOD_PID_D = 0.0;
        public static final double HOOD_PID_FF = 0.0;
        public static final double HOOD_ERROR_TOLERANCE = 1.0; //
        
        public static final double HOOD_MAX_OUTPUT_UP = 0.75;
        public static final double HOOD_MAX_OUTPUT_DOWN = -0.50;
        public static final double HOOD_MAX_OUTPUT_XBOX = 1.0;

        public static final double HOOD_SOFT_LIMIT_FORWARD = 5000.0; // TODO: measure this limit from encoder
        public static final double HOOD_SOFT_LIMIT_REVERSE = 0.0; // TODO: measure this limit from encoder

        public static final double SHOOTER_PID_P = 0.08;
        public static final double SHOOTER_PID_I = 0.0;
        public static final double SHOOTER_PID_D = 0.0;
        // Formlat for kV = (volts * seconds) / meter
        // Below is for 5500 RPM (91.7 RPS) at 12 V: 91.7 * 0.12 = 11.0 volts (leaves some headroom)
        public static final double SHOOTER_PID_FF_KV = 0.12; // Some commendations were 0.12 (phoenix6 is velocityVolts)
        public static final double SHOOTER_PID_FF_KS = 0.25;
        public static final double SHOOTER_ERROR_TOLERANCE_RPM = 25.0;
        public static final double SHOOTER_CURRENT_LIMIT = 60.0;

        public static final double SHOOTER_RPM_LOW = 2000.0;
        public static final double SHOOTER_RPM_MEDIUM = 3500.0;
        public static final double SHOOTER_RPM_HIGH = 5800.0;

    }

    public static class IntakeConstants
    {
        public static final double INTAKE_ARM_PID_P = 0.75;
        public static final double INTAKE_ARM_PID_I = 0.0;
        public static final double INTAKE_ARM_PID_D = 0.1;
        public static final double INTAKE_ARM_PID_FF = 0.0;
        public static final double INTAKE_ARM_ERROR_TOLERANCE = 1.0; // degrees
        public static final double INTAKE_ARM_MAX_OUTPUT = 0.25;     
        public static final double INTAKE_ARM_MAX_OUTPUT_XBOX = 1.0;

        public static final double INTAKE_ROLLER_IN_SPEED = 0.75;
        public static final double INTAKE_ROLLER_OUT_SPEED = -0.5;
    }
}
