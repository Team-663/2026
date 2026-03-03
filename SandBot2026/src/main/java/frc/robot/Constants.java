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

        public static final int INTAKE_ARM_CAN_ID       = 12;
        public static final int INTAKE_ROLLER_CAN_ID    = 13;
        public static final int INTAKE_AGITATOR_CAN_ID  = 14;

        public static final int SHOOTER_KICKER_CAN_ID   = 21;
        public static final int SHOOTER_TURRET_CAN_ID   = 22;
        public static final int SHOOTER_HOOD_CAN_ID     = 23;
        public static final int SHOOTER_MASTER_CAN_ID   = 24; // from turret perspective, LEFT motor if mounted outside
        public static final int SHOOTER_SLAVE_CAN_ID    = 25;

        public static final int CLIMBER_1_CAN_ID        = 30;
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

        // Hood motor: BAG motor + 16:1 gearbox, TalonSRX with SRX Mag Encoder on the
        // 16:1 output shaft. That shaft carries a 16-tooth gear meshed with a 200-tooth gear.
        // Encoder: 4096 native ticks per revolution of the 16t gear shaft.
        // Gear mesh: 200t / 16t = 12.5 → 12.5 encoder revolutions per 1 revolution of the 200t gear.
        // 1° of 200t gear = (200/16) * (4096/360) = 142.222… encoder ticks.
        public static final double HOOD_GEAR_TEETH_INPUT  = 16.0;
        public static final double HOOD_GEAR_TEETH_OUTPUT = 200.0;
        public static final double HOOD_TICKS_PER_ENCODER_REV = 4096.0;
        public static final double HOOD_TICKS_PER_DEGREE =
            (HOOD_GEAR_TEETH_OUTPUT / HOOD_GEAR_TEETH_INPUT)
            * (HOOD_TICKS_PER_ENCODER_REV / 360.0); // ≈ 142.22
        public static final double HOOD_PID_P = 0.6;   // Start here; increase if it never reaches target
        public static final double HOOD_PID_I = 0.0;   // Add small value (e.g. 0.001) if steady-state error persists
        public static final double HOOD_PID_D = 4.0;   // ~10x kP for damping; reduce if it vibrates
        public static final double HOOD_PID_FF = 0.0;   // No feedforward needed for position control
        public static final double HOOD_ERROR_TOLERANCE = 10; // native units (~0.88 degrees of output shaft)
        public static final int    HOOD_PID_IZONE = 50; // I accumulator only active within this error band (native units)
        
        // Peak output clamped low for fine control — hood doesn't need much power
        public static final double HOOD_MAX_OUTPUT = 0.4;
        public static final double HOOD_MANUAL_SPEED_SCALAR = 0.4;
        public static final double HOOD_MAX_OUTPUT_XBOX = 1.0;

        // Ramp rate: seconds from 0 to full output — prevents jerky motion
        public static final double HOOD_CLOSED_LOOP_RAMP = 0.25;  // 250 ms ramp

        // Hood travel limits (degrees of the 200-tooth gear)
        public static final double HOOD_MIN_ANGLE_DEG = 0.0;   // never go negative
        public static final double HOOD_MAX_ANGLE_DEG = 37.0;  // max travel

        public static final double HOOD_ANGLE_LONG_SHOT = 32.0;
        public static final double HOOD_ANGLE_MEDIUM_SHOT = 20.0;
        public static final double HOOD_ANGLE_SHORT_SHOT = 10.0;

        // Soft limits in native encoder ticks (derived from degree limits)
        public static final double HOOD_SOFT_LIMIT_FORWARD = HOOD_MAX_ANGLE_DEG * HOOD_TICKS_PER_DEGREE; // ~5689 ticks
        public static final double HOOD_SOFT_LIMIT_REVERSE = HOOD_MIN_ANGLE_DEG * HOOD_TICKS_PER_DEGREE; // 0 ticks

        public static final double SHOOTER_PID_P = 0.003;
        public static final double SHOOTER_PID_I = 0.0;
        public static final double SHOOTER_PID_D = 0.0;
        // Formlat for kV = (volts * seconds) / meter
        // Below is for 5500 RPM (91.7 RPS) at 12 V: 91.7 * 0.12 = 11.0 volts (leaves some headroom)
        public static final double SHOOTER_PID_FF_KV = 0.118; // Some commendations were 0.12 (phoenix6 is velocityVolts)
        public static final double SHOOTER_PID_FF_KS = 0.25;
        public static final double SHOOTER_ERROR_TOLERANCE_RPM = 100.0;
        public static final double SHOOTER_CURRENT_LIMIT = 100.0;

        public static final double SHOOTER_RPM_LOW = 3000.0;
        public static final double SHOOTER_RPM_MEDIUM = 3800.0;
        public static final double SHOOTER_RPM_HIGH = 5000.0;

        // Minimum RPM the flywheel must be spinning before balls are fed in.
        // Prevents jamming if the shooter is idle or spinning up from rest.
        public static final double SHOOTER_RPM_MIN_FEED = 1500.0;

        public static final double KICKER_FEED_SPEED    = 1.0;
        public static final double AGITATOR_FEED_SPEED  = 0.9;

    }

    public static class IntakeConstants
    {
        // ----------------------------------------------------------------------------------
        // Through Bore Encoder wired to roboRIO DIO.
        //   • White  (ABS, duty-cycle) → DIO 0  (absolute position within 1 revolution)
        //   • Green  (Index)           → DIO 1  (1 pulse per revolution, for turn counting)
        //
        // Breakout cable colors:  White=ABS, Yellow=ChA, Blue=ChB, Green=Index
        //
        // Position units throughout: output-shaft ROTATIONS (1.0 = one full turn).
        // The absolute signal gives precise position within one turn (0.0–1.0).
        // The index pulse counts full revolutions for multi-turn tracking.
        // Combined: position = (turnCount + absPosition) gives continuous multi-turn position.
        // ----------------------------------------------------------------------------------

        public static final int INTAKE_ARM_ABS_DIO_PORT   = 0; // duty-cycle absolute (white)
        public static final int INTAKE_ARM_INDEX_DIO_PORT = 1; // index pulse (green)

        // Absolute encoder zero offset (0.0–1.0).
        // Set this to the raw absolute encoder reading when the arm is at your
        // desired "zero" / home position. All position readings will then be
        // relative to that point (home = 0.0).
        // To calibrate: move the arm to home, read "Intake/ArmPositionAbsRaw"
        // from SmartDashboard, and paste that value here.
        public static final double INTAKE_ARM_ABS_ZERO_OFFSET =  0.896;// TODO: set to measured value at home

        // Named arm positions (output-shaft rotations, multi-turn capable).
        public static final double INTAKE_ARM_STOWED = 0.0;
        public static final double INTAKE_ARM_DEPLOYED = 1.1;

        // PID values for the arm position loop (units = rotations).
        public static final double INTAKE_ARM_PID_P = 2.0;
        public static final double INTAKE_ARM_PID_I = 0.0;
        public static final double INTAKE_ARM_PID_D = 0.1;

        // Allowable position error (rotations). 0.02 ≈ 7° of output shaft.
        public static final double INTAKE_ARM_ERROR_TOLERANCE = 0.02;

        // Separate forward/reverse output limits — forward (deploy) is capped lower for safety.
        public static final double INTAKE_ARM_MAX_OUTPUT_FORWARD = 0.3; // positive motor direction
        public static final double INTAKE_ARM_MAX_OUTPUT_REVERSE = 0.6; // negative motor direction
        // Legacy alias used by PID clamp — keeps PID symmetric at the stricter forward limit.
        public static final double INTAKE_ARM_MAX_OUTPUT = INTAKE_ARM_MAX_OUTPUT_FORWARD;
        public static final double INTAKE_ARM_MAX_OUTPUT_XBOX = 1.0;

        // Software soft limits (corrected output-shaft rotations).
        // Motor output is zeroed when position is outside this range.
        public static final double INTAKE_ARM_SOFT_LIMIT_MIN = -0.15; // most-reverse allowed position
        public static final double INTAKE_ARM_SOFT_LIMIT_MAX =  1.5;  // most-forward allowed position

        public static final double INTAKE_ROLLER_IN_SPEED = 0.95;
        public static final double INTAKE_ROLLER_OUT_SPEED = -0.75;
    }
}
