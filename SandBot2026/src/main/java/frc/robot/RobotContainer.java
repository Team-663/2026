// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.SpinShooter;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer
{

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final CommandXboxController driverXbox = new CommandXboxController(0);
    final CommandXboxController operatorXbox = new CommandXboxController(1);
    // The robot's subsystems and commands are defined here...
    public final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));

    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
    private final SendableChooser<Command> autoChooser;
    public final Field2d field = new Field2d();

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driverXbox.getLeftY(),
            () -> driverXbox.getLeftX())
            .withControllerRotationAxis(driverXbox::getRightX)
            .scaleRotation(-1)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative
     * input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
            driverXbox::getRightY)
            .headingWhile(true);

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative
     * input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> -driverXbox.getLeftY(),
            () -> -driverXbox.getLeftX())
            .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                    2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
            .withControllerHeadingAxis(() -> Math.sin(
                    driverXbox.getRawAxis(
                            2) *
                            Math.PI)
                    *
                    (Math.PI *
                            2),
                    () -> Math.cos(
                            driverXbox.getRawAxis(
                                    2) *
                                    Math.PI)
                            *
                            (Math.PI *
                                    2))
            .headingWhile(true)
            .translationHeadingOffset(true)
            .translationHeadingOffset(Rotation2d.fromDegrees(
                    0));

    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        // Create the NamedCommands that will be used in PathPlanner
        NamedCommands.registerCommand("test", Commands.print("I EXIST"));

        // Have the autoChooser pull in all PathPlanner autos as options
        autoChooser = AutoBuilder.buildAutoChooser();

        // Set the default auto (do nothing)
        autoChooser.setDefaultOption("Do Nothing", Commands.none());

        // Add a simple auto option to have the robot drive forward for 1 second then
        // stop
        autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

        // Put the autoChooser on the SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData(field);
    }

    private void configureBindings()
    {
        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
        //Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        //        driveDirectAngle);
        Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
        //Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        //        driveDirectAngleKeyboard);

        //shooter.setDefaultCommand(shooter.shooterOffCmd());
        //intake.setDefaultCommand(intake.intakeOffCommand());


        //if (RobotBase.isSimulation())
        //{
        //    drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
        //} else
        //{
            //drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        //}

        if (Robot.isSimulation())
        {
                /*
            Pose2d target = new Pose2d(new Translation2d(1, 4),
                    Rotation2d.fromDegrees(90));
            // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
            driveDirectAngleKeyboard.driveToPose(() -> target,
                    new ProfiledPIDController(5,
                            0,
                            0,
                            new Constraints(5, 2)),
                    new ProfiledPIDController(5,
                            0,
                            0,
                            new Constraints(Units.degreesToRadians(360),
                                    Units.degreesToRadians(180))));
            driverXbox.start()
                    .onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
            driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                    () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

            // driverXbox.b().whileTrue(
            // drivebase.driveToPose(
            // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
            // );
            */

        }
        if (DriverStation.isTest())
        {
            //drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

            //driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            //driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            ///driverXbox.back().whileTrue(drivebase.centerModulesCommand());
            //driverXbox.leftBumper().onTrue(Commands.none());
            //driverXbox.rightBumper().onTrue(Commands.none());
        } else
        {
            
            driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));
            // Reset the intake encoder zero offset to the current position when the back button is pressed, allowing it to be rezeroed if it drifts over time. Does not require redeploying code.
            driverXbox.back().onTrue(intake.rezeroArmCmd());
            driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverXbox.rightBumper().onTrue(Commands.none());

            // Driver tests arm speed manually for now with triggers
            driverXbox.leftTrigger(0.1)
            .whileTrue(intake.setArmPctOutputCmd(()->driverXbox.getLeftTriggerAxis()*0.5))
            .onFalse(intake.armOffCmd());

            driverXbox.rightTrigger(0.1)
            .whileTrue(intake.setArmPctOutputCmd(()->driverXbox.getRightTriggerAxis()*-0.5))
            .onFalse(intake.armOffCmd());

            // Driver bumpers will set the arm to "1" for Right bumper and "0 " for left bumper
            driverXbox.leftBumper()
            .onTrue(intake.setArmPositionCmd(0.0));

            driverXbox.rightBumper()
            .onTrue(intake.setArmPositionCmd(1.0));

            
            // A button is for short shots
            operatorXbox.a()
                        //.whileTrue(shooter.shooterSetRPMCmd(Constants.ShooterConstants.SHOOTER_RPM_LOW))
                        .whileTrue(new SpinShooter(shooter, Constants.ShooterConstants.SHOOTER_RPM_LOW, Constants.ShooterConstants.HOOD_ANGLE_SHORT_SHOT))
                        .onFalse(shooter.shooterOffCmd());
            operatorXbox.b()
                        .whileTrue(new SpinShooter(shooter, Constants.ShooterConstants.SHOOTER_RPM_MEDIUM, Constants.ShooterConstants.HOOD_ANGLE_MEDIUM_SHOT))
                        .onFalse(shooter.shooterOffCmd());
            operatorXbox.y()
                        .whileTrue(new SpinShooter(shooter, Constants.ShooterConstants.SHOOTER_RPM_HIGH, Constants.ShooterConstants.HOOD_ANGLE_LONG_SHOT))
                        .onFalse(shooter.shooterOffCmd());

            //X Button undefined for now
            
            
            // Right trigger/bumpers are for the KICKER/AGITATOR
            // Trigger to control shooting
            operatorXbox.rightTrigger(0.1)
                .whileTrue(new ShootBalls(shooter))
                .onFalse(Commands.runOnce(() -> {
                    shooter.setKickerPctOutput(0.0);
                    shooter.setAgitatorPctOutput(0.0);
                })); // no subsystem requirement — must not interrupt SpinShooter

            // Right bumper will drive both kicker and agitator backwards in case something is stuck    
           operatorXbox.rightBumper()
                .whileTrue(Commands.run(() -> {
                    shooter.setKickerPctOutput(-Constants.ShooterConstants.KICKER_FEED_SPEED);
                    shooter.setAgitatorPctOutput(-Constants.ShooterConstants.AGITATOR_FEED_SPEED);
                }, shooter))
                .onFalse(Commands.runOnce(() -> {
                    shooter.setKickerPctOutput(0.0);
                    shooter.setAgitatorPctOutput(0.0);
                }, shooter));
                
            // Left trigger is for INTAKE ROLLERS 
            operatorXbox.leftTrigger(0.1)
                .whileTrue(intake.setRollerPctOutputCmd(()->operatorXbox.getLeftTriggerAxis()))
                .onFalse(intake.rollerOffCmd());

            // Left bumper will reverse the intake rollers in case of a jam
            operatorXbox.leftBumper()
                .whileTrue(intake.setRollerPctOutputCmd(()-> -Constants.IntakeConstants.INTAKE_ROLLER_OUT_SPEED))
                .onFalse(intake.rollerOffCmd());

            // Right stick X controls turret
            operatorXbox.axisGreaterThan(XboxController.Axis.kRightX.value, 0.15)
                .or(operatorXbox.axisLessThan(XboxController.Axis.kRightX.value, -0.15))
                .whileTrue(shooter.setTurretPctOutputCmd(()->operatorXbox.getRightX()*-1.0))
                .onFalse(shooter.turretOffCmd());

            // Left stick Y controls hood
            operatorXbox.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.15)
                .or(operatorXbox.axisLessThan(XboxController.Axis.kLeftY.value, -0.15))
                .whileTrue(shooter.setHoodPctOutputCmd(()->operatorXbox.getLeftY()*-1.0))
                .onFalse(shooter.hoodOffCmd());

            // Operator D-pad will set the hood to specific angles for now (can be changed to buttons later if preferred)
            operatorXbox.povUp()
                .whileTrue(shooter.setHoodAngleCmd(Constants.ShooterConstants.HOOD_ANGLE_LONG_SHOT));
            operatorXbox.povRight()
                .whileTrue(shooter.setHoodAngleCmd(Constants.ShooterConstants.HOOD_ANGLE_MEDIUM_SHOT));
            operatorXbox.povDown()
                .whileTrue(shooter.setHoodAngleCmd(2.0));

            // Start button resets the hood encoder to 0 for debugging
            operatorXbox.start()
                .onTrue(Commands.runOnce(() -> shooter.resetHoodEncoder(), shooter));
        }
    }

    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
