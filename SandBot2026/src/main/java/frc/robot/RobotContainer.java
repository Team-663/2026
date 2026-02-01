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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Shooter;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer
{

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final         CommandXboxController driverXbox = new CommandXboxController(0);
    // The robot's subsystems and commands are defined here...
    public final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));

    private final Shooter shooter = new Shooter();
    // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
    private final SendableChooser<Command> autoChooser;
    public final Field2d field = new Field2d();

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driverXbox.getLeftY() * -1,
            () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(driverXbox::getRightX)
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

        if (RobotBase.isSimulation())
        {
            drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
        } else
        {
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }

        if (Robot.isSimulation())
        {
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

        }
        if (DriverStation.isTest())
        {
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

            driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            driverXbox.back().whileTrue(drivebase.centerModulesCommand());
            driverXbox.leftBumper().onTrue(Commands.none());
            driverXbox.rightBumper().onTrue(Commands.none());
        } else
        {
            driverXbox.a()
                        .onTrue(shooter.shooterSetRPMCommand(Constants.ShooterConstants.SHOOTER_RPM_LOW))
                        .onFalse(shooter.shooterOffCommand());
            driverXbox.b()
                        .onTrue(shooter.shooterSetRPMCommand(Constants.ShooterConstants.SHOOTER_RPM_MEDIUM))
                        .onFalse(shooter.shooterOffCommand());
            driverXbox.y()
                        .onTrue(shooter.shooterSetRPMCommand(Constants.ShooterConstants.SHOOTER_RPM_HIGH))
                        .onFalse(shooter.shooterOffCommand());
        
            driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            driverXbox.back().whileTrue(Commands.none());
            driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverXbox.rightBumper().onTrue(Commands.none());
        }
    }

    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
