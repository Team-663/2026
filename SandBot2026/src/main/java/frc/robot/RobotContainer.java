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
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.commands.AutoRunAgitator;
import frc.robot.commands.AutoSetTurret;
import frc.robot.commands.AutoShootBalls;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
            () -> -driverXbox.getLeftY(),
            () -> -driverXbox.getLeftX())
            .withControllerRotationAxis(driverXbox::getRightX)
            .scaleRotation(-0.5)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(1.0)
            .allianceRelativeControl(true)
            .cubeTranslationControllerAxis(true);

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
        NamedCommands.registerCommand("ShootBallsClose", AutoShootBalls.create(shooter, ShooterConstants.SHOOTER_RPM_LOW, ShooterConstants.HOOD_ANGLE_SHORT_SHOT, 5));
        NamedCommands.registerCommand("ShootBallsMid", AutoShootBalls.create(shooter, ShooterConstants.SHOOTER_RPM_MEDIUM, ShooterConstants.HOOD_ANGLE_MEDIUM_SHOT, 5));
        NamedCommands.registerCommand("DeployIntake", intake.setArmPositionCmd(Constants.IntakeConstants.INTAKE_ARM_DEPLOYED));
        NamedCommands.registerCommand("TurretCenter",   AutoSetTurret.create(shooter,  0.0, 2.0));
        NamedCommands.registerCommand("TurretLeft90",   AutoSetTurret.create(shooter, -90.0, 2.0));
        NamedCommands.registerCommand("TurretLeft40",   AutoSetTurret.create(shooter, -40.0, 2.0));
        NamedCommands.registerCommand("TurretRight40",  AutoSetTurret.create(shooter,  40.0, 2.0));
        NamedCommands.registerCommand("RunAgitator",     AutoRunAgitator.create(shooter, 0.5, 1.0));
        NamedCommands.registerCommand("RunAgitatorBackwards", AutoRunAgitator.create(shooter, -.75, 0.25));

        // Have the autoChooser pull in all PathPlanner autos as options
        autoChooser = AutoBuilder.buildAutoChooser();

        // Set the default auto (do nothing)
        autoChooser.setDefaultOption("Do Nothing", Commands.none());

        // ── PathPlanner autos ─────────────────────────────────────────────────
        autoChooser.addOption("Shoot Close Left Then Backup",  new PathPlannerAuto("ShootCloseLeftThenBackup"));
        autoChooser.addOption("Shoot Close Mid Then Backup",   new PathPlannerAuto("ShootCloseMidThenBackup"));
        autoChooser.addOption("Shoot Close Right Then Backup", new PathPlannerAuto("ShootCloseRightThenBackup"));
        autoChooser.addOption("Shoot Mid Left Then Backup",    new PathPlannerAuto("ShootMidLeftThenBackup"));
        autoChooser.addOption("Shoot Mid Right Then Backup",   new PathPlannerAuto("ShootMidRightThenBackup"));

        // ── Basic drive tests ──────────────────────────────────────────────────
        autoChooser.addOption("Drive Forward 1s", drivebase.driveForward().withTimeout(1));

        // ── Shooter tests (spin up + feed, no driving) ────────────────────────
        autoChooser.addOption("Test: Shoot Close (5s)",
            AutoShootBalls.create(shooter, ShooterConstants.SHOOTER_RPM_LOW,    ShooterConstants.HOOD_ANGLE_SHORT_SHOT,  5.0));
        autoChooser.addOption("Test: Shoot Mid (5s)",
            AutoShootBalls.create(shooter, ShooterConstants.SHOOTER_RPM_MEDIUM, ShooterConstants.HOOD_ANGLE_MEDIUM_SHOT, 5.0));
        autoChooser.addOption("Test: Shoot Long (5s)",
            AutoShootBalls.create(shooter, ShooterConstants.SHOOTER_RPM_HIGH,   ShooterConstants.HOOD_ANGLE_LONG_SHOT,   5.0));

        // ── Turret tests ──────────────────────────────────────────────────────
        autoChooser.addOption("Test: Turret Center",    AutoSetTurret.create(shooter,  0.0, 2.0));
        autoChooser.addOption("Test: Turret Left 40°",  AutoSetTurret.create(shooter, -40.0, 2.0));
        autoChooser.addOption("Test: Turret Right 40°", AutoSetTurret.create(shooter,  40.0, 2.0));

        // ── Intake tests ──────────────────────────────────────────────────────
        autoChooser.addOption("Test: Deploy Intake",
            intake.setArmPositionCmd(Constants.IntakeConstants.INTAKE_ARM_DEPLOYED));
        autoChooser.addOption("Test: Stow Intake",
            intake.setArmPositionCmd(Constants.IntakeConstants.INTAKE_ARM_STOWED));

       
        // Put the autoChooser on the SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData(field);
    }

    private void configureBindings()
    {
        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
        Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);


        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        
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

            // Driver tests arm speed manually for now with triggers
            driverXbox.leftTrigger(0.1)
            .whileTrue(intake.setArmPctOutputCmd(()->driverXbox.getLeftTriggerAxis()*0.5))
            .onFalse(intake.armOffCmd());

            driverXbox.rightTrigger(0.1)
            .whileTrue(intake.setArmPctOutputCmd(()->driverXbox.getRightTriggerAxis()*-0.5))
            .onFalse(intake.armOffCmd());

            // Driver bumpers will set the arm to "1" for Right bumper and "0 " for left bumper
            driverXbox.leftBumper()
            .onTrue(intake.setArmPositionCmd(Constants.IntakeConstants.INTAKE_ARM_STOWED));

            driverXbox.rightBumper()
            .onTrue(intake.setArmPositionCmd(Constants.IntakeConstants.INTAKE_ARM_DEPLOYED));

            
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

                            
            //X Button sets turret angle to 0
            operatorXbox.x()
                .onTrue(shooter.setTurretAngleCmd(0.0));
            
            
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
                .whileTrue(intake.setRollerPctOutputCmd(()->Constants.IntakeConstants.INTAKE_ROLLER_OUT_SPEED))
                .onFalse(intake.rollerOffCmd());

            // Right stick X controls turret
            operatorXbox.axisGreaterThan(XboxController.Axis.kRightX.value, 0.15)
                .or(operatorXbox.axisLessThan(XboxController.Axis.kRightX.value, -0.15))
                .whileTrue(shooter.setTurretPctOutputCmd(()->operatorXbox.getRightX()*Constants.ShooterConstants.TURRET_MAX_OUTPUT_XBOX))
                .onFalse(shooter.holdTurretPositionCmd());

            // Left stick Y controls hood
            operatorXbox.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.15)
                .or(operatorXbox.axisLessThan(XboxController.Axis.kLeftY.value, -0.15))
                .whileTrue(shooter.setHoodPctOutputCmd(()->operatorXbox.getLeftY()*-1.0))
                .onFalse(shooter.hoodOffCmd());

            // Operator D-pad will set the hood to specific angles for now (can be changed to buttons later if preferred)
            operatorXbox.povUp()
                .whileTrue(shooter.setHoodAngleCmd(Constants.ShooterConstants.HOOD_ANGLE_LONG_SHOT));
                operatorXbox.povDown()
                .whileTrue(shooter.setHoodAngleCmd(2.0));
                
                // Operator left POV sets the turret to 30 degrees
                operatorXbox.povLeft()
                    .onTrue(shooter.setTurretAngleCmd(Constants.ShooterConstants.TURRET_SOFT_LIMIT_REVERSE));
                operatorXbox.povRight()
                    .onTrue(shooter.setTurretAngleCmd(Constants.ShooterConstants.TURRET_SOFT_LIMIT_FORWARD));

            // Start button resets the hood encoder to 0 for debugging
            operatorXbox.start()
                .onTrue(Commands.runOnce(() -> shooter.resetHoodEncoder(), shooter));
        }
    }

    public Command getAutonomousCommand()
    {
        return autoChooser.getSelected();
    }
}
