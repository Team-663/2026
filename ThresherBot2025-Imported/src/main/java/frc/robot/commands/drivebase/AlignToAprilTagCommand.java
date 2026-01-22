// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import java.util.Arrays;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem; // Use the provided SwerveSubsystem
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToAprilTagCommand extends Command
{
  private final SwerveSubsystem m_swerveSubsystem; // Use SwerveSubsystem
  private final SwerveDrive swerve;
  private final int tagID;

  private final PIDController xPID = new PIDController(2.5, 0, 0);
  private final PIDController yPID = new PIDController(2.5, 0, 0);
  private final PIDController rotPID = new PIDController(4.0, 0, 0);
  private final int m_targetDist;


  public AlignToAprilTagCommand(SwerveSubsystem swerveSubsystem, int tagID, int targetDist)
  {
    this.m_swerveSubsystem = swerveSubsystem;
    this.swerve = m_swerveSubsystem.getSwerveDrive();;
    this.tagID = tagID;
    this.m_targetDist = targetDist;

    xPID.setTolerance(0.05);
    yPID.setTolerance(0.05);
    rotPID.setTolerance(Math.toRadians(2));

    addRequirements(m_swerveSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    LimelightHelpers.PoseEstimate vision =
    LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    boolean seesTag =
      vision != null &&
      vision.tagCount > 0 &&
      Arrays.stream(vision.rawFiducials)
          .anyMatch(f -> f.id == tagID);

    ChassisSpeeds speeds;

    if (seesTag) {
        speeds = visionAlign(vision);
    } else {
        speeds = odometryAlign();
    }

    swerve.driveFieldOriented(speeds);
  }

 private ChassisSpeeds visionAlign(LimelightHelpers.PoseEstimate vision)
 {

    // Use camera-space pose
    double[] pose =
        LimelightHelpers.getTargetPose_CameraSpace("limelight");

    double xError = pose[0] - m_targetDist;
    double yError = pose[1];
    double yawError = Math.toRadians(pose[5]);

    double vx = xPID.calculate(xError, 0);
    double vy = yPID.calculate(yError, 0);
    double omega = rotPID.calculate(yawError, 0);

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        vx, vy, omega,
        swerve.getPose().getRotation()
    );
  }

  private ChassisSpeeds odometryAlign()
  {

      Pose2d robotPose = swerve.getPose();
      //BAFR: TODO
      //Pose2d tagPose = AprilTagFieldLayout
      //    .loadFromResource(AprilTagFields.k2025ReefscapeWelded)
      //    .getTagPose(tagID)
      //    .get()
      //    .toPose2d();
      /*
      Pose2d desiredPose = tagPose.transformBy(
          new Transform2d(
              new Translation2d(-m_targetDist, 0),
              Rotation2d.fromDegrees(180)
          )
      );
      

      double vx = xPID.calculate(robotPose.getX(), desiredPose.getX());
      double vy = yPID.calculate(robotPose.getY(), desiredPose.getY());
      double omega = rotPID.calculate(
          robotPose.getRotation().getRadians(),
          desiredPose.getRotation().getRadians()
      );
      */
      double vx = 0.0;
      double vy = 0.0;
      double omega = 0.0;
      return ChassisSpeeds.fromFieldRelativeSpeeds(
          vx, vy, omega,
          robotPose.getRotation()
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_swerveSubsystem.stopSwerveDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint();
  }
}
