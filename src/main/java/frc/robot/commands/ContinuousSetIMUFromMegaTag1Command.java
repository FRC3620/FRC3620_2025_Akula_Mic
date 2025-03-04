// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.RobotContainer;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ContinuousSetIMUFromMegaTag1Command extends Command {
  boolean resetIMU;
  TaggedLogger logger = LoggingMaster.getLogger(getClass());
  /** Creates a new ContinuousSetIMUFromMegaTag1Command. */
  public ContinuousSetIMUFromMegaTag1Command() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    resetIMU = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     int mostTargets = 0;
    Pose2d pose = null;
    String used = null;
    for (var cameraData : RobotContainer.visionSubsystem.getAllCameraData()) {
      PoseEstimate pe = cameraData.megaTag1.getPoseEstimate();
      if (pe != null && pe.tagCount > mostTargets) {
        mostTargets = pe.tagCount;
        pose = pe.pose;
        used = cameraData.getLimelightName();
      }
    }
    if (pose != null) {
      logger.info ("{} had {} targets, using pose {}", used, mostTargets, pose);
      if (RobotContainer.swerveSubsystem != null) {
        SwerveDrive sd = RobotContainer.swerveSubsystem.getSwerveDrive();
        Rotation3d r3d = new Rotation3d(0, 0, pose.getRotation().getRadians());

        Rotation2d before = sd.getYaw();
        sd.setGyro(r3d);
        Rotation2d after = sd.getYaw();
        logger.info ("Swerve yaw changed from {} to {}", before.getDegrees(), after.getDegrees());
        resetIMU = true;
      }
    } 
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    
  }
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
