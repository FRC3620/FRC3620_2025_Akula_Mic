// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Optional;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToClosestStickCommand extends InstantCommand {

  public enum WhichStick {
    LEFT, RIGHT
  }

  WhichStick whichStick;

  TaggedLogger logger = LoggingMaster.getLogger(getClass());

  /** Creates a new DriveToPoseSlowCommand. */
  public DriveToClosestStickCommand(WhichStick whichStick) {

    this.whichStick = whichStick;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  // Initialize with a default value
  public void initialize() {
    if (RobotContainer.visionSubsystem.getDoWeAlign()) {
      Pose2d pose;
      int tagID = RobotContainer.visionSubsystem.getNearestTagID(RobotContainer.swerveSubsystem.getPose());
      logger.info("Saw ID = {}", tagID);

      // Check if tagID is within the allowed range
      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {

        if (ally.get() == Alliance.Red && tagID >= 6 && tagID <= 11) {

          if (whichStick == WhichStick.LEFT) {
            pose = RobotContainer.visionSubsystem.getNearestLeftStickPose(tagID);
          } else {
            pose = RobotContainer.visionSubsystem.getNearestRightStickPose(tagID);
          }
          RobotContainer.swerveSubsystem.setTargetPose(pose);
          logger.info("Target Pose = {}", pose);

          CommandScheduler.getInstance().schedule(new DriveToPoseCommand(RobotContainer.swerveSubsystem, pose));

        } else if (ally.get() == Alliance.Blue && tagID >= 17 && tagID <= 22) {

          if (whichStick == WhichStick.LEFT) {
            pose = RobotContainer.visionSubsystem.getNearestLeftStickPose(tagID);
          } else {
            pose = RobotContainer.visionSubsystem.getNearestRightStickPose(tagID);
          }
          RobotContainer.swerveSubsystem.setTargetPose(pose);
          logger.info("Target Pose = {}", pose);

          CommandScheduler.getInstance().schedule(new DriveToPoseCommand(RobotContainer.swerveSubsystem, pose));

        } else {

          logger.info("No reef ID seen. DriveToClosestStick Stopped.");
        }

      }
    }

  }

}
