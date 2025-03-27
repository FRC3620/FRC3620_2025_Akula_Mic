package frc.robot.commands.swervedrive;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import java.util.Optional;

public class AutoAlignToAlgaeCommand extends InstantCommand {

  TaggedLogger logger = LoggingMaster.getLogger(getClass());
  int tagID = -1;

  public AutoAlignToAlgaeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    if (RobotContainer.visionSubsystem.getDoWeAlign()) {
      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          tagID = RobotContainer.visionSubsystem.getNearestTagIDRed(RobotContainer.swerveSubsystem.getPose());
        } else {
          tagID = RobotContainer.visionSubsystem.getNearestTagIDBlue(RobotContainer.swerveSubsystem.getPose());
        }
        logger.info("Detected Tag ID = {}", tagID);

        Pose2d targetPose = null;
        Pose2d startPose = null;

        if (tagID % 2 == 1) { // Odd Tag ID
          startPose = RobotContainer.visionSubsystem.getBlueStartingAlgaePose(tagID);
          targetPose = RobotContainer.visionSubsystem.getBlueAlgaePose(tagID);

          if (startPose == null) {
            logger.error("Invalid Start Pose for Tag ID = {}", tagID);
            return; // Exit if the startPose is invalid
          }
          if (targetPose == null) {
            logger.error("Invalid Target Pose for Tag ID = {}", tagID);
            return; // Exit if the targetPose is invalid
          }

          logger.info("Starting Pose = {}", startPose);
          CommandScheduler.getInstance().schedule(
              new SequentialCommandGroup(
                  new DriveToPoseCommand(RobotContainer.swerveSubsystem, startPose),
                  new DriveToPoseCommand(RobotContainer.swerveSubsystem, targetPose)));
        } else { // Even Tag ID
          targetPose = RobotContainer.visionSubsystem.getBlueAlgaePose(tagID);

          if (targetPose == null) {
            logger.error("Invalid Target Pose for Tag ID = {}", tagID);
            return; // Exit if the targetPose is invalid
          }

          CommandScheduler.getInstance().schedule(new DriveToPoseCommand(RobotContainer.swerveSubsystem, targetPose));
        }

        logger.info("Target Pose = {}", targetPose);
        RobotContainer.swerveSubsystem.setTargetPose(targetPose);

      } else {
        logger.info("No valid alliance or tag detected. AutoAligningToAlgaeCommand Stopped.");
      }
    }
  }
}
