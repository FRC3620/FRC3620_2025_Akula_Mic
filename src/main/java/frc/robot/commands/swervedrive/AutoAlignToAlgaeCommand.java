package frc.robot.commands.swervedrive;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.esefcommands.RunEndEffectorUntilHasAlgae;
import frc.robot.commands.esefcommands.SetESEFPositionCommand;
import frc.robot.subsystems.esefsubsystem.ESEFPosition;

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
      Pose2d targetPose = null;
      Pose2d startPose = RobotContainer.swerveSubsystem.getPose(); // Initialize startPose with a valid value

      if (tagID <= 22 && tagID >= 17 && tagID % 2 == 1) { // Odd Tag ID
        startPose = RobotContainer.visionSubsystem.getAlgaeStartingPose(tagID);
        targetPose = RobotContainer.visionSubsystem.getAlgaePose(tagID);

        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                new DriveToPoseCommand(RobotContainer.swerveSubsystem, startPose).withTimeout(2.5),
                new DriveToPoseCommand(RobotContainer.swerveSubsystem, targetPose)));

      } else if (tagID <= 22 && tagID >= 17 && tagID % 2 == 0) {

        // Even Tag ID
        targetPose = RobotContainer.visionSubsystem.getAlgaePose(tagID);
        startPose = RobotContainer.visionSubsystem.getAlgaeStartingPose(tagID);

        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                new DriveToPoseCommand(RobotContainer.swerveSubsystem, startPose).withTimeout(2.5),
                new DriveToPoseCommand(RobotContainer.swerveSubsystem, targetPose)));
      }

      // Below is for Algae Auto Align on the Red Alliance

      if (tagID >= 6 && tagID <= 11 && tagID % 2 == 0) { // Even Tag ID
        targetPose = RobotContainer.visionSubsystem.getAlgaePose(tagID);
        startPose = RobotContainer.visionSubsystem.getAlgaeStartingPose(tagID);

        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                new DriveToPoseCommand(RobotContainer.swerveSubsystem, startPose).withTimeout(2.5),
                new DriveToPoseCommand(RobotContainer.swerveSubsystem, targetPose)));

      } else if (tagID >= 6 && tagID <= 11 && tagID % 2 == 1) {
        targetPose = RobotContainer.visionSubsystem.getAlgaePose(tagID);
        startPose = RobotContainer.visionSubsystem.getAlgaeStartingPose(tagID);

        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                new DriveToPoseCommand(RobotContainer.swerveSubsystem, startPose).withTimeout(2.5),
                new DriveToPoseCommand(RobotContainer.swerveSubsystem, targetPose)));
      }

    } else {
      logger.info("AutoAligningToAlgaeCommand Stopped.");
    }
  }
}