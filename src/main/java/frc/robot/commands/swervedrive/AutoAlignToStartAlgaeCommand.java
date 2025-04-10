package frc.robot.commands.swervedrive;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.esefcommands.RunEndEffectorUntilHasAlgae;
import frc.robot.commands.esefcommands.SetESEFPositionCommand;
import frc.robot.subsystems.esefsubsystem.ESEFPosition;

import java.util.Optional;

public class AutoAlignToStartAlgaeCommand extends InstantCommand {

  TaggedLogger logger = LoggingMaster.getLogger(getClass());

  public AutoAlignToStartAlgaeCommand() {}

  @Override
  public void initialize() {
    if (!RobotContainer.visionSubsystem.getDoWeAlign()) {
      logger.info("AutoAligningToStartAlgaeCommand Stopped: Alignment not active.");
      return;
    }

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      logger.warn("Alliance not available — tagID not set. AutoAlign skipped.");
      return;
    }

    int tagID = (alliance.get() == Alliance.Red)
        ? RobotContainer.visionSubsystem.getNearestTagIDRed(RobotContainer.swerveSubsystem.getPose())
        : RobotContainer.visionSubsystem.getNearestTagIDBlue(RobotContainer.swerveSubsystem.getPose());

    logger.info("Saw ID = {}", tagID);
    SmartDashboard.putNumber("frc3620/vision/TargetAprilTag", tagID);

    if ((tagID >= 17 && tagID <= 22) || (tagID >= 6 && tagID <= 11)) {
      Pose2d startPose = RobotContainer.visionSubsystem.getAlgaeStartingPose(tagID);
      if (startPose != null) {
        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                new DriveToPoseCommand(RobotContainer.swerveSubsystem, startPose)
            )
        );
      } else {
        logger.warn("Null startPose for tagID {}", tagID);
      }
    } else {
      logger.info("TagID {} not in valid range for AutoAlignToStartAlgaeCommand.", tagID);
    }
  }
}
