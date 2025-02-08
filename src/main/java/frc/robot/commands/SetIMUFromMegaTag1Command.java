package frc.robot.commands;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import swervelib.SwerveDrive;

public class SetIMUFromMegaTag1Command extends InstantCommand {
  TaggedLogger logger = LoggingMaster.getLogger(getClass());
  public SetIMUFromMegaTag1Command() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int mostTargets = 0;
    Pose2d pose = null;
    String used = null;
    for (var cameraData : RobotContainer.visionSubsystem.getAllCameraData()) {
      PoseEstimate pe = cameraData.megaTag1.getPoseEstimate();
      if (pe.tagCount > mostTargets) {
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
      }
    } else {
      logger.info ("no cameras had targets!");
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
