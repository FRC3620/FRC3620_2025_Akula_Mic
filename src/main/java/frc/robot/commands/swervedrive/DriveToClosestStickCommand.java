// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToClosestStickCommand extends InstantCommand {

  public enum WhichStick{
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
  @Override
  public void initialize() {
      int tagID = RobotContainer.visionSubsystem.getNearestTagID(RobotContainer.swerveSubsystem.getPose());

      logger.info("Saw ID = {}", tagID);

      if(tagID > 0){

        Pose2d pose;

        if(whichStick == WhichStick.LEFT){
          pose = RobotContainer.visionSubsystem.getNearestLeftStickPose(tagID);
        }else{
          pose = RobotContainer.visionSubsystem.getNearestRightStickPose(tagID);
        } 

        logger.info("Target Pose = {}", pose);

        CommandScheduler.getInstance().schedule(RobotContainer.swerveSubsystem.driveToPoseSlow(pose));
      }

  }
}
