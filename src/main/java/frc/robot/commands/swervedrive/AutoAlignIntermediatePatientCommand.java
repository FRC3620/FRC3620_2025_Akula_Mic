// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Optional;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignIntermediatePatientCommand extends Command {
  /** Creates a new AutoAlignIntermediatePatientCommand. */
  TaggedLogger logger = LoggingMaster.getLogger(getClass());
  int tagID = -1;
  boolean reached = false;
  Pose2d _targetPose = null;

  public AutoAlignIntermediatePatientCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
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
        Pose2d startPose = RobotContainer.swerveSubsystem.getPose(); // Initialize startPose with a valid value

        targetPose = RobotContainer.visionSubsystem.getAlgaeIntermediatePose(tagID);
        _targetPose = targetPose;

        if (targetPose == null) {
          logger.error("Invalid Target Pose for Tag ID = {}", tagID);
          return; // Exit if the targetPose is invalid
        }

        logger.info("Starting Pose = {}", startPose);
        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                new DriveToPoseCommand(RobotContainer.swerveSubsystem, targetPose)));

        /*
         * if (ally.get() == Alliance.Blue && tagID % 2 == 1) { // Odd Tag ID
         * targetPose = RobotContainer.visionSubsystem.getAlgaePose(tagID);
         * 
         * if (targetPose == null) {
         * logger.error("Invalid Target Pose for Tag ID = {}", tagID);
         * return; // Exit if the targetPose is invalid
         * }
         * 
         * logger.info("Starting Pose = {}", startPose);
         * CommandScheduler.getInstance().schedule(
         * new SequentialCommandGroup(
         * new DriveToPoseCommand(RobotContainer.swerveSubsystem, targetPose))
         * );
         * 
         * } else if(ally.get() == Alliance.Blue && tagID % 2 == 0){ // Even Tag ID
         * targetPose = RobotContainer.visionSubsystem.getAlgaePose(tagID);
         * 
         * if (targetPose == null) {
         * logger.error("Invalid Target Pose for Tag ID = {}", tagID);
         * return; // Exit if the targetPose is invalid
         * }
         * 
         * CommandScheduler.getInstance().schedule(
         * new DriveToPoseCommand(RobotContainer.swerveSubsystem, targetPose)
         * );
         * } else {
         * logger.
         * info("No valid alliance or tag detected. AutoAligningToAlgaeCommand Stopped."
         * );
         * }
         * 
         * 
         * //Below is for Algae Auto Align on the Red Alliance
         * 
         * if (ally.get() == Alliance.Red && tagID % 2 == 0) { // Even Tag ID
         * targetPose = RobotContainer.visionSubsystem.getAlgaePose(tagID);
         * 
         * if (startPose == null) {
         * logger.error("Invalid Start Pose for Tag ID = {}", tagID);
         * return; // Exit if the startPose is invalid
         * }
         * if (targetPose == null) {
         * logger.error("Invalid Target Pose for Tag ID = {}", tagID);
         * return; // Exit if the targetPose is invalid
         * }
         * 
         * logger.info("Starting Pose = {}", startPose);
         * CommandScheduler.getInstance().schedule(
         * new SequentialCommandGroup(
         * new DriveToPoseCommand(RobotContainer.swerveSubsystem, targetPose))
         * );
         * 
         * } else if(ally.get() == Alliance.Red && tagID % 2 == 1){ // Odd Tag ID
         * targetPose = RobotContainer.visionSubsystem.getAlgaePose(tagID);
         * 
         * if (targetPose == null) {
         * logger.error("Invalid Target Pose for Tag ID = {}", tagID);
         * return; // Exit if the targetPose is invalid
         * }
         * 
         * CommandScheduler.getInstance().schedule(
         * new DriveToPoseCommand(RobotContainer.swerveSubsystem, targetPose));
         * }
         * 
         * logger.info("Target Pose = {}", targetPose);
         * RobotContainer.swerveSubsystem.setTargetPose(targetPose);
         * 
         * } else {
         * logger.
         * info("No valid alliance or tag detected. AutoAligningToAlgaeCommand Stopped."
         * );
         * }
         */
      }
    }
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d error = _targetPose.relativeTo(RobotContainer.swerveSubsystem.getPose()); 
    double xError = Math.abs(error.getX());
    double yError = Math.abs(error.getY());
    double rotError = Math.abs(error.getRotation().getDegrees());
    if ((xError<0.05) && (yError<0.05) && (rotError<5)){
      reached = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    reached = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(reached){
      return true;
    }
    return false;
  }
}
