// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveModule;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToAngleCommand extends Command {
  Angle angle;
  Distance distanceToTravel;
  SwerveSubsystem swerveSubsystem;

  /** Creates a new TurnToAngleCommand. */
  public TurnToAngleCommand(Angle _angle, Distance _distancetotravel, SwerveSubsystem _swerveSubsystem) {
    angle = _angle;
    distanceToTravel = _distancetotravel;
    swerveSubsystem = _swerveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_swerveSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // make a note of the drive wheel positions so that we calculate how far we have moved

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // set all the swerve modules desired state to have a low velocity
    // and face a certain direction
    for (SwerveModule swerveModule : swerveSubsystem.getSwerveDrive().getModules()) {
      SwerveModuleState desiredState =
          new SwerveModuleState(0.5, new Rotation2d(angle));
     
      swerveModule.setDesiredState(desiredState, false, true);
    }

    // Update kinematics because we are not using setModuleStates
    //kinematics.toSwerveModuleStates(new ChassisSpeeds());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // set all the swerve modules desured state to have a zero velocity
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // read information from the swerve, determine how far we have moved
    // and return true if we have moved as far as we wanted
    return false;
  }
}
