// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToCoralCommand extends Command {

  VisionSubsystem visionSubsystem;

  double kP_drive = 0.1;
  double kP_turn = 0.02;
  double kMinDrive = 0.1;
  double kMinTurn = 0.05;

  double desiredTX = 23.85;
  double desiredTY = 25.25;
  double desiredTa = 3.65;

  double tx;
  double ty;
  double ta;
  

  /** Creates a new DriveToCoral. */
  public DriveToCoralCommand() {
    this.visionSubsystem = RobotContainer.visionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tx = 0;
    ty = 0;
    ta = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = visionSubsystem.getTx();
    ty = visionSubsystem.getTy();
    ta = visionSubsystem.getTa();

    double errorTurn = desiredTX - tx;
    double errorDrive = desiredTY - ty;

    double driveSpeed = kP_drive * errorDrive;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
