// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.esefcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.esefsubsystem.ESEFSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunEndEffectorUntilHasCoral extends Command {
  double speed;
  ESEFSubsystem esefSubsystem;
  /** Creates a new SetEndEffectorSpeedCommand. */
  public RunEndEffectorUntilHasCoral(double _speed, ESEFSubsystem subsystem) {
    esefSubsystem = subsystem;
    speed = _speed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(esefSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    esefSubsystem.setEndEffSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    esefSubsystem.setEndEffSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    esefSubsystem.setEndEffSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return esefSubsystem.hasCoral();
  }
}
