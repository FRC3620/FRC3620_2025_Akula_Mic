package frc.robot.commands.esefcommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.esefsubsystem.ESEFPosition;
import frc.robot.subsystems.esefsubsystem.ESEFSubsystem;

public class SetESEFPositionCommand extends InstantCommand {
  final ESEFSubsystem subsystem;
  final ESEFPosition esefPosition;

  public SetESEFPositionCommand(ESEFPosition esefPosition, ESEFSubsystem subsystem) {
    this.subsystem = subsystem;
    this.esefPosition = esefPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.setPosition(esefPosition);
  }
}
