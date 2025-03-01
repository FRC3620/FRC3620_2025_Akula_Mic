// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ButtonBox;
import frc.robot.ButtonBox.CommandPair;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HankPullTheTriggerCommand extends Command {

  CommandPair commandPair;
  ButtonBox buttonBox;

  /** Creates a new HankPullTheTriggerCommand. */
  public HankPullTheTriggerCommand(ButtonBox _buttonBox) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.buttonBox = _buttonBox;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    commandPair = buttonBox.getSelectedCommandPair();
    if (commandPair == null)
      return;
    Command firstCommand = commandPair.getFirstCommand();
    if (firstCommand == null)
      return;
    //CommandScheduler.getInstance().schedule(firstCommand);
    firstCommand.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (commandPair == null)
      return;
    Command secondCommand = commandPair.getSecondCommand();
    if (secondCommand == null)
      return;
    //CommandScheduler.getInstance().schedule(secondCommand);
    secondCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
