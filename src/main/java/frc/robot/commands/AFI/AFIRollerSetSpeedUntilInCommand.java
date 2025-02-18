// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AFI;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AFISubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AFIRollerSetSpeedUntilInCommand extends Command {
  /** Creates a new AFIRollerSetSpeedCommand. */
  double speed;
  AFISubsystem afiSubsystem;
  Timer timer;
  public boolean hasAlgae = false;

  public AFIRollerSetSpeedUntilInCommand(double _speed, AFISubsystem _afiSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    speed = _speed;
    afiSubsystem = _afiSubsystem;
    addRequirements(afiSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    afiSubsystem.setAFIRollerSpeed(speed);
    double t = timer.get();
    System.out.println(t);
    SmartDashboard.putNumber("frc3620/AFI/algaeTimer", timer.get());
    if (timer.hasElapsed(0.5)) {
      if (afiSubsystem.measuredRollerSpeed < 20) {
        hasAlgae = true;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    afiSubsystem.setAFIRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (hasAlgae) {
      return true;
    } else {
      return false;
    }
  }
}
