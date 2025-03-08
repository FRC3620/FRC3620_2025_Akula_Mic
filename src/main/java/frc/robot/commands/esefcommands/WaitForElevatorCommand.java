package frc.robot.commands.esefcommands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.esefsubsystem.ESEFSubsystem;

public class WaitForElevatorCommand extends Command {
  ESEFSubsystem esefSubsystem;

  Distance closeEnough = Inches.of(2).unaryMinus();
  /** Creates a new WaitForElevatorCommand. */
  public WaitForElevatorCommand(ESEFSubsystem esefSubsystem) {
    this.esefSubsystem = esefSubsystem;
    SmartDashboard.putBoolean("frc3620/Elevator/waiting", false);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("frc3620/Elevator/waiting", true);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("frc3620/Elevator/waiting", false);
  }

  @Override
  public boolean isFinished() {
    Distance desiredHeight = esefSubsystem.getRequestedPosition().getElevatorHeight();
    Distance currentHeight = esefSubsystem.getCurrentPosition().getElevatorHeight();
    boolean done = false;
    if (currentHeight.minus(desiredHeight).gt(closeEnough)) {
      done = true;
    }
    return done;
  }
}
