package frc.robot.commandfactories;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SetClimberPostionCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommandFactory {
  ClimberSubsystem climberSubsystem;

  public ClimberCommandFactory(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
  }

  public void setupSmartDashboardCommands() {
    SmartDashboard.putData("climber:p1", new SetClimberPostionCommand(ClimberSubsystem.pos1, climberSubsystem));
    SmartDashboard.putData("climber:p2", new SetClimberPostionCommand(ClimberSubsystem.pos2, climberSubsystem));
  }

  public Command makeSetClimberPowerCommand(DoubleSupplier ds) {
      return climberSubsystem.run(() -> climberSubsystem.setClimberPower(ds.getAsDouble()));
  }

}
