package frc.robot.commandfactories;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommandFactory {
  ClimberSubsystem climberSubsystem;

  public ClimberCommandFactory(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
  }

  public void setupSmartDashboardCommands() {
}

  public Command makeSetClimberPowerCommand(DoubleSupplier ds) {
      return climberSubsystem.run(() -> climberSubsystem.setClimberPower(ds.getAsDouble()));
  }

}
