package frc.robot.commandfactories;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.SetClimberPostionCommand;
>>>>>>> ESEFTuning
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommandFactory {
  ClimberSubsystem climberSubsystem;

  public ClimberCommandFactory(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
  }

  public void setupSmartDashboardCommands() {
}

  public Command makeSetClimberPowerCommand(DoubleSupplier powerSupplier) {
    return new RunCommand(() -> {
        double power = powerSupplier.getAsDouble();
        if (Math.abs(power) > 0.1) { // Only override if joystick moves
            climberSubsystem.setClimberPower(power);
        }
    }, climberSubsystem);
  }


}
