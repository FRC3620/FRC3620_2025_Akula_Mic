package frc.robot.commandfactories;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

  public Command makeSetClimberPowerCommand(DoubleSupplier powerSupplier) {
    return new RunCommand(() -> {
        double power = powerSupplier.getAsDouble();
        if (Math.abs(power) > 0.1) { // Only override if joystick moves
            climberSubsystem.setClimberPower(power);
        }
    }, climberSubsystem);
  }


}
