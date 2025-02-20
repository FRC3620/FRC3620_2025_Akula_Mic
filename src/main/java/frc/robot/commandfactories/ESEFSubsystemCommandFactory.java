package frc.robot.commandfactories;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.esefcommands.SetElevatorPositionCommand;
import frc.robot.commands.esefcommands.SetEndEffectorSpeedCommand;
import frc.robot.commands.esefcommands.SetManualElevatorCommand;
import frc.robot.commands.esefcommands.SetShoulderPositionCommand;
import frc.robot.subsystems.esefsubsystem.ESEFSubsystem;

public class ESEFSubsystemCommandFactory {
  ESEFSubsystem esefSubsystem;

  public ESEFSubsystemCommandFactory(ESEFSubsystem esefSubsystem) {
    this.esefSubsystem = esefSubsystem;
  }

  public void setupSmartDashboardCommands() {
    // SmartDashboard.putData("Shoulder.P1", new SetShoulderPositionCommand(null, null));
    SmartDashboard.putData("ShoulderSetPosition1", new SetShoulderPositionCommand(Degrees.of(0), esefSubsystem));
    SmartDashboard.putData("ShoulderSetPosition2", new SetShoulderPositionCommand(Degrees.of(30), esefSubsystem));
    SmartDashboard.putData("ShoulderSetPosition3", new SetShoulderPositionCommand(Degrees.of(60), esefSubsystem));
    SmartDashboard.putData("ShoulderSetPosition4", new SetShoulderPositionCommand(Degrees.of(90), esefSubsystem));

    SmartDashboard.putData("ElevatorSetPosition1", new SetElevatorPositionCommand(Inches.of(8.0), esefSubsystem));
    SmartDashboard.putData("ElevatorSetPosition2", new SetElevatorPositionCommand(Inches.of(12.0), esefSubsystem));
    SmartDashboard.putData("ElevatorSetPosition3", new SetElevatorPositionCommand(Inches.of(20.0), esefSubsystem));
    SmartDashboard.putData("ElevatorSetPositionHome", new SetElevatorPositionCommand(Inches.of(0.0), esefSubsystem));
    SmartDashboard.putData("move End Effector", new SetEndEffectorSpeedCommand(0.5, esefSubsystem));

    SmartDashboard.putNumber("Elevator.ManualPosition", 5);
    SmartDashboard.putData("Elevator.ManualControl", new SetManualElevatorCommand());
  }

}
