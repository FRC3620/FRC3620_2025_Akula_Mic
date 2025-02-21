package frc.robot.commandfactories;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.esefcommands.RunEndEffectorUntilCoralGone;
import frc.robot.commands.esefcommands.RunEndEffectorUntilHasCoral;
import frc.robot.commands.esefcommands.SetESEFPositionCommand;
import frc.robot.commands.esefcommands.SetElevatorPositionCommand;
import frc.robot.commands.esefcommands.SetEndEffectorSpeedCommand;
import frc.robot.commands.esefcommands.SetManualElevatorCommand;
import frc.robot.commands.esefcommands.SetShoulderPositionCommand;
import frc.robot.subsystems.esefsubsystem.ESEFPosition;
import frc.robot.subsystems.esefsubsystem.ESEFSubsystem;

public class ESEFSubsystemCommandFactory {
  ESEFSubsystem esefSubsystem;

  public ESEFSubsystemCommandFactory(ESEFSubsystem esefSubsystem) {
    this.esefSubsystem = esefSubsystem;
  }

  public void setupSmartDashboardCommands() {
    SmartDashboard.putData("Shoulder Set Position 0", new SetShoulderPositionCommand(Degrees.of(0), esefSubsystem));
    SmartDashboard.putData("Shoulder Set Position 30", new SetShoulderPositionCommand(Degrees.of(30), esefSubsystem));
    SmartDashboard.putData("Shoulder Set Position 60", new SetShoulderPositionCommand(Degrees.of(60), esefSubsystem));
    SmartDashboard.putData("Shoulder Set Position 90", new SetShoulderPositionCommand(Degrees.of(90), esefSubsystem));
    SmartDashboard.putData("Shoulder Set Position 120", new SetShoulderPositionCommand(Degrees.of(120), esefSubsystem));
    
    SmartDashboard.putData("Elevator Set Position 8", new SetElevatorPositionCommand(Inches.of(8.0), esefSubsystem));
    SmartDashboard.putData("Elevator Set Position 12", new SetElevatorPositionCommand(Inches.of(12.0), esefSubsystem));
    SmartDashboard.putData("Elevator Set Position 20", new SetElevatorPositionCommand(Inches.of(20.0), esefSubsystem));
    SmartDashboard.putData("Elevator Set Position 0", new SetElevatorPositionCommand(Inches.of(0.0), esefSubsystem));

    SmartDashboard.putData("Run EndEffector until has coral", new RunEndEffectorUntilHasCoral(0.7, esefSubsystem));
    SmartDashboard.putData("Run EndEffector until coral gone", new RunEndEffectorUntilCoralGone(0.7, esefSubsystem));
    SmartDashboard.putData("move End Effector", new SetEndEffectorSpeedCommand(0.5, esefSubsystem));

    SmartDashboard.putNumber("Elevator.ManualPosition", 5);
    SmartDashboard.putData("Elevator.ManualControl", new SetManualElevatorCommand());

    SmartDashboard.putData("ESEF 0 height, 90 shoulder", new SetESEFPositionCommand(new ESEFPosition(0, 90), esefSubsystem));
    SmartDashboard.putData("ESEF 21 height, 90 shoulder", new SetESEFPositionCommand(new ESEFPosition(21, 90), esefSubsystem));
    SmartDashboard.putData("ESEF 21 height, 60 shoulder", new SetESEFPositionCommand(new ESEFPosition(21, 60), esefSubsystem));
    SmartDashboard.putData("ESEF 21 height, 120 shoulder", new SetESEFPositionCommand(new ESEFPosition(21, 120), esefSubsystem));
    SmartDashboard.putData("ESEF 48 height, 0 shoulder", new SetESEFPositionCommand(new ESEFPosition(48, 0), esefSubsystem));
    SmartDashboard.putData("ESEF 48 height, 120 shoulder", new SetESEFPositionCommand(new ESEFPosition(48, 120), esefSubsystem));
    SmartDashboard.putData("ESEF 23 height, 120 shoulder", new SetESEFPositionCommand(new ESEFPosition(23, 120), esefSubsystem));

    SmartDashboard.putData("ESEF L1", new SetESEFPositionCommand(ESEFPosition.PresetPosition.L1.getPosition(), esefSubsystem));
    SmartDashboard.putData("ESEF L2", new SetESEFPositionCommand(ESEFPosition.PresetPosition.L2.getPosition(), esefSubsystem));
    SmartDashboard.putData("ESEF L3", new SetESEFPositionCommand(ESEFPosition.PresetPosition.L3.getPosition(), esefSubsystem));
    SmartDashboard.putData("ESEF L4", new SetESEFPositionCommand(ESEFPosition.PresetPosition.L4.getPosition(), esefSubsystem));

  }

}
