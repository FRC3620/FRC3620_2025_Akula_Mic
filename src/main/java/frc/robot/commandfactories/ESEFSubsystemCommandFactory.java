package frc.robot.commandfactories;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.esefcommands.RunEndEffectorUntilCoralGone;
import frc.robot.commands.esefcommands.RunEndEffectorUntilHasCoral;
import frc.robot.commands.esefcommands.SetESEFPositionCommand;
import frc.robot.commands.esefcommands.SetESEFPositionPatientCommand;
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
    SmartDashboard.putData("Run EndEffector until coral gone", new RunEndEffectorUntilCoralGone(0.9, esefSubsystem));
    SmartDashboard.putData("move End Effector", new SetEndEffectorSpeedCommand(0.5, esefSubsystem));

    SmartDashboard.putData("Bump Elevator + 1", new InstantCommand(() -> esefSubsystem.bumpElevatorHeight(Inches.of(1))));
    SmartDashboard.putData("Bump Elevator - 1", new InstantCommand(() -> esefSubsystem.bumpElevatorHeight(Inches.of(-1))));
    SmartDashboard.putData("Bump Shoulder + 2.5", new InstantCommand(() -> esefSubsystem.bumpShoulderAngle(Degrees.of(2.5))));
    SmartDashboard.putData("Bump Shoulder - 2.5", new InstantCommand(() -> esefSubsystem.bumpShoulderAngle(Degrees.of(-2.5))));

    SmartDashboard.putData("ESEF 0 height, 90 shoulder", new SetESEFPositionCommand(new ESEFPosition(0, 90), esefSubsystem));
    SmartDashboard.putData("ESEF 21 height, 90 shoulder", new SetESEFPositionCommand(new ESEFPosition(21, 90), esefSubsystem));
    SmartDashboard.putData("ESEF 21 height, 60 shoulder", new SetESEFPositionCommand(new ESEFPosition(21, 60), esefSubsystem));
    SmartDashboard.putData("ESEF 21 height, 120 shoulder", new SetESEFPositionCommand(new ESEFPosition(21, 120), esefSubsystem));
    SmartDashboard.putData("ESEF 48 height, 0 shoulder", new SetESEFPositionCommand(new ESEFPosition(48, 0), esefSubsystem));
    SmartDashboard.putData("ESEF 48 height, 120 shoulder", new SetESEFPositionCommand(new ESEFPosition(48, 120), esefSubsystem));

    SmartDashboard.putData("Patient ESEF L4", new SetESEFPositionPatientCommand(ESEFPosition.PresetPosition.L4.getPosition(), esefSubsystem));

    SmartDashboard.putData("ESEF 33 height, 90 shoulder", new SetESEFPositionCommand(new ESEFPosition(33, 90), esefSubsystem));
    SmartDashboard.putData("ESEF 55 height, 90 shoulder", new SetESEFPositionCommand(new ESEFPosition(55, 90), esefSubsystem));
    SmartDashboard.putData("ESEF 55 height, 70 shoulder", new SetESEFPositionCommand(new ESEFPosition(55, 70), esefSubsystem));


    SmartDashboard.putData("ESEF Home", new SetESEFPositionCommand(ESEFPosition.PresetPosition.Home.getPosition(), esefSubsystem));
    SmartDashboard.putData("ESEF L1", new SetESEFPositionCommand(ESEFPosition.PresetPosition.L1.getPosition(), esefSubsystem));
    SmartDashboard.putData("ESEF L2", new SetESEFPositionCommand(ESEFPosition.PresetPosition.L2.getPosition(), esefSubsystem));
    SmartDashboard.putData("ESEF L3", new SetESEFPositionCommand(ESEFPosition.PresetPosition.L3.getPosition(), esefSubsystem));
    SmartDashboard.putData("ESEF L4", new SetESEFPositionCommand(ESEFPosition.PresetPosition.L4.getPosition(), esefSubsystem));

    SmartDashboard.putData("ESEF Climber", new SetESEFPositionCommand(ESEFPosition.PresetPosition.CLIMB.getPosition(), esefSubsystem));
  }

}
