package frc.robot.commands.esefcommands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* This command will read inputs from ShuffleBoard and set the shooter speed and angle appropriately. The command will be initiated/ended from the Dashboard  */

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.esefsubsystem.ESEFSubsystem;

public class SetManualElevatorCommand extends Command {

  ESEFSubsystem esef = RobotContainer.esefSubsystem;

  /** Creates a new ManualShooterSpeedAndAngleCommand. */
  public SetManualElevatorCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(esef);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // We need to read the data from SmartDashboard before we set the intiial Speed and Angle

    double elevatorPos = SmartDashboard.getNumber("Elevator.ManualPosition", 5);

    esef.setElevatorPosition(Inches.of(elevatorPos));


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        // We need to read the data from SmartDashboard before we set the intiial Speed and Angle


        double elevatorPos = SmartDashboard.getNumber("Elevator.ManualPosition", 5);

        esef.setElevatorPosition(Inches.of(elevatorPos));


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
