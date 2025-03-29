// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.esefcommands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.esefsubsystem.ESEFPosition;
import frc.robot.subsystems.esefsubsystem.ESEFSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetESEFPositionPatientCommand extends Command {
  /** Creates a new SetESEFPositionPatientCommand. */
  final ESEFSubsystem subsystem;
  final ESEFPosition esefPosition;

  boolean reached = false;

  public SetESEFPositionPatientCommand(ESEFPosition esefPosition, ESEFSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.esefPosition = esefPosition;
    setName("SetESEFPositionPatientCommand: " + esefPosition);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.setPosition(esefPosition);
    SmartDashboard.putBoolean("PatientStatus", reached);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ActualEl = subsystem.getElevatorPosition().in(Inches);
    double RequestedEl = esefPosition.getElevatorHeight().in(Inches);
    double ActualSh = subsystem.getShoulderPosition().in(Degrees);
    double RequestedSh = esefPosition.getShoulderAngle().in(Degrees);

    if((Math.abs(ActualEl-RequestedEl)<0.5) && (Math.abs(ActualSh-RequestedSh)<1)){
      reached = true;
    }
    SmartDashboard.putBoolean("PatientStatus", reached);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    reached = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(reached){
      return true;
    }else{
    return false;
    }
  }
}
