// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.esefsubsystem;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ESEFSubsystem extends SubsystemBase {
  /** Creates a new ESEFSubsystem. */
  private ESEFShoulderMechanism shoulderMechanism;
  private ESEFElevatorMechanism elevatorMechanism;
  private ESEFEndEffectorMechanism endEffectorMechanism;
  public ESEFSubsystem() { //constructor
    shoulderMechanism = new ESEFShoulderMechanism();
    elevatorMechanism = new ESEFElevatorMechanism();
    endEffectorMechanism = new ESEFEndEffectorMechanism();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shoulderMechanism.periodic();
    elevatorMechanism.periodic();
  }

  public void setShoulderPosition(Angle position){
    shoulderMechanism.setShoulderPosition(position);
  }
  public void setElevatorPosition(Distance position){
    elevatorMechanism.setElevatorPosition(position);
  }
  public void setEndEffSpeed(double speed){
    endEffectorMechanism.setEndEffSpeed(speed);
  }

}
