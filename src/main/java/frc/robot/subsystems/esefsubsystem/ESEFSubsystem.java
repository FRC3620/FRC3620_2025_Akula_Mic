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
  private ESEFPositionController positionController;

  public ESEFSubsystem() { //constructor
    shoulderMechanism = new ESEFShoulderMechanism();
    elevatorMechanism = new ESEFElevatorMechanism();
    endEffectorMechanism = new ESEFEndEffectorMechanism();
    positionController = new ESEFPositionController(elevatorMechanism, shoulderMechanism);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shoulderMechanism.periodic();
    elevatorMechanism.periodic();
    positionController.periodic();
    endEffectorMechanism.periodic();
  }

  public void setPosition(ESEFPosition position) {
    positionController.setPosition(position);
  }

  public void setShoulderPosition(Angle position){
    shoulderMechanism.setSetpoint(position);
  }
  public Angle getShoulderPosition(){
    return shoulderMechanism.getCurrentAngle();
  }
  public void bumpShoulderAngle(Angle delta) {
    positionController.bumpShoulderAngle(delta);
  }
  public void bumpElevatorHeight(Distance delta) {
    positionController.bumpElevatorHeight(delta);
  }
  public void setElevatorPosition(Distance position){
    elevatorMechanism.setSetpoint(position);
  }
  public Distance getElevatorPosition(){
    return elevatorMechanism.getCurrentHeight();
  }
  public void setEndEffSpeed(double speed){
    endEffectorMechanism.setEndEffSpeed(speed);
  }
  public boolean hasCoral() {
    return endEffectorMechanism.hasCoral();
  }
  public double getEndEffectorVelocity() {
    return endEffectorMechanism.getEndEffectorVelocity();
  }
  public void callReverseESEFCalibration() {
    elevatorMechanism.ESEFRecalibration();
  }
}
