package frc.robot.subsystems.esefsubsystem;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ESEFPosition {
  final Distance elevatorHeight;
  final Angle shoulderAngle;

  public ESEFPosition(Distance elevatorHeight, Angle shoulderAngle) {
    this.elevatorHeight = elevatorHeight;
    this.shoulderAngle = shoulderAngle;
  }

  public Distance getElevatorHeight() {
    return elevatorHeight;
  }

  public Angle getShoulderAngle() {
    return shoulderAngle;
  }

  public String toString() {
    return elevatorHeight + "m, " + shoulderAngle + "d";
  }

}
