package frc.robot.subsystems.esefsubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ESEFPosition {
  final Distance elevatorHeight;
  final Angle shoulderAngle;

  public ESEFPosition(Distance elevatorHeight, Angle shoulderAngle) {
    this.elevatorHeight = elevatorHeight;
    this.shoulderAngle = shoulderAngle;
  }

  public ESEFPosition(double elevatorHeightInInches, double shoulderAngleInDegrees) {
    this(Inches.of(elevatorHeightInInches), Degrees.of(shoulderAngleInDegrees));
  }

  public Distance getElevatorHeight() {
    return elevatorHeight;
  }

  public Angle getShoulderAngle() {
    return shoulderAngle;
  }

  public String toString() {
    return elevatorHeight.in(Inches) + "in, " + shoulderAngle.in(Degrees) + "d";
  }

}
