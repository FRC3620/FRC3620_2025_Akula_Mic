package frc.robot.subsystems.esefsubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ESEFPosition {
  final Distance elevatorHeight;
  final Angle shoulderAngle;

  /**
   * Enum for predefined arm positions
   */
  public enum PresetPosition {
    Home(0, 90),        // Fully retracted, shoulder at 90°
    Barge(0, 90),        // Fully retracted, shoulder at 90°
    StationPickup(0, 90),        // Fully retracted, shoulder at 90°
    L1(0, 80),        // Fully retracted, shoulder at 90°
    L2(12, 63),     // Low scoring position
    L3(31, 57),     // Mid scoring position
    L4(57.5, 63.25),    // High scoring position
    CLIMB(16.9, 142),
    AlgaeL2(8, 50),
    AlgaeL3(20, 50);      // Position for climbing

    private final ESEFPosition position;

    PresetPosition(double elevatorHeightInInches, double shoulderAngleInDegrees) {
      this.position = new ESEFPosition(elevatorHeightInInches, shoulderAngleInDegrees);
    }

    public ESEFPosition getPosition() {
      return position;
    }
  }

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
