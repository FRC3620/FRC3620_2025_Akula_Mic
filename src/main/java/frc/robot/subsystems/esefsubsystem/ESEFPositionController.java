package frc.robot.subsystems.esefsubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ESEFPositionController {
  /* --------------------------------------------------------------------- */
  /* Don't touch anything from here to where it says                       */
  /* "You can add and modify the code below."                              */
  /* --------------------------------------------------------------------- */
  final ESEFElevatorMechanism elevatorMechanism;
  final ESEFShoulderMechanism shoulderMechanism;
  
  ESEFPosition ultimateSetpoint;
  ESEFPosition intermediateSetpoint;

  ESEFMech ultimateSetpointMech = new ESEFMech();
  ESEFMech intermediateSetpointMech = new ESEFMech();
  ESEFMech actualPositionMech = new ESEFMech();

  double height_breakpoint_in_meters = 0.5;
  double shoulder_breakpoint_in_degrees_low = 80;
  double shoulder_breakpoint_in_degrees_high = 110;

  public ESEFPositionController(ESEFElevatorMechanism elevatorMechanism, ESEFShoulderMechanism shoulderMechanism) {
    this.elevatorMechanism = elevatorMechanism;
    this.shoulderMechanism = shoulderMechanism;

    intermediateSetpoint = new ESEFPosition(Meters.of(0), Degrees.of(0));

    SmartDashboard.putData("frc3620/ESEF/setpointMech", ultimateSetpointMech.getMech());
    SmartDashboard.putData("frc3620/ESEF/intermediateSetpointMech", intermediateSetpointMech.getMech());
    SmartDashboard.putData("frc3620/ESEF/actualPositionMech", actualPositionMech.getMech());
  }

  ESEFPosition limitedESEFPosition(Distance d, Angle a) {
    double d_meters = d.in(Meters);
    double a_degrees = a.in(Degrees);
    if (d_meters < height_breakpoint_in_meters) {
      a_degrees = MathUtil.clamp(a_degrees, 80, 100);
    }
    d_meters = MathUtil.clamp(d_meters, 0, 2);
    return new ESEFPosition(Meters.of(d_meters), Degrees.of(a_degrees));
  }

  void updateDashboardForUltimate() {
    SmartDashboard.putNumber("frc3620/ESEF/ultimate.e", ultimateSetpoint.elevatorHeight.in(Meters));
    SmartDashboard.putNumber("frc3620/ESEF/ultimate.s", ultimateSetpoint.shoulderAngle.in(Degrees));
  }

  void updateDashboardForIntermediate() {
    SmartDashboard.putNumber("frc3620/ESEF/intermediate.e", intermediateSetpoint.elevatorHeight.in(Meters));
    SmartDashboard.putNumber("frc3620/ESEF/intermediate.s", intermediateSetpoint.shoulderAngle.in(Degrees));
  }

  public void setPosition (ESEFPosition position) {
    ultimateSetpoint = limitedESEFPosition(position.elevatorHeight, position.shoulderAngle);
    updateDashboardForUltimate();
    ultimateSetpointMech.setShoulderAngle(ultimateSetpoint.shoulderAngle);
    ultimateSetpointMech.setElevatorHeight(ultimateSetpoint.elevatorHeight);
    recalculate();
  }

  public void bumpElevatorHeight(Distance delta) {
    ultimateSetpoint = limitedESEFPosition(ultimateSetpoint.elevatorHeight.plus(delta), ultimateSetpoint.shoulderAngle);
    updateDashboardForUltimate();
    ultimateSetpointMech.setElevatorHeight(ultimateSetpoint.elevatorHeight);
    recalculate();
  }

  public void bumpShoulderAngle(Angle delta) {
    ultimateSetpoint = limitedESEFPosition(ultimateSetpoint.elevatorHeight, ultimateSetpoint.shoulderAngle.plus(delta));
    updateDashboardForUltimate();
    ultimateSetpointMech.setShoulderAngle(ultimateSetpoint.shoulderAngle);
    recalculate();
  }

  void setElevatorHeightSetpoint(Distance d) {
    intermediateSetpoint = new ESEFPosition(d, intermediateSetpoint.shoulderAngle);
    updateDashboardForIntermediate();
    elevatorMechanism.setSetpoint(d);
    intermediateSetpointMech.setElevatorHeight(d);
  }

  void setShoulderAngleSetpoint(Angle a) {
    intermediateSetpoint = new ESEFPosition(intermediateSetpoint.elevatorHeight, a);
    updateDashboardForIntermediate();
    shoulderMechanism.setSetpoint(a);
    intermediateSetpointMech.setShoulderAngle(a);
  }

  public void periodic() {
    Distance currentHeight = elevatorMechanism.getCurrentHeight();
    Angle currentShoulderAngle = shoulderMechanism.getCurrentAngle();
    SmartDashboard.putNumber("frc3620/ESEF/actual.e", currentHeight.in(Meters));
    SmartDashboard.putNumber("frc3620/ESEF/actual.s", currentShoulderAngle.in(Degrees));

    actualPositionMech.setElevatorHeight(currentHeight);
    actualPositionMech.setShoulderAngle(currentShoulderAngle);
    tweakIntermediateSetpoints(currentHeight, currentShoulderAngle);
  }

  /* --------------------------------------------------------------------- */
  /* You can add and modify the code below.                                */
  /* --------------------------------------------------------------------- */

  /**
   * This is called whenever we have a new setPoint.
   */
  void recalculate() {

    setElevatorHeightSetpoint(ultimateSetpoint.getElevatorHeight());
    setShoulderAngleSetpoint(ultimateSetpoint.getShoulderAngle());
  }

  /**
   * look at current mechanism positions and change individual mechanism setpoints if necessary.
   */
  void tweakIntermediateSetpoints(Distance currentHeight, Angle currentShoulderAngle) {
    double elevatorMeters = currentHeight.in(Meters);
    double shoulderDegrees = currentShoulderAngle.in(Degrees);
    double targetShoulderDegrees = ultimateSetpoint.getShoulderAngle().in(Degrees);
    double targetElevatorMeters = ultimateSetpoint.getElevatorHeight().in(Meters);

    if (elevatorMeters < height_breakpoint_in_meters) {
        // If the elevator is below the limit, restrict the shoulder
        targetShoulderDegrees = MathUtil.clamp(targetShoulderDegrees, 80, 100);
    } 
    // If the elevator is above the breakpoint, let the shoulder move to the original target
    else {
        targetShoulderDegrees = ultimateSetpoint.getShoulderAngle().in(Degrees);
    }

    // If the shoulder is outside the limit, restrict the elevator
    if (shoulderDegrees > shoulder_breakpoint_in_degrees_high || shoulderDegrees < shoulder_breakpoint_in_degrees_low) {
      targetElevatorMeters = MathUtil.clamp(targetElevatorMeters, height_breakpoint_in_meters, ESEFElevatorMechanism.kElevatorMaxHeightMeters);
    }
    // If the shoulder is within the limits, let elevator loose
    else {
      targetElevatorMeters = ultimateSetpoint.getElevatorHeight().in(Meters);
    }

    // Actually set the intermediate shoulder setpoint
    setShoulderAngleSetpoint(Degrees.of(targetShoulderDegrees));
    setElevatorHeightSetpoint(Meters.of(targetElevatorMeters));
}



}
