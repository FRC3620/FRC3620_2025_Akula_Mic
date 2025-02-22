package frc.robot.subsystems.esefsubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.Utilities;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ESEFPositionController {
  /* --------------------------------------------------------------------- */
  /* Don't touch anything from here to where it says                       */
  /* "You can add and modify the code below."                              */
  /* --------------------------------------------------------------------- */
  TaggedLogger logger = LoggingMaster.getLogger(getClass());
  
  final ESEFElevatorMechanism elevatorMechanism;
  final ESEFShoulderMechanism shoulderMechanism;
  
  ESEFPosition ultimateSetpoint;
  ESEFPosition intermediateSetpoint;

  ESEFMech ultimateSetpointMech = new ESEFMech();
  ESEFMech intermediateSetpointMech = new ESEFMech();
  ESEFMech actualPositionMech = new ESEFMech();

  Distance height_breakpoint = Inches.of(15);
  Angle shoulder_breakpoint_low = Degrees.of(75);
  Angle shoulder_breakpoint_high = Degrees.of(95);

  public ESEFPositionController(ESEFElevatorMechanism elevatorMechanism, ESEFShoulderMechanism shoulderMechanism) {
    this.elevatorMechanism = elevatorMechanism;
    this.shoulderMechanism = shoulderMechanism;

    ultimateSetpoint = new ESEFPosition(Meters.of(0), Degrees.of(90));
    updateDashboardForUltimate();
    intermediateSetpoint = new ESEFPosition(Meters.of(0), Degrees.of(90));
    updateDashboardForIntermediate();

    SmartDashboard.putData("frc3620/ESEF/setpointMech", ultimateSetpointMech.getMech());
    SmartDashboard.putData("frc3620/ESEF/intermediateSetpointMech", intermediateSetpointMech.getMech());
    SmartDashboard.putData("frc3620/ESEF/actualPositionMech", actualPositionMech.getMech());
  }

  ESEFPosition limitedESEFPosition(Distance d, Angle a) {
    if (d.lt(height_breakpoint)) {
      a = Utilities.clamp(a, shoulder_breakpoint_low, shoulder_breakpoint_high);
    }
    d = Utilities.clamp (d, ESEFElevatorMechanism.kElevatorMinHeight, ESEFElevatorMechanism.kElevatorMaxHeight);
    return new ESEFPosition(d, a);
  }

  void updateDashboardForUltimate() {
    SmartDashboard.putNumber("frc3620/ESEF/ultimate.e", ultimateSetpoint.elevatorHeight.in(Inches));
    SmartDashboard.putNumber("frc3620/ESEF/ultimate.s", ultimateSetpoint.shoulderAngle.in(Degrees));
  }

  void updateDashboardForIntermediate() {
    SmartDashboard.putNumber("frc3620/ESEF/intermediate.e", intermediateSetpoint.elevatorHeight.in(Inches));
    SmartDashboard.putNumber("frc3620/ESEF/intermediate.s", intermediateSetpoint.shoulderAngle.in(Degrees));
  }

  public void setPosition (ESEFPosition position) {
    // Set dynamic height breakpoint based on the target height, ensuring it's never below the minimum
    height_breakpoint = maxDistance(determineDynamicBreakpoint(position.elevatorHeight), height_breakpoint);
    ultimateSetpoint = new ESEFPosition(position.elevatorHeight, position.shoulderAngle);
    
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
    SmartDashboard.putNumber("frc3620/ESEF/actual.e", currentHeight.in(Inches));
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
    // TODO: are we ok with doing this?
    setElevatorHeightSetpoint(ultimateSetpoint.getElevatorHeight());
    setShoulderAngleSetpoint(ultimateSetpoint.getShoulderAngle());
  }

  /**
   * look at current mechanism positions and change individual mechanism setpoints if necessary.
   */
  void tweakIntermediateSetpoints(Distance currentHeight, Angle currentShoulderAngle) {
    Angle targetShoulderAngle = ultimateSetpoint.shoulderAngle;
    if (currentHeight.lt(height_breakpoint)) {
      // If the elevator is below the limit, restrict the shoulder
      targetShoulderAngle = Utilities.clamp(ultimateSetpoint.shoulderAngle, shoulder_breakpoint_low, shoulder_breakpoint_high);
    }

    Distance targetElevatorHeight = ultimateSetpoint.elevatorHeight;
    if (currentShoulderAngle.gt(shoulder_breakpoint_high) || currentShoulderAngle.lt(shoulder_breakpoint_low)) {
      // If the shoulder is outside the limit, restrict the elevator
      targetElevatorHeight = Utilities.clamp(targetElevatorHeight, height_breakpoint, ESEFElevatorMechanism.kElevatorMaxHeight);
    }

    // Actually set the intermediate shoulder setpoint
    setShoulderAngleSetpoint(targetShoulderAngle);
    setElevatorHeightSetpoint(targetElevatorHeight);
  }

  private Distance determineDynamicBreakpoint(Distance targetHeight) {
    Distance targetBreakpt;
    if (targetHeight.lt(Inches.of(25))) {
        targetBreakpt = Inches.of(15); // Lower breakpoint for very low placements (will be overridden by min)
    } else if (targetHeight.lt(Inches.of(50))) {
        targetBreakpt = Inches.of(25); // Medium level breakpoint
    } else {
        targetBreakpt = Inches.of(40); // Higher breakpoint for upper placements
    }
    SmartDashboard.putNumber("frc3620/ESEF/dynamicBreakpoint", targetBreakpt.in(Inches));
    return targetBreakpt;
}

private Distance maxDistance(Distance d1, Distance d2) {
  return d1.gt(d2) ? d1 : d2;
}
}
