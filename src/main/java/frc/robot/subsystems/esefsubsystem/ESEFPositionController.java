package frc.robot.subsystems.esefsubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.Utilities;
import org.usfirst.frc3620.logger.LoggingMaster;

import dev.doglog.DogLog;
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

  Distance height_breakpoint = Inches.of(8);
  Distance height_breakpoint_minimum = Inches.of(8);
  Angle shoulder_breakpoint_low = Degrees.of(52);
  Angle shoulder_breakpoint_high = Degrees.of(92.5);

  boolean wpilog_is_primed = false;

  public ESEFPositionController(ESEFElevatorMechanism elevatorMechanism, ESEFShoulderMechanism shoulderMechanism) {
    this.elevatorMechanism = elevatorMechanism;
    this.shoulderMechanism = shoulderMechanism;

    ultimateSetpoint = new ESEFPosition(Meters.of(0), Degrees.of(90));
    updateDashboardForUltimate();
    intermediateSetpoint = new ESEFPosition(Meters.of(0), Degrees.of(90));
    updateDashboardForIntermediate();
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
    if (wpilog_is_primed) {
      updateUltimateMech();
    }
  }

  void updateDashboardForIntermediate() {
    SmartDashboard.putNumber("frc3620/ESEF/intermediate.e", intermediateSetpoint.elevatorHeight.in(Inches));
    SmartDashboard.putNumber("frc3620/ESEF/intermediate.s", intermediateSetpoint.shoulderAngle.in(Degrees));
    if (wpilog_is_primed) {
      updateIntermediateMech();
    }
  }

  void updateUltimateMech() {
    ultimateSetpointMech.setShoulderAngle(ultimateSetpoint.shoulderAngle);
    ultimateSetpointMech.setElevatorHeight(ultimateSetpoint.elevatorHeight);
    SmartDashboard.putData("frc3620/ESEF/ultimate.mech", ultimateSetpointMech.getMech());
  }

  void updateIntermediateMech() {
    intermediateSetpointMech.setShoulderAngle(intermediateSetpoint.shoulderAngle);
    intermediateSetpointMech.setElevatorHeight(intermediateSetpoint.elevatorHeight);
    SmartDashboard.putData("frc3620/ESEF/intermediate.mech", intermediateSetpointMech.getMech());
  }

  public void setPosition (ESEFPosition position) {

    Distance currentHeight = elevatorMechanism.getCurrentHeight();
    Angle currentShoulderAngle = shoulderMechanism.getCurrentAngle();

    // Set dynamic height breakpoint based on the target height, ensuring it's never below the minimum
    SmartDashboard.putNumber("frc3620/ESEF/debug/targetHeight", position.elevatorHeight.in(Inches));
    SmartDashboard.putNumber("frc3620/ESEF/debug/dynamicBreakpoint", determineDynamicBreakpoint(position.elevatorHeight).in(Inches));
    SmartDashboard.putNumber("frc3620/ESEF/debug/previousBreakpoint", height_breakpoint.in(Inches));
    height_breakpoint = maxDistance(determineDynamicBreakpoint(position.elevatorHeight), height_breakpoint_minimum);
    SmartDashboard.putNumber("frc3620/ESEF/debug/updatedBreakpoint", height_breakpoint.in(Inches));    
    ultimateSetpoint = new ESEFPosition(position.elevatorHeight, position.shoulderAngle);
    
    updateDashboardForUltimate();
    recalculate();
  }

  public void bumpElevatorHeight(Distance delta) {
    ultimateSetpoint = limitedESEFPosition(ultimateSetpoint.elevatorHeight.plus(delta), ultimateSetpoint.shoulderAngle);
    updateDashboardForUltimate();
    recalculate();
  }

  public void bumpShoulderAngle(Angle delta) {
    ultimateSetpoint = limitedESEFPosition(ultimateSetpoint.elevatorHeight, ultimateSetpoint.shoulderAngle.plus(delta));
    updateDashboardForUltimate();
    recalculate();
  }

  void setElevatorHeightSetpoint(Distance d) {
    intermediateSetpoint = new ESEFPosition(d, intermediateSetpoint.shoulderAngle);
    elevatorMechanism.setSetpoint(d);
    updateDashboardForIntermediate();
  }

  void setShoulderAngleSetpoint(Angle a) {
    intermediateSetpoint = new ESEFPosition(intermediateSetpoint.elevatorHeight, a);
    shoulderMechanism.setSetpoint(a);
    updateDashboardForIntermediate();
  }

  public void periodic() {
    Distance currentHeight = elevatorMechanism.getCurrentHeight();
    Angle currentShoulderAngle = shoulderMechanism.getCurrentAngle();
    actualPositionMech.setElevatorHeight(currentHeight);
    actualPositionMech.setShoulderAngle(currentShoulderAngle);

    SmartDashboard.putNumber("frc3620/ESEF/actual.e", currentHeight.in(Inches));
    SmartDashboard.putNumber("frc3620/ESEF/actual.s", currentShoulderAngle.in(Degrees));
    SmartDashboard.putData("frc3620/ESEF/actual.mech", actualPositionMech.getMech());

    if (! wpilog_is_primed) {
      // need to do this here; if the mechanism2ds are logged in the constructor,
      // they do not seem to make it into the wpilog
      updateUltimateMech();
      updateIntermediateMech();
      wpilog_is_primed = true;
    }

    tweakIntermediateSetpoints(currentHeight, currentShoulderAngle);
  }

  /* --------------------------------------------------------------------- */
  /* You can add and modify the code below.                                */
  /* --------------------------------------------------------------------- */

  /**
   * This is called whenever we have a new setPoint.
   */
  void recalculate() {
    Distance currentHeight = elevatorMechanism.getCurrentHeight();
    Angle currentShoulderAngle = shoulderMechanism.getCurrentAngle();
    tweakIntermediateSetpoints(currentHeight, currentShoulderAngle);

    // setElevatorHeightSetpoint(ultimateSetpoint.getElevatorHeight());
    // setShoulderAngleSetpoint(ultimateSetpoint.getShoulderAngle());
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

    // Actually set the intermediate setpoints
    setShoulderAngleSetpoint(targetShoulderAngle);
    setElevatorHeightSetpoint(targetElevatorHeight);
  }

  private Distance determineDynamicBreakpoint(Distance targetHeight) {
    Distance targetBreakpt;
    if (targetHeight.lt(Inches.of(40))) {
        targetBreakpt = Inches.of(8); // Lower breakpoint for very low placements (will be overridden by min)
    } else if (targetHeight.lt(Inches.of(50))) {
        targetBreakpt = Inches.of(25); // Medium level breakpoint
    } else {
        targetBreakpt = Inches.of(43); // Higher breakpoint for upper placements
    }
    //SmartDashboard.putNumber("frc3620/ESEF/dynamicBreakpoint", targetBreakpt.in(Inches));
    return targetBreakpt;
  }

  private Distance maxDistance(Distance d1, Distance d2) {
    return d1.gt(d2) ? d1 : d2;
  }
}
