package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import java.util.Map;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.RobotMode;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**/
public class BlinkySubsystem extends SubsystemBase {

  public enum BlinkyStickHeight {
    L1, L2, L3, L4, L2ALGAE, L3ALGAE, BARGEALGAE;
  }

  public enum ModeState {
    CLIMB, FLOOR_PICKUP, CORAL_PICKUP;
  }

  public enum HealthStatus {
    OKAY, WARNING, ERROR, HAIRONFIRE;
  }

  static Color heightColor = Color.kPurple;

  TaggedLogger logger = LoggingMaster.getLogger(getClass());

  static final Dimensionless BRIGHTNESS = Value.of(0.25);

  AddressableLED addressableLED;
  AddressableLEDBuffer addressableLEDBuffer;

  AddressableLEDBufferView lowerLeft, lowerRight, topBar, upperLeft, upperRight;
  LEDPattern lowerLeftPattern, upperLeftPattern, lowerRightPattern, upperRightPattern, topBarPattern;

  private static final LEDPattern PATTERN_SCROLLING_RAINBOW = LEDPattern.rainbow(255, 32)
      .scrollAtRelativeSpeed(Hertz.of(2)).atBrightness(BRIGHTNESS);

  private static final LEDPattern PATTERN_RED = LEDPattern.solid(Color.kRed).atBrightness(BRIGHTNESS);
  private static final LEDPattern PATTERN_BLUE = LEDPattern.solid(Color.kBlue).atBrightness(BRIGHTNESS);
  private static final LEDPattern PATTERN_MAIZE = LEDPattern.solid(Color.kYellow).atBrightness(BRIGHTNESS);
  private static final LEDPattern OFF = LEDPattern.solid(Color.kBlack).atBrightness(BRIGHTNESS);
  private static final LEDPattern PATTERN_GREEN = LEDPattern.solid(Color.kGreen).atBrightness(BRIGHTNESS);
  private static final LEDPattern PATTERN_ORANGE = LEDPattern.solid(Color.kOrangeRed).atBrightness(BRIGHTNESS);

  private static final LEDPattern PATTERN_L1 = LEDPattern.steps(Map.of(0.75, heightColor)).reversed();
  private static final LEDPattern PATTERN_L2 = LEDPattern.steps(Map.of(0.5, heightColor)).reversed();
  private static final LEDPattern PATTERN_L3 = LEDPattern.steps(Map.of(0.25, heightColor)).reversed();
  private static final LEDPattern PATTERN_L4 = LEDPattern.solid(heightColor);
  private static final LEDPattern PATTERN_L2_ALGAE = LEDPattern.steps(Map.of(0.66, Color.kCyan)).reversed();
  private static final LEDPattern PATTERN_L3_ALGAE = LEDPattern.steps(Map.of(0.33, Color.kCyan)).reversed();
  private static final LEDPattern PATTERN_CLIMB = LEDPattern.solid(Color.kOrange).blink(Time.ofBaseUnits(0.2, Seconds));
  private static final LEDPattern PATTERN_FLOOR_PICKUP = LEDPattern.solid(Color.kCyan).blink(Time.ofBaseUnits(0.2, Seconds));
  private static final LEDPattern PATTERN_CORAL_PICKUP = LEDPattern.rainbow(255, 32).scrollAtRelativeSpeed(Hertz.of(5)).atBrightness(BRIGHTNESS);

  private final LEDPattern PATTERN_BREATHE_BLUE = LEDPattern.solid(Color.kBlue).breathe(Seconds.of(2))
      .atBrightness(BRIGHTNESS);
  private final LEDPattern PATTERN_BREATHE_MAIZE = LEDPattern.solid(Color.kYellow).breathe(Seconds.of(2))
      .atBrightness(BRIGHTNESS);

  private static final LEDPattern PATTERN_POLICE = PATTERN_BLUE.blink(Seconds.of(0.6)).overlayOn(PATTERN_RED);

  private static final LEDPattern PATTERN_STEP_DEMO = LEDPattern
      .steps(Map.of(0.00, Color.kGreen, 0.33, Color.kViolet, 0.67, Color.kBlueViolet))
      .scrollAtRelativeSpeed(Hertz.of(2)).atBrightness(BRIGHTNESS);

  public BlinkySubsystem() {
    addressableLED = new AddressableLED(0);
    addressableLEDBuffer = new AddressableLEDBuffer(76);

    addressableLED.setLength(addressableLEDBuffer.getLength());
    addressableLED.setData(addressableLEDBuffer);
    addressableLED.start();

    lowerLeft = addressableLEDBuffer.createView(1, 15);
    upperLeft = addressableLEDBuffer.createView(16, 34);
    topBar = addressableLEDBuffer.createView(35, 37);
    lowerRight = addressableLEDBuffer.createView(39, 53);
    upperRight = addressableLEDBuffer.createView(54, 72);

    lowerLeftPattern = PATTERN_BREATHE_BLUE;
    lowerRightPattern = PATTERN_BREATHE_MAIZE;
    topBarPattern = PATTERN_STEP_DEMO;
    upperLeftPattern = OFF;
    upperRightPattern = OFF;

  }

  @Override
  public void periodic() {
    lowerLeftPattern.applyTo(lowerLeft);
    lowerRightPattern.applyTo(lowerRight);
    topBarPattern.applyTo(topBar);
    upperLeftPattern.applyTo(upperLeft);
    upperRightPattern.applyTo(upperRight);

    // Set the LEDs
    addressableLED.setData(addressableLEDBuffer);
  }

  public void setMode(ModeState m) {
    if (m == ModeState.CLIMB) {
      upperLeftPattern = upperRightPattern = PATTERN_CLIMB;
    }
    if (m == ModeState.FLOOR_PICKUP) {
      upperLeftPattern = upperRightPattern = PATTERN_FLOOR_PICKUP;
    }
    if (m == ModeState.CORAL_PICKUP) {
      upperLeftPattern = upperRightPattern = PATTERN_CORAL_PICKUP;
    }
  }

  public void setESEF(BlinkyStickHeight l) {
    if (l == BlinkyStickHeight.L1) {
      upperLeftPattern = upperRightPattern = PATTERN_L1;
    }
    if (l == BlinkyStickHeight.L2) {
      upperLeftPattern = upperRightPattern = PATTERN_L2;
    }
    if (l == BlinkyStickHeight.L3) {
      upperLeftPattern = upperRightPattern = PATTERN_L3;
    }
    if (l == BlinkyStickHeight.L4) {
      upperLeftPattern = upperRightPattern = PATTERN_L4;
    }
    if (l == BlinkyStickHeight.L2ALGAE) {
      upperLeftPattern = upperRightPattern = PATTERN_L2_ALGAE;
    }
    if (l == BlinkyStickHeight.L3ALGAE) {
      upperLeftPattern = upperRightPattern = PATTERN_L3_ALGAE;
    }
    if(l == BlinkyStickHeight.BARGEALGAE){
      upperLeftPattern = upperRightPattern = PATTERN_GREEN;
    }

  }

  public void setHealthStatus(HealthStatus h) {
    switch (h) {
      case OKAY:
        topBarPattern = PATTERN_GREEN;
        break;
      case WARNING:
        topBarPattern = PATTERN_ORANGE;
        break;
      case ERROR:
        topBarPattern = PATTERN_RED;
        break;
      case HAIRONFIRE:
        topBarPattern = PATTERN_POLICE;
        break;
      default:
        topBarPattern = OFF;
        break;
    }
  }

  public void setRobotMode(RobotMode mode) {
    switch (mode) {
      case DISABLED:
        lowerLeftPattern = PATTERN_BREATHE_BLUE;
        lowerRightPattern = PATTERN_BREATHE_MAIZE;
        break;
      default:
        lowerLeftPattern = PATTERN_BLUE;
        lowerRightPattern = PATTERN_MAIZE;
        break;
    }
  }
}