package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import java.util.Map;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.RobotMode;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

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

  private static final LEDPattern PATTERN_RED_BLINK = LEDPattern.solid(Color.kRed).atBrightness(BRIGHTNESS).blink(Seconds.of(0.1));
  private static final LEDPattern PATTERN_RED = LEDPattern.solid(Color.kRed).atBrightness(BRIGHTNESS);
  private static final LEDPattern PATTERN_BLUE = LEDPattern.solid(Color.kBlue).atBrightness(BRIGHTNESS);
  private static final LEDPattern PATTERN_MAIZE = LEDPattern.solid(Color.kYellow).atBrightness(BRIGHTNESS);
  private static final LEDPattern OFF = LEDPattern.solid(Color.kBlack).atBrightness(BRIGHTNESS);
  private static final LEDPattern PATTERN_GREEN = LEDPattern.solid(Color.kGreen).atBrightness(BRIGHTNESS);
  private static final LEDPattern PATTERN_ORANGE = LEDPattern.solid(Color.kOrangeRed).atBrightness(BRIGHTNESS.div(2));

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

  boolean isAutoAllignFinished = false;

  public BlinkySubsystem() {
    addressableLED = new AddressableLED(0);
    addressableLEDBuffer = new AddressableLEDBuffer(76);

    addressableLED.setLength(addressableLEDBuffer.getLength());
    addressableLED.setData(addressableLEDBuffer);
    addressableLED.start();

    if (RobotContainer.robotParameters.isShortLEDStrip()) {
      lowerLeft = addressableLEDBuffer.createView(0, 3);
      upperLeft = addressableLEDBuffer.createView(4, 7);
      topBar = addressableLEDBuffer.createView(8, 11);
      lowerRight = addressableLEDBuffer.createView(12, 15);
      upperRight = addressableLEDBuffer.createView(16, 19);
    } else {
      lowerLeft = addressableLEDBuffer.createView(1, 15);
      upperLeft = addressableLEDBuffer.createView(16, 34);
      topBar = addressableLEDBuffer.createView(35, 37);
      lowerRight = addressableLEDBuffer.createView(39, 53);
      upperRight = addressableLEDBuffer.createView(54, 72);
    }

    lowerLeftPattern = PATTERN_BREATHE_BLUE;
    lowerRightPattern = PATTERN_BREATHE_MAIZE;
    topBarPattern = OFF;
    upperLeftPattern = OFF;
    upperRightPattern = OFF;
  }

  @Override
  public void periodic() {
    double matchTime = Timer.getMatchTime();

    if (matchTime <= 30 && matchTime > 0 && DriverStation.isTeleop()) {
      upperLeftPattern = upperRightPattern = PATTERN_RED_BLINK;
    }

    lowerLeftPattern.applyTo(lowerLeft);
    lowerRightPattern.applyTo(lowerRight);
    topBarPattern.applyTo(topBar);
    upperLeftPattern.applyTo(upperLeft);
    upperRightPattern.applyTo(upperRight);

    // Set the LEDs
    addressableLED.setData(addressableLEDBuffer);
  }

  // this should probably be in command factory, but I'm lazy
  public Command setModeCommand(ModeState m) {
    return Commands.runOnce(() -> setMode(m)).withName("blinky.setModeCommand[" + m + "]");
  }

  private void setMode(ModeState m) {
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

  public Command setESEFCommand(BlinkyStickHeight l) {
    return Commands.runOnce(() -> setESEF(l)).withName("blinky.setESEFCommand[" + l + "]");
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
    setLowerBarPattern();
  }

  public void setAutoAllignFinished(boolean b){
    isAutoAllignFinished = b;
    setLowerBarPattern();
  }

  void setLowerBarPattern(){
    if(isAutoAllignFinished){
      lowerLeftPattern = PATTERN_GREEN;
      lowerRightPattern = PATTERN_GREEN;
    }
    else{
      if(Robot.getCurrentRobotMode() == RobotMode.DISABLED){
        lowerLeftPattern = PATTERN_BREATHE_BLUE;
        lowerRightPattern = PATTERN_BREATHE_MAIZE;
      }
      else{
        lowerLeftPattern = PATTERN_BLUE;
        lowerRightPattern = PATTERN_MAIZE;
      }
    }
  }
}