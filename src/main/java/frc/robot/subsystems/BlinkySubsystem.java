package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import java.util.Map;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**/
public class BlinkySubsystem extends SubsystemBase {
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

  private final LEDPattern PATTERN_BREATHE_BLUE = LEDPattern.solid(Color.kBlue).breathe(Seconds.of(2)).atBrightness(BRIGHTNESS);
  private final LEDPattern PATTERN_BREATHE_MAIZE = LEDPattern.solid(Color.kYellow).breathe(Seconds.of(2)).atBrightness(BRIGHTNESS);

  private static final LEDPattern PATTERN_POLICE = PATTERN_BLUE.blink(Seconds.of(0.6)).overlayOn(PATTERN_RED);

  private static final LEDPattern PATTERN_STEP_DEMO = LEDPattern.steps(Map.of(0.00, Color.kGreen, 0.33, Color.kViolet, 0.67, Color.kBlueViolet)).scrollAtRelativeSpeed(Hertz.of(2)).atBrightness(BRIGHTNESS);

  public BlinkySubsystem() {
    addressableLED = new AddressableLED(0);
    addressableLEDBuffer = new AddressableLEDBuffer(18);

    addressableLED.setLength(addressableLEDBuffer.getLength());
    addressableLED.setData(addressableLEDBuffer);
    addressableLED.start();

    lowerLeft = addressableLEDBuffer.createView(0, 3);
    lowerRight = addressableLEDBuffer.createView(4, 7);
    topBar = addressableLEDBuffer.createView(8, 11);

    lowerLeftPattern = PATTERN_BREATHE_BLUE;
    lowerRightPattern = PATTERN_BREATHE_MAIZE;
    topBarPattern = PATTERN_STEP_DEMO;
  }

  @Override
  public void periodic() {
    lowerLeftPattern.applyTo(lowerLeft);
    lowerRightPattern.applyTo(lowerRight);
    topBarPattern.applyTo(topBar);

    // Set the LEDs
    addressableLED.setData(addressableLEDBuffer);
  }


}