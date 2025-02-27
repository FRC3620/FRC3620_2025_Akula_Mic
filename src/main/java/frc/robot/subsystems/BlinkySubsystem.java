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

  static final Dimensionless BRIGHTNESS = Value.of(0.1);

  AddressableLED addressableLED;
  AddressableLEDBuffer addressableLEDBuffer;

  AddressableLEDBufferView left, right, top;
  LEDPattern leftPattern, rightPattern, topPattern;

  private static final LEDPattern PATTERN_SCROLLING_RAINBOW = LEDPattern.rainbow(255, 32)
      .scrollAtRelativeSpeed(Hertz.of(2)).atBrightness(BRIGHTNESS);

  private static final LEDPattern PATTERN_RED = LEDPattern.solid(Color.kRed).atBrightness(BRIGHTNESS);
  private static final LEDPattern PATTERN_BLUE = LEDPattern.solid(Color.kBlue).atBrightness(BRIGHTNESS);

  private static final LEDPattern PATTERN_POLICE = PATTERN_BLUE.blink(Seconds.of(0.6)).overlayOn(PATTERN_RED);

  private static final LEDPattern PATTERN_STEP_DEMO = LEDPattern.steps(Map.of(0.00, Color.kGreen, 0.33, Color.kViolet, 0.67, Color.kBlueViolet)).scrollAtRelativeSpeed(Hertz.of(2)).atBrightness(BRIGHTNESS);

  public BlinkySubsystem() {
    addressableLED = new AddressableLED(0);
    addressableLEDBuffer = new AddressableLEDBuffer(12);

    addressableLED.setLength(addressableLEDBuffer.getLength());
    addressableLED.setData(addressableLEDBuffer);
    addressableLED.start();

    left = addressableLEDBuffer.createView(0, 3);
    right = addressableLEDBuffer.createView(4, 7);
    top = addressableLEDBuffer.createView(8, 11);

    leftPattern = PATTERN_SCROLLING_RAINBOW;
    rightPattern = PATTERN_POLICE;
    topPattern = PATTERN_STEP_DEMO;
  }

  @Override
  public void periodic() {
    leftPattern.applyTo(left);
    rightPattern.applyTo(right);
    topPattern.applyTo(top);

    // Set the LEDs
    addressableLED.setData(addressableLEDBuffer);
  }

}