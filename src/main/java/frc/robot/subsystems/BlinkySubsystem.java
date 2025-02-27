package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

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

  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 32);
  // Our LED strip has a density of 120 LEDs per meter
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  // Create a new pattern that scrolls the rainbow pattern across the LED strip,
  // moving at a speed
  // of 1 meter per second.
  private final LEDPattern PATTERN_SCROLLING_RAINBOW = m_rainbow
      .scrollAtAbsoluteSpeed(MetersPerSecond.of(.01), kLedSpacing).atBrightness(BRIGHTNESS);

  private final LEDPattern PATTERN_RED = LEDPattern.solid(Color.kRed).atBrightness(BRIGHTNESS);
  private final LEDPattern PATTERN_BLUE = LEDPattern.solid(Color.kBlue).atBrightness(BRIGHTNESS);

  private final LEDPattern PATTERN_POLICE = PATTERN_BLUE.blink(Seconds.of(1)).overlayOn(PATTERN_RED);

  public BlinkySubsystem() {
    addressableLED = new AddressableLED(0);
    addressableLEDBuffer = new AddressableLEDBuffer(6);

    addressableLED.setLength(addressableLEDBuffer.getLength());
    addressableLED.setData(addressableLEDBuffer);
    addressableLED.start();

    left = addressableLEDBuffer.createView(0, 1);
    right = addressableLEDBuffer.createView(2, 3);
    top = addressableLEDBuffer.createView(4, 5);

    leftPattern = PATTERN_SCROLLING_RAINBOW;
    rightPattern = PATTERN_POLICE;
    topPattern = PATTERN_RED;
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