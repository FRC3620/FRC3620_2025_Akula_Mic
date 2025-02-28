package org.usfirst.frc3620.motors;

import com.revrobotics.spark.SparkMax;

public class SparkMAXFetcher extends MotorWatcherFetcher {
  SparkMax sparkMax;

  SparkMAXFetcher(SparkMax _sparkMAX) {
    super();
    sparkMax = _sparkMAX;
  }

  @Override
  public void setPower(double power) {
    sparkMax.set(power);
  }

  @Override
  Double measureTemperature() {
    return temperature = sparkMax.getMotorTemperature();
  }

  @Override
  Double measurePosition() {
    return position = sparkMax.getEncoder().getPosition();
  }

  @Override
  Double measureOutputCurrent() {
    return outputCurrent = sparkMax.getOutputCurrent();
  }

  @Override
  Double measureVelocity() {
    return velocity = sparkMax.getEncoder().getVelocity();
  }
}
