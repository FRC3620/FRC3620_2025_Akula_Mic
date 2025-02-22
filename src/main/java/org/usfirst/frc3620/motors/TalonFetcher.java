package org.usfirst.frc3620.motors;

import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFetcher extends MotorWatcherFetcher {
  TalonFX talonFX;

  public TalonFetcher(TalonFX _talonFX) {
    super();
    talonFX = _talonFX;
  }

  @Override
  public void setPower(double power) {
    talonFX.set(power);
  }

  @Override
  Double measureTemperature() {
    return temperature = talonFX.getDeviceTemp().getValueAsDouble();
  }

  @Override
  Double measurePosition() {
    return position = talonFX.getPosition().getValueAsDouble();
  }

  @Override
  Double measureOutputCurrent() {
    return outputCurrent = talonFX.getStatorCurrent().getValueAsDouble();
  }

  @Override
  Double measureVelocity() {
    return velocity = talonFX.getVelocity().getValueAsDouble();
  }
}
