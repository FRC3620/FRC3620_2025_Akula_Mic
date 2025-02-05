// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.blinky;

import org.usfirst.frc3620.RobotMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

import frc.robot.subsystems.BlinkySubsystem.LightSegment;

public class DefaultBlinkyCommand extends Command {

  final Pattern patternReadyToShoot = new BlinkPattern().setColor(Color.kGreen);
  final Pattern patternGotPiece = new SolidPattern().setColor(Color.kGreen);
  final Pattern patternNoPiece = new SolidPattern().setColor(Color.kGray);
  final Pattern patternIdle = new BlinkPattern().setColor(Color.kGreen).setOnSeconds(0.1).setOffSeconds(1.0);
  // final Pattern patternSick = new
  // ChasePattern().setColor1(Color.kRed).setColor2(Color.kBlue).setBlinkTime(0.25);
  final Pattern patternSick = new BlinkPattern().setColor1(Color.kRed).setColor2(Color.kBlue).setBlinkTime(0.50);
  final Pattern patternReallySick = new BlinkPattern().setColor1(Color.kRed).setColor2(Color.kBlue).setBlinkTime(0.10);

  LightSegment lightSegment;

  public DefaultBlinkyCommand(LightSegment lightSegment) {
    this.lightSegment = lightSegment;
    addRequirements(lightSegment);
  }

  @Override
  public void execute() {
    Pattern p=new SolidPattern();
    if (Robot.getCurrentRobotMode() == RobotMode.DISABLED) {
     p=patternSick;
    } else {

     p=patternIdle;

    }
    lightSegment.setPattern(p);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  
  }

}