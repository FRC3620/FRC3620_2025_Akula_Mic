package frc.robot.commandfactories;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.SetPivotPositionCommand;
import frc.robot.commands.AFI.AFIRollerSetSpeedCommand;
import frc.robot.commands.AFI.AFIRollerSetSpeedContinuousCommand;
import frc.robot.commands.AFI.AFIRollerSetSpeedUntilInCommand;

import frc.robot.subsystems.AFISubsystem;

public class AFISubsystemCommandFactory {
  AFISubsystem afiSubsystem;

  public AFISubsystemCommandFactory(AFISubsystem afiSubsystem) {
    this.afiSubsystem = afiSubsystem;
  }

  public void setupSmartDashboardCommands() {
    SmartDashboard.putData("PivotPositionUp", new SetPivotPositionCommand(Degrees.of(70), afiSubsystem));
    SmartDashboard.putData("PivotPosition2", new SetPivotPositionCommand(Degrees.of(20), afiSubsystem));
    SmartDashboard.putData("PivotPositionGroundPickup", new SetPivotPositionCommand(Degrees.of(20), afiSubsystem));

    SmartDashboard.putNumber("AFIPivotSlider",0);

    SmartDashboard.putData("AFISetRollerSpeedContinuous", new AFIRollerSetSpeedContinuousCommand(() -> {
      return SmartDashboard.getNumber("AFIPivotSlider", -0.1);
    }, afiSubsystem).withName("ContinuousFromSlider"));

    SmartDashboard.putData("AFISetRollerSpeedContinuous0.1", new AFIRollerSetSpeedContinuousCommand(() -> {
      return 0.1;
    }, afiSubsystem).withName("Continuous0.1"));

    SmartDashboard.putData("AFISpit", new AFIRollerSetSpeedCommand(-0.05, afiSubsystem));

    SmartDashboard.putData("AFISetRollerSpeedUntilIn", new AFIRollerSetSpeedUntilInCommand(0.5, afiSubsystem));
    SmartDashboard.putData("Trying Again", new AFIRollerSetSpeedUntilInCommand(0.5, afiSubsystem));
    SmartDashboard.putData("AFIStopRoller", new AFIRollerSetSpeedCommand(0.0, afiSubsystem));

    SmartDashboard.putData("AFISetRollerSpeed1", new AFIRollerSetSpeedCommand(0.1, afiSubsystem));


  }
}
