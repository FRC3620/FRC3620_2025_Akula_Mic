package frc.robot.commandfactories;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.fasterxml.jackson.annotation.JsonAutoDetect.Visibility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.swervedrive.DriveToAlgaeCommand;
import frc.robot.commands.swervedrive.SwerveDriveDiagnosticCommand;
import frc.robot.commands.swervedrive.DriveToAlgaeCommand.WhichStick;
import frc.robot.commands.swervedrive.DriveToPoseCommand;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class SwerveSubsystemCommandFactory {
  SwerveSubsystem swerveSubsystem;

  public SwerveSubsystemCommandFactory(SwerveSubsystem afiSubsystem) {
    this.swerveSubsystem = afiSubsystem;
  }

  public void setupSmartDashboardCommands() {
    // SmartDashboard.putData("Drive 10 feet",
    // swerveSubsystem.driveToDistanceCommand(Units.feetToMeters(10), 0.5));
    SmartDashboard.putData("Test Drive To Pose", swerveSubsystem.pathFinderCommand());

    SmartDashboard.putData("Drive 10 feet", swerveSubsystem.driveToDistanceCommand(Units.feetToMeters(10), 0.5));
    SmartDashboard.putData("TurnToAngle", new TurnToAngleCommand(Degrees.of(90), Meters.of(2), swerveSubsystem));
    SmartDashboard.putData("TurnToAngle-135", new TurnToAngleCommand(Degrees.of(-135), Meters.of(2), swerveSubsystem));

    SmartDashboard.putData("Drive To Nearest Tag LEFT", new DriveToAlgaeCommand(WhichStick.LEFT));
    SmartDashboard.putData("Drive To Nearest Tag RIGHT", new DriveToAlgaeCommand(WhichStick.RIGHT));
   

    Pose2d testPose = new Pose2d(5.1,3.09, Rotation2d.fromDegrees(120));
    SmartDashboard.putData("Drive To TEST", swerveSubsystem.driveToPoseSlow(testPose));
  
    SmartDashboard.putData("Drive to Pose", new DriveToPoseCommand(swerveSubsystem, new Pose2d(5.19, 3.04, Rotation2d.fromDegrees(120))));

    SmartDashboard.putData(new SwerveDriveDiagnosticCommand(swerveSubsystem));

  }

}
