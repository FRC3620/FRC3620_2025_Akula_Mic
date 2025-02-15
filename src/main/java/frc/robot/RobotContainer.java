package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.io.File;

import org.tinylog.TaggedLogger;

import org.usfirst.frc3620.logger.LogCommand;
import org.usfirst.frc3620.logger.LoggingMaster;

import com.pathplanner.lib.auto.NamedCommands;

import org.usfirst.frc3620.CANDeviceFinder;
import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.ChameleonController;
import org.usfirst.frc3620.ChameleonController.ControllerType;
import org.usfirst.frc3620.FlySkyConstants;
import org.usfirst.frc3620.RobotParametersContainer;
import org.usfirst.frc3620.Utilities;
import org.usfirst.frc3620.XBoxConstants;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.esefcommands.SetElevatorPositionCommand;
import frc.robot.commands.esefcommands.SetEndEffectorSpeedCommand;
import frc.robot.commands.esefcommands.SetManualElevatorCommand;
import frc.robot.commands.esefcommands.SetShoulderPositionCommand;
import frc.robot.subsystems.AFISubsystem;
import frc.robot.subsystems.BlinkySubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HealthSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.esefsubsystem.ESEFSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import swervelib.imu.SwerveIMU;
import frc.robot.commands.ContinuousSetIMUFromMegaTag1Command;
import frc.robot.commands.SetClimberPostionCommand;
import frc.robot.commands.SetIMUFromMegaTag1Command;
import frc.robot.commands.SetPivotPositionCommand;
import frc.robot.commands.AFI.AFIRollerSetSpeedCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final static TaggedLogger logger = LoggingMaster.getLogger(RobotContainer.class);

  // need this
  public static CANDeviceFinder canDeviceFinder;
  public static RobotParameters robotParameters;

  Alert missingDevicesAlert = new Alert(HealthSubsystem.HARDWARE_ALERT_GROUP_NAME, "", Alert.AlertType.kError);

  // hardware here...
  private static DigitalInput practiceBotJumper;

  public static PowerDistribution powerDistribution = null;
  public static PneumaticsModuleType pneumaticModuleType = null;

  // subsystems here
  public static ESEFSubsystem esefSubsystem;
  public static AFISubsystem afiSubsystem;
  public static SwerveSubsystem swerveSubsystem;
  public static HealthSubsystem healthSubsystem;
  ClimberSubsystem climberSubsystem;
  public static BlinkySubsystem blinkySubsystem;
  public static VisionSubsystem visionSubsystem;


  // joysticks here....
  public static ChameleonController driverJoystick;
  public static Joystick operatorJoystick;

  // We'll be using this
  public static AprilTagFieldLayout aprilTagFieldLayout;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    canDeviceFinder = new CANDeviceFinder();

    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    robotParameters = RobotParametersContainer.getRobotParameters(RobotParameters.class);
    logger.info("got parameters for chassis '{}'", robotParameters.getName());
    Utilities.logMetadataToDataLog("Robot", robotParameters.getName());

    practiceBotJumper = new DigitalInput(0);
    boolean iAmACompetitionRobot = amIACompBot();
    if (!iAmACompetitionRobot) {
      logger.warn("this is a test chassis, will try to deal with missing hardware!");
    }

    /*
    if (canDeviceFinder.isDevicePresent(CANDeviceType.REV_PH, 1, "REV PH") || iAmACompetitionRobot) {
      pneumaticModuleType = PneumaticsModuleType.REVPH;
    } else if (canDeviceFinder.isDevicePresent(CANDeviceType.CTRE_PCM, 0, "CTRE PCM")) {
      pneumaticModuleType = PneumaticsModuleType.CTREPCM;
    }
    */

    if (canDeviceFinder.isDevicePresent(CANDeviceType.REV_PDH, 1) || iAmACompetitionRobot) {
      powerDistribution = new PowerDistribution();
    } else if (canDeviceFinder.isDevicePresent(CANDeviceType.CTRE_PDP, 0)) {
      powerDistribution = new PowerDistribution();
    }

    makeSubsystems();

    if (!canDeviceFinder.getMissingDeviceSet().isEmpty()) {
      missingDevicesAlert.set(true);
      missingDevicesAlert.setText("Missing from CAN bus: " + canDeviceFinder.getMissingDeviceSet());
    }

    // Configure the button bindings
    configureButtonBindings();

    setupSmartDashboardCommands();

    setupAutonomousCommands();

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    Utilities.addDataLogForNT("SmartDashboard/swerve");
  }

  private void makeSubsystems() {
    if (canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, 1, "Swerve Drive 1") || shouldMakeAllCANDevices()) {
      canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, 3, "Swerve Drive 3");
      canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, 5, "Swerve Drive 5");
      canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, 7, "Swerve Drive 7");
      canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 2, "Swerve Drive 2");
      canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 4, "Swerve Drive 4");
      canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 6, "Swerve Drive 6");
      canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 8, "Swerve Drive 8");

      String swerveFolder = robotParameters.getSwerveDirectoryName();

      SmartDashboard.putString("swerveFolder", swerveFolder);
      logger.info("using swerveFolder '{}'", swerveFolder);
      swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), swerveFolder));
    }

    esefSubsystem = new ESEFSubsystem();
    afiSubsystem = new AFISubsystem();
    climberSubsystem = new ClimberSubsystem();
    blinkySubsystem = new BlinkySubsystem();
    visionSubsystem = new VisionSubsystem();
    
    // need to create healthSubsystem LAST!!!!!!!
    healthSubsystem = new HealthSubsystem();

    CommandScheduler.getInstance().schedule(new ContinuousSetIMUFromMegaTag1Command());
  }


  public String getDriverControllerName() {
    return driverJoystick.getName();
  }

  public void setDriverControllerName(ControllerType driveControllerType) {
    driverJoystick.setCurrentControllerType(driveControllerType);
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureButtonBindings() {

    driverJoystick = new ChameleonController(new Joystick(0));


    if (swerveSubsystem != null) {
      /*
        Converts driver input into a field-relative ChassisSpeeds that is controlled
        by angular velocity.
       */
      SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
          () -> getDriveVerticalJoystick() * -1,
          () -> getDriveHorizontalJoystick() * -1)
          .withControllerRotationAxis(() -> getDriveSpinJoystick() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

      /*
        Clone's the angular velocity input stream and converts it to a fieldRelative
        input stream.()
       */
      SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> getDriveHorizontalJoystick(),
          () -> getDriveVerticalJoystick())
          .headingWhile(true);

      /*
        Clone's the angular velocity input stream and converts it to a robotRelative
        input stream.
       */
      SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
          .allianceRelativeControl(false);

      SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
          () -> -getDriveVerticalJoystick(),
          () -> -getDriveHorizontalJoystick())
          .withControllerRotationAxis(() -> getDriveSpinJoystick() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

      /*     
      // Derive the heading axis with math!
      SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
          .withControllerHeadingAxis(() -> Math.sin(
              driverXbox.getRawAxis(
                  2) *
                  Math.PI)
              *
              (Math.PI *
                  2),
              () -> Math.cos(
                  driverXbox.getRawAxis(
                      2) *
                      Math.PI)
                  *
                  (Math.PI *
                      2))
          .headingWhile(true);
      */
      Command driveFieldOrientedDirectAngle = swerveSubsystem.driveFieldOriented(driveDirectAngle);
      Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
      Command driveRobotOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveRobotOriented);
      Command driveSetpointGen = swerveSubsystem.driveWithSetpointGeneratorFieldRelative(
          driveDirectAngle);
      //Command driveFieldOrientedDirectAngleKeyboard = swerveSubsystem.driveFieldOriented(driveDirectAngleKeyboard);
      Command driveFieldOrientedAnglularVelocityKeyboard = swerveSubsystem
          .driveFieldOriented(driveAngularVelocityKeyboard);
     // Command driveSetpointGenKeyboard = swerveSubsystem.driveWithSetpointGeneratorFieldRelative(
     //     driveDirectAngleKeyboard);

      if (RobotBase.isSimulation()) {
        //swerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
      } else {
        swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      }

      if (Robot.isSimulation()) {
        //driverXbox.start()
        //    .onTrue(Commands.runOnce(() -> swerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
        //driverXbox.button(1).whileTrue(swerveSubsystem.sysIdDriveMotorCommand());
      }

      /*
        note from Doug:
        this looks kind of incorrect; we will NEVER be in test mode when the robot is
        coming up
       */
      /*
      if (DriverStation.isTest()) {
        swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

        driverXbox.x().whileTrue(Commands.runOnce(swerveSubsystem::lock, swerveSubsystem).repeatedly());
        driverXbox.y().whileTrue(swerveSubsystem.driveToDistanceCommand(1.0, 0.2));
        driverXbox.start().onTrue((Commands.runOnce(swerveSubsystem::zeroGyro)));
        driverXbox.back().whileTrue(swerveSubsystem.centerModulesCommand());
        driverXbox.leftBumper().onTrue(Commands.none());
        driverXbox.rightBumper().onTrue(Commands.none());
      } else {
        driverXbox.a().onTrue((Commands.runOnce(swerveSubsystem::zeroGyro)));
        driverXbox.x().onTrue(Commands.runOnce(swerveSubsystem::addFakeVisionReading));
        driverXbox.b().whileTrue(
            swerveSubsystem.driveToPose(
                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
        driverXbox.start().whileTrue(Commands.none());
        driverXbox.back().whileTrue(Commands.none());
        driverXbox.leftBumper().whileTrue(Commands.runOnce(swerveSubsystem::lock, swerveSubsystem).repeatedly());
        driverXbox.rightBumper().onTrue(Commands.none());
      }*/
    }

    //driverJoystick = new Joystick(0);
    operatorJoystick = new Joystick(1);

    //new JoystickButton(driverJoystick, XBoxConstants.BUTTON_A)
    //    .onTrue(new LogCommand("'A' button hit"));

  }

  private void setupSmartDashboardCommands() {
    // SmartDashboard.putData("Shoulder.P1", new SetShoulderPositionCommand(null,
    // null));
    SmartDashboard.putData("ShoulderSetPosition1", new SetShoulderPositionCommand(Degrees.of(0), esefSubsystem));
    SmartDashboard.putData("ShoulderSetPosition2", new SetShoulderPositionCommand(Degrees.of(30), esefSubsystem));
    SmartDashboard.putData("ShoulderSetPosition3", new SetShoulderPositionCommand(Degrees.of(60), esefSubsystem));
    SmartDashboard.putData("ShoulderSetPosition4", new SetShoulderPositionCommand(Degrees.of(90), esefSubsystem));
    
    SmartDashboard.putData("ElevatorSetPosition1", new SetElevatorPositionCommand(Inches.of(8.0), esefSubsystem));
    SmartDashboard.putData("ElevatorSetPosition2", new SetElevatorPositionCommand(Inches.of(12.0), esefSubsystem));
    SmartDashboard.putData("ElevatorSetPosition3", new SetElevatorPositionCommand(Inches.of(20.0), esefSubsystem));
    SmartDashboard.putData("ElevatorSetPositionHome", new SetElevatorPositionCommand(Inches.of(0.0), esefSubsystem));
    SmartDashboard.putData("move End Effector", new SetEndEffectorSpeedCommand(0.5, esefSubsystem));

    SmartDashboard.putData("PivotPosition2", new SetPivotPositionCommand(Degrees.of(45), afiSubsystem));
    SmartDashboard.putData("PivotPositionInit", new SetPivotPositionCommand(Degrees.of(0), afiSubsystem));
    SmartDashboard.putNumber("Elevator.ManualPosition", 5);
    SmartDashboard.putData("Elevator.ManualControl", new SetManualElevatorCommand());

    SmartDashboard.putData("PivotPosition2", new SetPivotPositionCommand(Degrees.of(10.0), afiSubsystem));
    SmartDashboard.putData("PivotPositionInit", new SetPivotPositionCommand(Degrees.of(00.), afiSubsystem));

    SmartDashboard.putData("AFISetRollerSpeed1", new AFIRollerSetSpeedCommand(0.1, afiSubsystem));
    SmartDashboard.putData("AFISetRollerSpeed2", new AFIRollerSetSpeedCommand(0.5, afiSubsystem));
    SmartDashboard.putData("AFIStopRoller", new AFIRollerSetSpeedCommand(0.0, afiSubsystem));

    // SmartDashboard.putData('CoralSpeed');

    // SmartDashboard.putData(new xxxxCommand());
    SmartDashboard.putData("climber:p1", new SetClimberPostionCommand(ClimberSubsystem.pos1, climberSubsystem));
    SmartDashboard.putData("climber:p2", new SetClimberPostionCommand(ClimberSubsystem.pos2, climberSubsystem));

    SmartDashboard.putData("Reset IMU from Limelight data", new SetIMUFromMegaTag1Command());

    SmartDashboard.putData("Drive 10 feet", swerveSubsystem.driveToDistanceCommand(Units.feetToMeters(10), 0.5));
    
  }

  SendableChooser<Command> chooser = new SendableChooser<>();

  public void setupAutonomousCommands() {
    SmartDashboard.putData("Auto mode", chooser);

    // chooser.addOption("Example Command", new ExampleCommand(exampleSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new GoldenAutoCommand(driveSubsystem, shooterSubsystem,
    // VisionSubsystem, intakeSubsystem);
    return chooser.getSelected();
  }

  /**
   * Determine if this robot is a competition robot.
   * <p>
   * <li>
   * <ul>
   * It is if it's connected to an FMS.
   * </ul>
   * <ul>
   * It is if it is missing a grounding jumper on DigitalInput 0.
   * </ul>
   * <ul>
   * It is if the robot_parameters.json says so for this MAC address.
   * </ul>
   * </li>
   * </p>
   *
   * @return true if this robot is a competition robot.
   */
  @SuppressWarnings({ "unused", "RedundantIfStatement", "PointlessBooleanExpression" })
  public static boolean amIACompBot() {
    if (DriverStation.isFMSAttached()) {
      return true;
    }

    if (practiceBotJumper.get() == true) {
      return true;
    }

    if (robotParameters.isCompetitionRobot()) {
      return true;
    }

    return false;
  }

  /**
   * Determine if we should make software objects, even if the device does
   * not appear on the CAN bus.
   * <p>
   * <li>
   * <ul>
   * We should if it's connected to an FMS.
   * </ul>
   * <ul>
   * We should if it is missing a grounding jumper on DigitalInput 0.
   * </ul>
   * <ul>
   * We should if the robot_parameters.json says so for this MAC address.
   * </ul>
   * </li>
   * </p>
   *
   * @return true if we should make all software objects for CAN devices
   */
  @SuppressWarnings({ "unused", "RedundantIfStatement" })
  public static boolean shouldMakeAllCANDevices() {
    if (amIACompBot()) {
      return true;
    }

    if (robotParameters.shouldMakeAllCANDevices()) {
      return true;
    }

    return false;
  }

  public static double getDriveVerticalJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_Y, FlySkyConstants.AXIS_LEFT_Y);
    double deadzone = 0.1;
    if (driverJoystick.getCurrentControllerType() == ControllerType.B) {
      deadzone = 0.02;
    }
    SmartDashboard.putNumber("driver.y.raw", axisValue);
    if (Math.abs(axisValue) < deadzone) {
      return 0;
    }
    return axisValue;
  }

  public static double getDriveHorizontalJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_X, FlySkyConstants.AXIS_LEFT_X);
    double deadzone = 0.05;
    if (driverJoystick.getCurrentControllerType() == ControllerType.B) {
      deadzone = 0.02;
    }
    SmartDashboard.putNumber("driver.x.raw", axisValue);
    if (Math.abs(axisValue) < deadzone) {
      return 0;
    }
    if (axisValue < 0) {
      return -(axisValue * axisValue);
    }
    return axisValue * axisValue;
  }

  public static double getDriveSpinJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_RIGHT_X, FlySkyConstants.AXIS_RIGHT_X);
    double deadzone = 0.05;
    if (driverJoystick.getCurrentControllerType() == ControllerType.B) {
      deadzone = 0.02;
    }
    
    SmartDashboard.putNumber("driver.spin.raw", axisValue);

    //axisValue = 0;

    double rv = 0;
    if (Math.abs(axisValue) >= deadzone) {
      rv = axisValue * axisValue;
      if (axisValue < 0) {
        rv = -rv;
      }
    }
    return axisValue;
  }

}