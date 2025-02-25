package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.io.File;
import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.tinylog.TaggedLogger;

import org.usfirst.frc3620.logger.LogCommand;
import org.usfirst.frc3620.logger.LoggingMaster;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FileVersionException;

import org.usfirst.frc3620.CANDeviceFinder;
import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.ChameleonController;
import org.usfirst.frc3620.ChameleonController.ControllerType;
import org.usfirst.frc3620.FlySkyConstants;
import org.usfirst.frc3620.JoystickAnalogButton;
import org.usfirst.frc3620.RobotParametersContainer;
import org.usfirst.frc3620.Utilities;
import org.usfirst.frc3620.XBoxConstants;

import frc.robot.ButtonBox.ButtonId;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commandfactories.AFISubsystemCommandFactory;
import frc.robot.commandfactories.ClimberCommandFactory;
import frc.robot.commandfactories.ESEFSubsystemCommandFactory;
import frc.robot.commandfactories.SwerveSubsystemCommandFactory;
import frc.robot.commands.esefcommands.SetElevatorPositionCommand;
import frc.robot.commands.esefcommands.SetEndEffectorSpeedCommand;
import frc.robot.commands.esefcommands.RunEndEffectorUntilCoralGone;
import frc.robot.commands.esefcommands.SetManualElevatorCommand;
import frc.robot.commands.esefcommands.RunEndEffectorUntilHasCoral;
import frc.robot.commands.esefcommands.SetESEFPositionCommand;
import frc.robot.commands.esefcommands.SetShoulderPositionCommand;
import frc.robot.commands.swervedrive.DriveToClosestStickCommand;
import frc.robot.commands.swervedrive.DriveToClosestStickCommand.WhichStick;
import frc.robot.commands.swervedrive.TestDriveToPoseCommand;
import frc.robot.subsystems.AFISubsystem;
import frc.robot.subsystems.BlinkySubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HealthSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.esefsubsystem.ESEFPosition;
import frc.robot.subsystems.VisionSubsystem.WhichBlueStick;
import frc.robot.subsystems.esefsubsystem.ESEFSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveInputStream;
import swervelib.imu.SwerveIMU;
import frc.robot.commands.ContinuousSetIMUFromMegaTag1Command;
import frc.robot.commands.HankPullTheTriggerCommand;
import frc.robot.commands.SetClimberPostionCommand;
import frc.robot.commands.SetIMUFromMegaTag1Command;
import frc.robot.commands.SetPivotPositionCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.AFI.AFIRollerSetSpeedCommand;
import frc.robot.commands.AFI.AFIRollerSetSpeedContinuousCommand;
import frc.robot.commands.AFI.AFIRollerSetSpeedUntilInCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

  private final SendableChooser<Command> autoChooser;

  public final static TaggedLogger logger = LoggingMaster.getLogger(RobotContainer.class);

  // need this
  public static CANDeviceFinder canDeviceFinder;
  public static RobotParameters robotParameters;

  Alert missingDevicesAlert = new Alert(HealthSubsystem.HARDWARE_ALERT_GROUP_NAME, "", Alert.AlertType.kError);

  // hardware here...
  // private static DigitalInput practiceBotJumper;

  public static PowerDistribution powerDistribution = null;
  public static PneumaticsModuleType pneumaticModuleType = null;

  // subsystems here
  public static ESEFSubsystem esefSubsystem;
  public static AFISubsystem afiSubsystem;
  public static SwerveSubsystem swerveSubsystem;
  public static HealthSubsystem healthSubsystem;
  public static ClimberSubsystem climberSubsystem;
  public static BlinkySubsystem blinkySubsystem;
  public static VisionSubsystem visionSubsystem;

  // command factories here
  public static AFISubsystemCommandFactory afiCommandFactory;
  public static SwerveSubsystemCommandFactory swerveCommandFactory;
  public static ESEFSubsystemCommandFactory esefCommandFactory;
  public static ClimberCommandFactory climberCommandFactory;

  // joysticks here....
  public static ChameleonController driverJoystick;
  public static Joystick operatorJoystick;
  public static GenericHID buttonboxHID;

  public static ButtonBox buttonBox;

  // We'll be using this
  public static AprilTagFieldLayout aprilTagFieldLayout;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * 
   * @throws ParseException
   * @throws IOException
   * @throws FileVersionException
   */
  public RobotContainer() throws FileVersionException, IOException, ParseException {

    canDeviceFinder = new CANDeviceFinder();

    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    robotParameters = RobotParametersContainer.getRobotParameters(RobotParameters.class);
    logger.info("got parameters for chassis '{}'", robotParameters.getName());
    Utilities.logMetadataToDataLog("Robot", robotParameters.getName());

    // practiceBotJumper = new DigitalInput(0);
    boolean iAmACompetitionRobot = amIACompBot();
    if (!iAmACompetitionRobot) {
      logger.warn("this is a test chassis, will try to deal with missing hardware!");
    }

    /*
     * if (canDeviceFinder.isDevicePresent(CANDeviceType.REV_PH, 1, "REV PH") ||
     * iAmACompetitionRobot) {
     * pneumaticModuleType = PneumaticsModuleType.REVPH;
     * } else if (canDeviceFinder.isDevicePresent(CANDeviceType.CTRE_PCM, 0,
     * "CTRE PCM")) {
     * pneumaticModuleType = PneumaticsModuleType.CTREPCM;
     * }
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

    makeCommandFactories();

    // Configure the button bindings
    configureButtonBindingsAndDefaultCommands();

    setupSmartDashboardCommands();

    if (swerveSubsystem != null) {
      autoChooser = AutoBuilder.buildAutoChooser();
    } else {
      autoChooser = null;
    }
    setupAutonomousCommands();

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    Utilities.addDataLogForNT("SmartDashboard/swerve");
  }

  private void makeSubsystems() {
    if (canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, 1, "Swerve Drive 1")
        || shouldMakeAllCANDevices()) {
      canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, 3, "Swerve Drive 3");
      canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, 5, "Swerve Drive 5");
      canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, 7, "Swerve Drive 7");
      canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 2, "Swerve Drive 2");
      canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 4, "Swerve Drive 4");
      canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 6, "Swerve Drive 6");
      canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 8, "Swerve Drive 8");

      String swerveFolder = robotParameters.getSwerveDirectoryName();
      if (swerveFolder == null)
        swerveFolder = "swerve/simulation";

      SmartDashboard.putString("frc3620/swerveFolder", swerveFolder);
      logger.info("using swerveFolder '{}'", swerveFolder);
      swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), swerveFolder));
      SmartDashboard.putData("frc3620/swerveSubsystem", swerveSubsystem);
    }

    esefSubsystem = new ESEFSubsystem();
    afiSubsystem = new AFISubsystem();
    SmartDashboard.putData("frc3620/AFI/afiSubsystem", afiSubsystem);
    climberSubsystem = new ClimberSubsystem();
    blinkySubsystem = new BlinkySubsystem();
    visionSubsystem = new VisionSubsystem();

    // need to create healthSubsystem LAST!!!!!!!
    healthSubsystem = new HealthSubsystem();
  }

  private void makeCommandFactories() {
    afiCommandFactory = new AFISubsystemCommandFactory(afiSubsystem);
    swerveCommandFactory = new SwerveSubsystemCommandFactory(swerveSubsystem);
    esefCommandFactory = new ESEFSubsystemCommandFactory(esefSubsystem);
    climberCommandFactory = new ClimberCommandFactory(climberSubsystem);
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
  private void configureButtonBindingsAndDefaultCommands() {
    driverJoystick = new ChameleonController(new Joystick(0));
    operatorJoystick = new Joystick(1);
    buttonboxHID = new Joystick(2);
    buttonBox = new ButtonBox(buttonboxHID);

    // make sure this command gets run when we start up
    CommandScheduler.getInstance().schedule(new ContinuousSetIMUFromMegaTag1Command());

    climberSubsystem.setDefaultCommand(climberCommandFactory.makeSetClimberPowerCommand(
        () -> MathUtil.applyDeadband(operatorJoystick.getRawAxis(XBoxConstants.AXIS_RIGHT_Y), 0.1))
        .withName("ControlClimberFromJoystick"));

    if (swerveSubsystem != null) {
      /*
       * Converts driver input into a field-relative ChassisSpeeds that is controlled
       * by angular velocity.
       */
      SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
          () -> getDriveVerticalJoystick() * -1,
          () -> getDriveHorizontalJoystick() * -1)
          .withControllerRotationAxis(() -> getDriveSpinJoystick() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

      /*
       * Clone's the angular velocity input stream and converts it to a fieldRelative
       * input stream.()
       */
      SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
          .withControllerHeadingAxis(() -> getDriveHorizontalJoystick(),
              () -> getDriveVerticalJoystick())
          .headingWhile(true);

      /*
       * Clone's the angular velocity input stream and converts it to a robotRelative
       * input stream.
       */
      SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
          .robotRelative(true)
          .allianceRelativeControl(false);

      SwerveInputStream driveRobotOrientedSlow = driveRobotOriented.copy()
          .scaleTranslation(0.3);

      SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
          () -> -getDriveVerticalJoystick(),
          () -> -getDriveHorizontalJoystick())
          .withControllerRotationAxis(() -> getDriveSpinJoystick() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

      Command driveFieldOrientedDirectAngle = swerveSubsystem.driveFieldOriented(driveDirectAngle);
      Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
      Command driveRobotOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveRobotOriented);
      Command driveSetpointGen = swerveSubsystem.driveWithSetpointGeneratorFieldRelative(
          driveDirectAngle);
      Command driveRobotOrientedSlowCommand = swerveSubsystem.driveFieldOriented(driveRobotOrientedSlow);

      if (RobotBase.isSimulation()) {
        // swerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
      } else {
        swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      }

      // driverJoystick.analogButton(XBoxConstants.AXIS_RIGHT_TRIGGER,
      // FlySkyConstants.AXIS_SWH)
      // .onTrue(swerveSubsystem.pathFinderCommand());

      driverJoystick.button(XBoxConstants.BUTTON_LEFT_BUMPER, FlySkyConstants.BUTTON_SWF)
                .whileTrue(driveRobotOrientedSlowCommand);
               
       driverJoystick.analogButton(XBoxConstants.AXIS_RIGHT_TRIGGER, FlySkyConstants.AXIS_SWH)
                .whileTrue(new HankPullTheTriggerCommand());
    }

    new JoystickAnalogButton(operatorJoystick, XBoxConstants.BUTTON_A)
        .onTrue(new AFIRollerSetSpeedUntilInCommand(0.5, afiSubsystem));

    new JoystickAnalogButton(operatorJoystick, XBoxConstants.BUTTON_B)
        .onTrue(new AFIRollerSetSpeedCommand(-0.05, afiSubsystem));

    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_LEFT_Y)
        .onTrue(new SetPivotPositionCommand(Degrees.of(70), afiSubsystem));

    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_LEFT_X)
        .onTrue(new SetPivotPositionCommand(Degrees.of(20), afiSubsystem));

    buttonBox.addButtonMapping(ButtonId.A1, new SetESEFPositionCommand(ESEFPosition.PresetPosition.L1.getPosition(), esefSubsystem), 
                                            new RunEndEffectorUntilCoralGone(0.9, esefSubsystem));
                    
    buttonBox.addButtonMapping(ButtonId.A2, new SetESEFPositionCommand(ESEFPosition.PresetPosition.L2.getPosition(), esefSubsystem), 
                                            new RunEndEffectorUntilCoralGone(0.9, esefSubsystem));

    buttonBox.addButtonMapping(ButtonId.A3, new SetESEFPositionCommand(ESEFPosition.PresetPosition.L3.getPosition(), esefSubsystem), 
                                            new RunEndEffectorUntilCoralGone(0.9, esefSubsystem));

    buttonBox.addButtonMapping(ButtonId.A4, new SetESEFPositionCommand(ESEFPosition.PresetPosition.L4.getPosition(), esefSubsystem), 
                                            new RunEndEffectorUntilCoralGone(0.9, esefSubsystem));

    buttonBox.addButtonMapping(ButtonId.D1, new SetESEFPositionCommand(ESEFPosition.PresetPosition.L1.getPosition(), esefSubsystem), 
                                            new RunEndEffectorUntilCoralGone(0.9, esefSubsystem));

    buttonBox.addButtonMapping(ButtonId.D2, new SetESEFPositionCommand(ESEFPosition.PresetPosition.L2.getPosition(), esefSubsystem), 
                                            new RunEndEffectorUntilCoralGone(0.9, esefSubsystem));

    buttonBox.addButtonMapping(ButtonId.D3, new SetESEFPositionCommand(ESEFPosition.PresetPosition.L3.getPosition(), esefSubsystem), 
                                            new RunEndEffectorUntilCoralGone(0.9, esefSubsystem));

    buttonBox.addButtonMapping(ButtonId.D4, new SetESEFPositionCommand(ESEFPosition.PresetPosition.L4.getPosition(), esefSubsystem), 
                                            new RunEndEffectorUntilCoralGone(0.9, esefSubsystem));
  }



  private void setupSmartDashboardCommands() throws FileVersionException, IOException, ParseException {
    // ESEF commands
    esefCommandFactory.setupSmartDashboardCommands();

    // AFI commands
    afiCommandFactory.setupSmartDashboardCommands();

    // Swerve commands
    if (swerveSubsystem != null) {
      swerveCommandFactory.setupSmartDashboardCommands();
    }

    // ESEF commands
    esefCommandFactory.setupSmartDashboardCommands();

    // climber commands
    climberCommandFactory.setupSmartDashboardCommands();

    if (swerveSubsystem != null) {
      SmartDashboard.putData("Reset IMU from Limelight data", new SetIMUFromMegaTag1Command());

      swerveCommandFactory.setupSmartDashboardCommands();
    }

  }

  public void setupAutonomousCommands() {
    if (autoChooser != null) {
      SmartDashboard.putData("Auto mode", autoChooser);
    }

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
    if (autoChooser != null) {
      return autoChooser.getSelected();
    }
    return null;
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

    /*
     * if (practiceBotJumper.get() == true) {
     * return true;
     * }
     */

    if (robotParameters.isCompetitionRobot()) {
      return true;
    }

    // right now, we only put roboRIO2s on a competition bot. This could change
    if (RobotBase.getRuntimeType() == RuntimeType.kRoboRIO2) {
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
    double deadband = 0.1;
    if (driverJoystick.getCurrentControllerType() == ControllerType.B) {
      deadband = 0.02;
    }
    SmartDashboard.putNumber("driver.y.raw", axisValue);
    axisValue = MathUtil.applyDeadband(axisValue, deadband);
    return axisValue;
  }

  public static double getDriveHorizontalJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_X, FlySkyConstants.AXIS_LEFT_X);
    double deadband = 0.05;
    if (driverJoystick.getCurrentControllerType() == ControllerType.B) {
      deadband = 0.02;
    }
    SmartDashboard.putNumber("driver.x.raw", axisValue);
    axisValue = MathUtil.applyDeadband(axisValue, deadband);
    return axisValue * axisValue * Math.signum(axisValue);
  }

  public static double getDriveSpinJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_RIGHT_X, FlySkyConstants.AXIS_RIGHT_X);
    double deadband = 0.05;
    if (driverJoystick.getCurrentControllerType() == ControllerType.B) {
      deadband = 0.02;
    }

    SmartDashboard.putNumber("driver.spin.raw", axisValue);

    axisValue = MathUtil.applyDeadband(axisValue, deadband);
    return axisValue * axisValue * Math.signum(axisValue);

  }

}