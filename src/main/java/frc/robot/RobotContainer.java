package frc.robot;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc3620.logger.LoggingMaster;

import com.ctre.phoenix6.SignalLogger;

import dev.doglog.DogLog;

import org.usfirst.frc3620.CANDeviceFinder;
import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.RobotParametersContainer;
import org.usfirst.frc3620.Utilities;

import org.tinylog.TaggedLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Elastic;

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

  Alert missingDevicesAlert = new Alert("Diagnostics", "", Alert.AlertType.kWarning);

  // hardware here...
  public static PowerDistribution powerDistribution;

  // subsystems here
  HeaterSubsystem heaterSubsystem;
  BatteryIdentifierSubsystem batteryIdentifierSubsystem;
  SimulatedBatterySubsystem simulatedBatterySubsystem;
  DataRecorderSubsystem dataRecorderSubsystem;

  // joysticks here....

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // turn off CTRE hoot files
    SignalLogger.enableAutoLogging(false);

    canDeviceFinder = new CANDeviceFinder();
    for (var d : canDeviceFinder.getDeviceSet()) {
      logger.info("Have device {}", d);
    }

    robotParameters = RobotParametersContainer.getRobotParameters(RobotParameters.class);
    logger.info("got parameters for chassis '{}'", robotParameters.getName());
    Utilities.logMetadataToDataLog("Robot", robotParameters.getName());

    boolean iAmACompetitionRobot = amIACompBot();
    if (!iAmACompetitionRobot) {
      logger.warn("this is a test chassis, will try to deal with missing hardware!");
    }

    if (canDeviceFinder.isDevicePresent(CANDeviceType.CTRE_PDP, 0) || Robot.isSimulation()) {
      powerDistribution = new PowerDistribution(0, ModuleType.kCTRE);
      // DogLog.setPdh(powerDistribution);
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
  }

  private void makeSubsystems() {
    heaterSubsystem = new HeaterSubsystem();
    batteryIdentifierSubsystem = new BatteryIdentifierSubsystem();
    dataRecorderSubsystem = new DataRecorderSubsystem();
    if (Robot.isSimulation()) {
      simulatedBatterySubsystem = new SimulatedBatterySubsystem(powerDistribution);
      simulatedBatterySubsystem.addHeaters(heaterSubsystem.heaters);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  Elastic.Notification testIsCompleteNotification = new Elastic.Notification(Elastic.NotificationLevel.INFO, "Info",
      "Test Battery Command is complete.").withNoAutoDismiss();
  Elastic.Notification testWasInterruptedNotification = new Elastic.Notification(Elastic.NotificationLevel.INFO, "Info",
      "Test Battery Command was interrupted.").withNoAutoDismiss();
  Elastic.Notification noBatteryIdNotification = new Elastic.Notification(Elastic.NotificationLevel.ERROR, "Info",
      "Can't start test: need to know what battery we have!").withDisplaySeconds(10.0);

  private void setupSmartDashboardCommands() {
    // SmartDashboard.putData(new xxxxCommand());
    // SmartDashboard.putData("run motor",
    // heaterSubsystem.makeSetSpeedCommand(0.5).withName("Run
    // Motors").withTimeout(12));
    heaterSubsystem.setDefaultCommand(heaterSubsystem.makeSetSpeedCommand(0).withName("Stopped Motors"));

    Command startHeating = heaterSubsystem.makeSetSpeedCommand(0.90).withTimeout(48);
    Command timeout = heaterSubsystem.makeSetSpeedCommand(0.0).withTimeout(12);

    Command testBattery = startHeating.andThen(timeout).repeatedly()
        .until(() -> batteryVoltage < 10.6);
    Command logCutoffCriteria = Commands.runOnce(() -> DogLog.log("cutoff criteria", "batteryVoltage < 10.6"));

    BooleanConsumer notifyThatWeAreDone = interrupted -> {
      if (interrupted) {
        Elastic.sendNotification(testWasInterruptedNotification);
      } else {
        Elastic.sendNotification(testIsCompleteNotification);
      }
    };

    Command testAndNotify = logCutoffCriteria.andThen(testBattery).finallyDo(notifyThatWeAreDone);

    Command testOrNotifyAboutBatteryId = Commands.either( //
      testAndNotify, //
      Commands.runOnce(() -> Elastic.sendNotification(noBatteryIdNotification)), //
      () -> batteryIdentifierSubsystem.getBatteryId().isPresent()
    ).withName("Test Battery");
    SmartDashboard.putData(testOrNotifyAboutBatteryId);

    Command fancyTestAndNotify = new FancyTestCommand(heaterSubsystem).finallyDo(notifyThatWeAreDone);
    SmartDashboard.putData(Commands.either(  //
      fancyTestAndNotify, //
      Commands.runOnce(() -> Elastic.sendNotification(noBatteryIdNotification)), //
      () -> batteryIdentifierSubsystem.getBatteryId().isPresent() //
    ).withName("Fancy Test Battery"));

    SmartDashboard.putData(Commands.runOnce(() -> batteryIdentifierSubsystem.startVisionThread())
        .withName("Start Vision").ignoringDisable(true));
  }

  SendableChooser<Command> chooser = new SendableChooser<>();

  public void setupAutonomousCommands() {
    SmartDashboard.putData("Auto mode", chooser);
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
  public static boolean amIACompBot() {
    if (DriverStation.isFMSAttached()) {
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
  public static boolean shouldMakeAllCANDevices() {
    if (amIACompBot()) {
      return true;
    }

    if (robotParameters.shouldMakeAllCANDevices()) {
      return true;
    }

    return false;
  }

  static double batteryVoltage;

  class DataRecorderSubsystem extends SubsystemBase {
    int hb = 0;

    @Override
    public void periodic() {
      hb = hb + 1;
      heaterSubsystem.record(hb);

      if (powerDistribution != null) {
        batteryVoltage = powerDistribution.getVoltage();
        DogLog.log("pdb/v", batteryVoltage);
        DogLog.log("pdb/a", powerDistribution.getTotalCurrent());
        DogLog.log("pdb/w", powerDistribution.getTotalPower());
        DogLog.log("pdb/j", powerDistribution.getTotalEnergy());
        DogLog.log("pdb/hb", hb);
      } else {
        batteryVoltage = RobotController.getBatteryVoltage();
        if (simulatedBatterySubsystem != null) {
          DogLog.log("pdb/j", simulatedBatterySubsystem.getTotalEnergy());
          DogLog.log("pdb/hb", hb);
        }
      }

      DogLog.log("v", batteryVoltage);
      DogLog.log("hb", hb);
    }
  }

  public static double getBatteryVoltage() {
    return batteryVoltage;
  }
}