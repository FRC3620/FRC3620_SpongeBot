package frc.robot;

import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;

import org.usfirst.frc3620.Utilities.SlidingWindowStats;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fsm.FSMState;
import frc.robot.fsm.StateMachine;

public class FancyTestCommand extends Command {
  HeaterSubsystem heaterSubsystem;
  StateMachine<FancyState> fsm;

  final static double UNLOADED_CUTOFF_VOLTAGE = v_for_soc(0.45);

  ZonedDateTime timeOfTestStart = null;

  static double v_for_soc(double soc) {
    return 11.75 + (soc * 1.25);
  }

  public FancyTestCommand(HeaterSubsystem heaterSubsystem) {
    this.heaterSubsystem = heaterSubsystem;
    addRequirements(heaterSubsystem);
    fsm = new StateMachine<>("FancyTest");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DogLog.log("cutoff criteria", "mean of last 10 idle batteryVoltage < " + UNLOADED_CUTOFF_VOLTAGE);
    fsm.setState(restState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    fsm.update();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    heaterSubsystem.setHeaterPower(0);
    fsm.setState(null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return fsm.getCurrentState() == doneState || fsm.getCurrentState() == null;
  }

  abstract class FancyState extends FSMState {
    String name;

    FancyState(String name) {
      this.name = name;
    }

    @Override
    public String toString() {
      return name;
    }
  }

  FancyState restState = new FancyState("reststate") {
    Timer timer = new Timer();
    SlidingWindowStats stats = new SlidingWindowStats(10);

    public void onEnter() {
      timer.reset();
      timer.start();
      heaterSubsystem.setHeaterPower(0);

      stats.clear();
    }

    @Override
    public FancyState execute() {
      stats.addValue(RobotContainer.getBatteryVoltage());
      double v = stats.getMean();
      DogLog.log("v_rest_mean_of_10", v);
      if (timer.hasElapsed(12)) {
        if (v > UNLOADED_CUTOFF_VOLTAGE) {
          return activeState;
        } else {
          return doneState;
        }
      }
      return null;
    }
  };

  FancyState activeState = new FancyState("activestate") {
    Timer timer = new Timer();

    public void onEnter() {
      if (timeOfTestStart == null) {
        ZoneId zoneId = ZoneId.of("America/Detroit");
        timeOfTestStart = ZonedDateTime.now(zoneId);
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss");
        String timeOfTestStart_text = timeOfTestStart.format(formatter);
        DogLog.log("test_start_time", timeOfTestStart_text);
      }
      timer.reset();
      timer.start();
    }

    @Override
    public FancyState execute() {
      if (RobotContainer.getBatteryVoltage() < 8.00) {
        return doneState;
      }
      if (timer.hasElapsed(48)) {
        return restState;
      }
      heaterSubsystem.setHeaterPower(0.9);
      return null;
    }
  };

  FancyState doneState = new FancyState("donestate") {
    public void onEnter() {
      heaterSubsystem.setHeaterPower(0.0);
    }

    @Override
    public FancyState execute() {
      return null;
    }
  };
}
