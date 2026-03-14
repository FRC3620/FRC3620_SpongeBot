package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fsm.FSMState;
import frc.robot.fsm.StateMachine;

public class FancyTestCommand extends Command {
  HeaterSubsystem heaterSubsystem;
  StateMachine<FancyState> fsm;

  final static double UNLOAD_VOLTAGE_AT_50_PERCENT_SOC = (11.75 + 13.00) / 2.0; // 11.75 is 0% SOC, 13 is 100%

  public FancyTestCommand(HeaterSubsystem heaterSubsystem) {
    this.heaterSubsystem = heaterSubsystem;
    addRequirements(heaterSubsystem);
    fsm = new StateMachine<>("FancyTest");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DogLog.log("cutoff criteria", "idle batteryVoltage < " + UNLOAD_VOLTAGE_AT_50_PERCENT_SOC);
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

    public void onEnter() {
      timer.reset();
      timer.start();
      heaterSubsystem.setHeaterPower(0);
    }

    @Override
    public FancyState execute() {
      if (timer.hasElapsed(12)) {
        if (RobotContainer.getBatteryVoltage() > UNLOAD_VOLTAGE_AT_50_PERCENT_SOC) {
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
      timer.reset();
      timer.start();
    }

    @Override
    public FancyState execute() {
      heaterSubsystem.setHeaterPower(0.9);
      if (timer.hasElapsed(48)) {
        return restState;
      }
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
