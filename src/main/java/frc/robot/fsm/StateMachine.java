package frc.robot.fsm;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import dev.doglog.DogLog;

public class StateMachine<T extends FSMState> {

  public final TaggedLogger logger = LoggingMaster.getLogger(getClass());

  private String name;
  private T currentState;

  public StateMachine(String name) {
    this.name = name;
    _setState(null);
  }

  public void setState(T newState) {
    logger.debug("{} getting forced from {} to {}", name, currentState, newState);
    _setState(newState);
  }

  private void _setState(T newState) {
    if (currentState != null) {
      currentState.onExit();
    }
    DogLog.log("fsm/" + name, newState == null ? "(null)" : newState.toString());
    if (newState != null) {
      newState.onEnter();
    }
    currentState = newState;
  }

  public void update() {
    @SuppressWarnings("unchecked")
    T nextState = (T) currentState.execute();
    if (nextState != null) {
      logger.debug("{} moving from {} to {}", name, currentState, nextState);
      _setState(nextState);
    }
  }

  public void shutdown() {
    if (currentState != null) {
      logger.debug("{} shutdown: moving from {} to {}", name, currentState, "(null)");
      currentState.onExit();
      currentState = null;
    }
  }

  public T getCurrentState() {
    return currentState;
  }
}