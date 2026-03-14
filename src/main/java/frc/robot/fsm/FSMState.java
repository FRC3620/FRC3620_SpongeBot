package frc.robot.fsm;

public abstract class FSMState {
    public void onEnter() { }
    public abstract FSMState execute();
    public void onExit() { }
}