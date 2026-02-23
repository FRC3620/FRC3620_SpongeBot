# Requirements

## Phase 1
* Log to wpilog input and output motor voltages, currents, and wattages for each of the 4 heaters (H1-H4). They should be logged to 
```
H1/input/v
H1/input/a
H1/input/w
H1/output/v
H1/output/a
H1/output/w
H2/input/v
(etc)
```
* Log to wpilog the setpoint of the heaters under `H1/setpoint`, `H2/setpoint`, etc.
* Write a command to repeatedly run the heaters at 0.5 power for 12 seconds, then off for 3 seconds. Put this command on the SmartDashboard.
* Log to wpilog the power distribution board voltage under `MAIN/v`.
## Phase 2
* Fix the testing command to end it's test when the battery voltage gets down to 11.5v.
## Phase 3
* Use a camera to look for a Apriltag of family 36H11. If the camera sees a tag with an id that is 200 or greater, it should log it to `BATTERY/id`. You will want to try out the wpilib example for Apriltag recognition.
## Phase 4
* Implement an onboard kill switch. Read a DIO, and while it is true, run a command that puts zero power on the heaters.

# Recommended implementation

* Use doglog for your logging. 
* Create a subsystem class for the heaters.
* Make sure you declare an instance of the subsystem and create one in `RobotContainer`. There are comments in there to help put things in an appropriate place.
* Inside the subsystem, make a member variable (declared inside the class, outside any members) to hold all the motor controller:
```java
List<WPI_TalonSRX> heaters = new ArrayList<>();
```
* Inside the constructor, create `WPI_TalonSRX` objects for each of the 4 motor controllers. Add them to the `heaters` list.
Now, if you need do something to all the heaters at once, you can do something like:
```java
void setHeaters(double value) {
  for (var heater : heaters) {
    heater.set(value);
  }
}
```

* Make a command factory that creates a Command to set all the heaters to a certain power (using the `setHeaters` shown above).
* Inside the `periodic()` for the subsystem, get information from the motor controllers, and put it out to the log file.

```java
public void periodic() {
  for (var heater : heaters) {
    String name = "H" + heater.getDeviceId();

    double outputCurrent = heater.getOutputCurrent();
    DogLog.log(name + "/output/a", outputCurrent)'

    // get other information here, and log to Doglog with appropriate names...
  }
}
```

* In `RobotContainer.setSmartDashboardCommands`, create a command that will set the motors to 0.5 power for 12 seconds, then 0 power for 3 seconds. Have it run forever.
* In `RobotContainer.setSmartDashboardCommands` (or possibly the heater subsystem), create a command that will set the motors to 0 power. Make it the default command for the subsystem.

# IO Assignments

## Digital IO
(none)

## Analog IO
(none)

## PWM
(none)

## CAN Bus

### CTRE Talon SRX
* id 1: heater 1
* id 2: heater 2
* id 3: heater 3
* id 4: heater 4
