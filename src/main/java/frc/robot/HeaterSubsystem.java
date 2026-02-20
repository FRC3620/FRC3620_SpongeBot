// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HeaterSubsystem extends SubsystemBase {
  /** Creates a new HeaterSubsystem. */
  List<WPI_TalonSRX> heaters = new ArrayList<>();

  public HeaterSubsystem() {
    heaters.add(new WPI_TalonSRX(1));
    heaters.add(new WPI_TalonSRX(2));
    heaters.add(new WPI_TalonSRX(3));
    heaters.add(new WPI_TalonSRX(4));
  }

  void setHeaters(double value) {
    for (var heater : heaters) {
      heater.set(TalonSRXControlMode.PercentOutput, value);
    }
  }

  public Command makeSetSpeedCommand(double value) {
    return run(() -> setHeaters(value));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (var heater : heaters) {
      String name = "H" + heater.getDeviceID();

      double outputCurrent = heater.getStatorCurrent();
      double outputVoltage = heater.getMotorOutputVoltage();

      DogLog.log(name + "/output/v", outputVoltage);
      DogLog.log(name + "/output/a", outputCurrent);
      DogLog.log(name + "/output/w", outputCurrent*outputVoltage);
      
 double inputCurrent = heater.getSupplyCurrent();
      double inputVoltage = heater.getBusVoltage();

      DogLog.log(name + "/input/v", inputVoltage);
      DogLog.log(name + "/input/a", inputCurrent);
      DogLog.log(name + "/input/w", inputCurrent*inputVoltage);

      double setpoint = heater.getMotorOutputPercent();
      DogLog.log(name + "/setpoint", setpoint);
      // get other information here, and log to Doglog with appropriate names...
    }
  }
}
