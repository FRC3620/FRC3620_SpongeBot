// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HeaterSubsystem extends SubsystemBase {
  /** Creates a new HeaterSubsystem. */
  public List<WPI_TalonSRX> heaters = new ArrayList<>();
  PowerDistribution powerDistribution;
  int hb = 0;

  TaggedLogger logger = LoggingMaster.getLogger(getClass());

  double heaterPower = 0.00000001;

  Tracer tracer = null;

  public HeaterSubsystem() {
    powerDistribution = RobotContainer.powerDistribution;
    makeMotor(1);
    makeMotor(2);
    makeMotor(3);
    makeMotor(4);

    setHeaterPower(0);
  }

  void makeMotor(int deviceId) {
    WPI_TalonSRX talon = new WPI_TalonSRX(deviceId);
    heaters.add(talon);

    // https://v5.docs.ctr-electronics.com/en/latest/ch18_CommonAPI.html
    int fast=10;
    talon.setStatusFramePeriod(StatusFrame.Status_1_General, fast);
    talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, fast);
    talon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, fast);

    /*
    int slow=1000;
    talon.setStatusFramePeriod(StatusFrame.Status_6_Misc, slow);
    talon.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, slow);
    talon.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, slow);
    talon.setStatusFramePeriod(StatusFrame.Status_10_Targets, slow);
    talon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, slow);
    talon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, slow);
    talon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, slow);
    talon.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, slow);
    */

  }

  void setHeaterPower(double value) {
    if (value != heaterPower) {
      for (var heater : heaters) {
        heater.set(TalonSRXControlMode.PercentOutput, value);
      }
      heaterPower = value;
      DogLog.log("H/setpoint", value);
    }
  }

  public Command makeSetSpeedCommand(DoubleSupplier supplier) {
    return run(() -> setHeaterPower(supplier.getAsDouble()));
  }

  public Command makeSetSpeedCommand(double value) {
    return run(() -> setHeaterPower(value));
  }

  public void record(int hb) {
    double t0 = Timer.getFPGATimestamp();

    if (tracer != null) tracer.resetTimer();

    double[] currents = null;
    if (powerDistribution != null) {
      currents = powerDistribution.getAllCurrents();
      if (tracer != null) tracer.addEpoch("PDP currents");
    }

    double total_i = 0;
    double total_v = 0;
    int n_v = 0;

    for (var heater : heaters) {
      int deviceId = heater.getDeviceID();
      String name = "H" + deviceId;

      double outputCurrent = heater.getStatorCurrent();
      double outputVoltage = heater.getMotorOutputVoltage();
      if (tracer != null) tracer.addEpoch(name + " gather output");

      DogLog.log(name + "/output/v", outputVoltage);
      DogLog.log(name + "/output/a", outputCurrent);
      DogLog.log(name + "/output/w", outputCurrent*outputVoltage);
      DogLog.log(name + "/output/hb", hb);
      if (tracer != null) tracer.addEpoch(name + " log output");
      
      double inputCurrent = heater.getSupplyCurrent();
      double inputVoltage = heater.getBusVoltage();
      if (tracer != null) tracer.addEpoch(name + " gather input");

      DogLog.log(name + "/input/v", inputVoltage);
      DogLog.log(name + "/input/a", inputCurrent);
      DogLog.log(name + "/input/w", inputCurrent*inputVoltage);
      DogLog.log(name + "/input/hb", hb);
      if (tracer != null) tracer.addEpoch(name + " log input");

      total_i += inputCurrent;
      total_v += inputVoltage;
      n_v++;

      double setpoint = heater.getMotorOutputPercent();
      DogLog.log(name + "/setpoint", setpoint);
      if (tracer != null) tracer.addEpoch(name + " gather and log setpoint");

      // heater 1 is wired to PDP port 0, heater 2 to PDP port 1, etc.
      if (currents != null) {
        DogLog.log(name + "/pdb/a", currents[deviceId-1]);
        if (tracer != null) tracer.addEpoch(name + " log pdb current");
      }
    }
    
    var average_v = total_v / n_v;
    DogLog.log("H/v", average_v);
    DogLog.log("H/a", total_i);
    DogLog.log("H/w", total_i*average_v);



    if (tracer != null) {
      double t = Timer.getFPGATimestamp() - t0;
      DogLog.log("heater.periodic() time", t);
      if (t > 0.020) {
        logger.warn("heater.periodic() time = {}", t);
        tracer.printEpochs((s) -> logger.info("heater trace: {}", s));
      }
    }

    Command c = this.getCurrentCommand();
    DogLog.log("cmd", c == null ? "" : c.getName());
  }
}
