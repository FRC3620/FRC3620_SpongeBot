// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulatedBatterySubsystem extends SubsystemBase {
  double last_wattage = 0;
  double last_wattage_t = 0;

  final double batteryFullJoules = 17.2 * 12 * 3600;
  final double heaterOhms = 12.0 / 10.0;
  final double rInt = 0.020;

  double batteryJoules = batteryFullJoules;
  PDPSim pdpSim;

  List<TalonSRX> heaters = new ArrayList<>();
  Map<TalonSRX, TalonSRXSimCollection> heaterSims = new HashMap<>();
  Map<TalonSRX, Double> heaterResistance = new HashMap<>();

  DoubleEntry socEntry;

  /** Creates a new SimulatedBattery. */
  public SimulatedBatterySubsystem(PowerDistribution powerDistribution) {
    if (powerDistribution != null) {
      pdpSim = new PDPSim(powerDistribution);
    }
    DoubleTopic socTopic = NetworkTableInstance.getDefault().getDoubleTopic("sim/soc_setter");
    socEntry = socTopic.getEntry(1.0);
    socEntry.set(1.0);

    SmartDashboard.putData(Commands.runOnce(() -> { batteryJoules = batteryFullJoules; }).ignoringDisable(true).withName("Reset Simulated Battery"));
    SmartDashboard.putData(Commands.runOnce(() -> { setSOC(socEntry.get(1.0)); }).ignoringDisable(true).withName("Set Simulated SOC"));
  }

  public void addHeaters(Collection<? extends TalonSRX> heaters) {
    for (var heater : heaters) {
      this.heaters.add(heater);
      this.heaterSims.put(heater, new TalonSRXSimCollection(heater));
    }
  }

  @Override
  public void periodic() {
    double v_noload = calculateVoltageForStateOfCharge(batteryJoules / batteryFullJoules);
    DogLog.log("sim/v_noload", v_noload);

    heaterResistance.clear();
    for (var heater : heaters) {
      double power = heater.getMotorOutputPercent();
      if (power != 0) {
        assert power != 0;
      }
      double heater_r = (power != 0) ? (1.0 / power * heaterOhms) : Double.POSITIVE_INFINITY;
      DogLog.log("sim/H" + heater.getDeviceID() + "/r", heater_r);
      heaterResistance.put(heater, heater_r);
    }

    double sum_of_1_over_r = 0.0;
    for (Double r : heaterResistance.values()) {
      if (Double.isFinite(r)) {
        sum_of_1_over_r += (1.0 / r);
      }
    }
    double effective_r = (sum_of_1_over_r == 0) ? Double.POSITIVE_INFINITY : (1.0 / sum_of_1_over_r);
    DogLog.log("sim/heater_r", effective_r);

    double v_battery = v_noload;
    double i = 0.0;
    if (Double.isFinite(effective_r)) {
      v_battery = v_noload * (effective_r / (effective_r + rInt));
      i = v_battery / effective_r;
    }
    DogLog.log("sim/v_loaded", v_battery);
    DogLog.log("sim/i", i);

    for (var h : heaters) {
      var heaterSim = heaterSims.get(h);
      double r = heaterResistance.get(h);
      double i_h = 0.0;
      if (Double.isFinite(r)) {
        i_h = v_battery / r;
      }
      heaterSim.setSupplyCurrent(i_h);
      if (pdpSim != null) {
        pdpSim.setCurrent(h.getDeviceID(), i_h);
      }
      heaterSim.setBusVoltage(v_battery);
    }

    if (pdpSim != null) {
      pdpSim.setVoltage(v_battery);
    }
    RoboRioSim.setVInVoltage(v_battery);

    var w = v_noload * i;
    DogLog.log("sim/w", w);
    var average_w = (w + last_wattage) / 2.0;
    var t0 = Timer.getFPGATimestamp();
    var t = (t0 - last_wattage_t);
    last_wattage = w;
    last_wattage_t = t0;
    var j =  t * average_w;
    DogLog.log("sim/j_inc", j);
    DogLog.log("sim/j_inc_t", t);

    batteryJoules = batteryJoules - j;
    DogLog.log("sim/j", getTotalEnergy());
    DogLog.log("sim/j_remaining", batteryJoules);
    DogLog.log("sim/soc", batteryJoules / batteryFullJoules);
  }

  public void setSOC(double soc) {
    batteryJoules = soc * batteryFullJoules;
  }

  public double getTotalEnergy() {
    return batteryFullJoules - batteryJoules;
  }

  public static double calculateVoltageForStateOfCharge(double soc) {
    return 11.75 + (1.25 * soc);
  }
}
