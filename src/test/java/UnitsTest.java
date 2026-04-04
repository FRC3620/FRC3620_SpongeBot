import org.junit.Test;

import edu.wpi.first.units.EnergyUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;

import static edu.wpi.first.units.Units.*;

// make a test that does nothing so we just specify this in build.gradle
// (if we specify nothing, we get everything).
public class UnitsTest {

    EnergyUnit wh = Units.derive(Joules).aggregate(3600).named("WH").symbol("WH").make();
    TimeUnit hour1 = Units.derive(Seconds).aggregate(3600).named("Hr1").symbol("Hr1").make();
    TimeUnit hour2 = Units.derive(Seconds).aggregate(60).named("Hr2").symbol("Hr2").make();

    EnergyUnit wh1 = Units.derive(Watts.mult(hour1)).named("WH1").symbol("WH1").make();
    //EnergyUnit wh2 = Units.derive(hour1.times(Watts)).named("WH2").symbol("WH2").make();
    EnergyUnit wh2 = Units.derive(Watts.mult(hour2)).named("WH2").symbol("WH2").make();

    EnergyUnit ws = Units.derive(Watts.mult(Units.Second)).named("WS").symbol("WS").make();



    @Test
    public void watthours_and_joules() {
        System.out.println(hour1.ofBaseUnits(3600));
        System.out.println(hour2.ofBaseUnits(3600));

        System.out.println("===");

        var batteryCapacity = Amps.of(1).times(Volts.of(1)).times(Seconds.of(3600));
        System.out.println(batteryCapacity);
        System.out.println(batteryCapacity.in(wh) + " " + wh.name());
        System.out.println(batteryCapacity.in(wh1) + " " +  wh1.name());
        System.out.println(batteryCapacity.in(wh2) + " " + wh2);
        System.out.println(batteryCapacity.in(ws) + " " + ws);

        System.out.println("===");

        batteryCapacity = wh.of(1);
        System.out.println(batteryCapacity);
        System.out.println(batteryCapacity.in(Joule) + " " + Joule);
        System.out.println(batteryCapacity.in(wh) + " " + wh.name());
        System.out.println(batteryCapacity.in(wh1) + " " +  wh1.
        name());
        System.out.println(batteryCapacity.in(wh2) + " " + wh2);
        System.out.println(batteryCapacity.in(ws) + " " + ws);
    }
}
