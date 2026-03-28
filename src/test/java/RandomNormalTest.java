import java.util.Random;

import org.junit.Test;

// make a test that does nothing so we just specify this in build.gradle
// (if we specify nothing, we get everything).
public class RandomNormalTest {
    @Test
    public void doSomeNumbers() {
        double desiredMean = 12.0;
        double desiredStdDev = 0.05; // Standard deviation, not variance

        Random random = new Random();

        for (int i = 0; i < 50; i++) {
            double customNormalValue = desiredMean + desiredStdDev * random.nextGaussian();
            System.out.println(i + "," + customNormalValue);
        }
    }
}
