package frc.robot.Utilities;

public class EpsilonEquals {
    public static boolean epsilonEquals(double a, double b, double epsilon) {
		return (a - epsilon <= b) && (a + epsilon >= b);
	}
}
