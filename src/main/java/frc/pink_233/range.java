package frc.pink_233;

public class range {
public static double clip (double cmd, double max, double min) {
	if (cmd > max) {
		return max;
	}
	else if (cmd < min){
		return min;
	}
	return cmd;
	}
}