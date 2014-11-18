package console_tests;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.Random;

import ekf.EKF;
import ekf.PointDouble;

public class TestUtility {
	private static Random rand = new Random();
	public static double round2Decimals(double value) {
		return roundDecimals(value, 2);
	}

	public static double roundDecimals(double value, int decimalPlaces) {
		BigDecimal bd = new BigDecimal(value);
		bd = bd.setScale(decimalPlaces, RoundingMode.HALF_UP);
		return bd.doubleValue();
	}
	
	public static String ekfPosition(EKF e) {
		return e.getCurrDevicePose().get_xPos() + ", " + e.getCurrDevicePose().get_yPos();
	}

	public static PointDouble getError(PointDouble t, EKF e) {
		return new PointDouble(t.getX() - e.getCurrDevicePose().get_xPos(), t.getY() - e.getCurrDevicePose().get_yPos());
	}

	public static double randGaussian() {
		return rand.nextGaussian();
	}
}
