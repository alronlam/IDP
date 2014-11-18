package console_tests;

import java.io.File;
import java.io.FileWriter;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.Random;

import junit.framework.TestCase;
import ekf.DevicePose;
import ekf.EKF;
import ekf.PointDouble;

public class EKFTests extends TestCase {

	private EKF ekf;
	private final String TAG = "EKFTests";
	private Random random;

	// just for logging
	private String root = "data";

	@Override
	protected void setUp() throws Exception {
		ekf = new EKF();
		random = new Random();
		// Pre-condition. Should assert these first or else following test is
		// invalid
		// assertTrue(EKF.P_DIAGONAL_INITIAL == 0.1);
		// assertTrue(EKF.VRV_VARIANCE == 0.01);
	}

	@Override
	protected void tearDown() throws Exception {
		ekf = null;
	}

	/**
	 * Tests that the predict step using INS updates is correct. Checks X (state
	 * vector) and P (covariance matrix).
	 */
	public void testInsPredict() {

		// INS Update 1
		ekf.predictFromINS(Math.sqrt(2), Math.PI / 4);

		// Asserts that X was updated correctly

		DevicePose devicePose = ekf.getCurrDevicePose();
		assertEquals("X should be 1", 1.0, round2Decimals(devicePose.get_xPos()));
		assertEquals("Y should be 1", 1.0, round2Decimals(devicePose.get_yPos()));
		assertEquals("Heading should be 45", 45.0, round2Decimals(devicePose.getHeading()));

		// Asserts that P was updated correctly.
		ArrayList<ArrayList<Double>> P = ekf.getP();

		assertEquals("P[0][0] should be 0.3", 0.3, roundDecimals(P.get(0).get(0), 4));
		assertEquals("P[0][1] should be 0", 0.0, roundDecimals(P.get(0).get(1), 4));
		assertEquals("P[0][2] should be -0.0215", -0.0215, roundDecimals(P.get(0).get(2), 4));
		assertEquals("P[1][0] should be 0", 0.0, roundDecimals(P.get(1).get(0), 4));
		assertEquals("P[1][1] should be 0.3", 0.3, roundDecimals(P.get(1).get(1), 4));
		assertEquals("P[1][2] should be 0.1785", 0.1785, roundDecimals(P.get(1).get(2), 4));
		assertEquals("P[2][0] should be -0.0215", -0.0215, roundDecimals(P.get(2).get(0), 4));
		assertEquals("P[2][1] should be 0.1785", 0.1785, roundDecimals(P.get(2).get(1), 4));
		assertEquals("P[2][2] should be 0.1617", 0.1617, roundDecimals(P.get(2).get(2), 4));
	}

	/**
	 * Tests that adding a feature is correct. Checks X (state vector) and P
	 * (covariance matrix). This is just a very simple test case where this
	 * feature to be added is the only feature being tracked.
	 */
	public void testAddOneFeature() {

		ekf.addFeature(0, 1);

		// Check X (state vector) contents
		ArrayList<Double> X = ekf.getX();

		assertEquals("X's size should be 5.", 5, X.size());
		assertEquals("X[3] should be 0", 0.0, X.get(3));
		assertEquals("X[4] should be 1", 1.0, X.get(4));

		// Check P (covariance matrix) contents

		// Check P size
		ArrayList<ArrayList<Double>> P = ekf.getP();
		int rows = P.size();
		assertEquals("P's rows should be 5", 5, rows);

		for (int i = 0; i < rows; i++) {
			ArrayList<Double> row = P.get(i);
			assertEquals("Row " + i + " should have 5 columns.", 5, row.size());
		}

		// Check the actual contents

		// lower-left
		assertEquals("P[3][0] should be 0.1", 0.1, roundDecimals(P.get(3).get(0), 1));
		assertEquals("P[3][1] should be 0.0", 0.0, roundDecimals(P.get(3).get(1), 1));
		assertEquals("P[3][2] should be 0.0", 0.0, roundDecimals(P.get(3).get(2), 1));
		assertEquals("P[4][0] should be 0.0", 0.0, roundDecimals(P.get(4).get(0), 1));
		assertEquals("P[4][1] should be 0.1", 0.1, roundDecimals(P.get(4).get(1), 1));
		assertEquals("P[4][2] should be 0.0", 0.0, roundDecimals(P.get(4).get(2), 1));

		// upper-right (should be the transpose of lower-left)
		assertEquals("P[0][3] should be 0.1", 0.1, roundDecimals(P.get(0).get(3), 1));
		assertEquals("P[0][4] should be 0.0", 0.0, roundDecimals(P.get(0).get(4), 1));
		assertEquals("P[1][3] should be 0.0", 0.0, roundDecimals(P.get(1).get(3), 1));
		assertEquals("P[1][4] should be 0.1", 0.1, roundDecimals(P.get(1).get(4), 1));
		assertEquals("P[2][3] should be 0.0", 0.0, roundDecimals(P.get(2).get(3), 1));
		assertEquals("P[2][4] should be 0.0", 0.0, roundDecimals(P.get(2).get(4), 1));

		// lower-right
		// cannot assert P[3][3]'s value because it is random
		assertEquals("P[3][4] should be 0.0", 0.0, P.get(3).get(4));
		assertEquals("P[4][3] should be 0.0", 0.0, P.get(4).get(3));
		assertEquals("P[4][4] should be 0.1", 0.1, P.get(4).get(4));

	}

	/**
	 * This test just checks if re-observing one feature works (no exceptions)
	 */
	public void testReobserveFeature() {
		// Log.d(TAG, "TestReobserveFeature Start:" +
		// ekf.getCurrDevicePose().toString());
		ekf.addFeature(0, 1);
		ekf.predictFromINS(Math.sqrt(2), Math.PI / 4);
		// Log.d(TAG, "TestReobserveFeature After INS:" +
		// ekf.getCurrDevicePose().toString());
		ekf.updateFromReobservedFeatureThroughDistanceHeading(0, 0.1, 1.1);
		// Log.d(TAG, "TestReobserveFeature After Reobserve Feature:" +
		// ekf.getCurrDevicePose().toString());
	}

	/**
	 * Case where everything is perfect (prediction + correction). Actual
	 * movement is sqrt(2) units 45deg, then one unit 90deg
	 */
	public void testPerfectCase() {

		// Log.d(TAG, "TestPerfectCase Start: " +
		// ekf.getCurrDevicePose().toString());
		ekf.addFeature(0, 5);
		ekf.predictFromINS(Math.sqrt(2), Math.PI / 4);

		// Log.d(TAG, "TestPerfectCase After INS 1: " +
		// ekf.getCurrDevicePose().toString());
		ekf.updateFromReobservedFeatureCoords(0, 0, 5);

		// Log.d(TAG, "TestPerfectCase After Reobserve 1: " +
		// ekf.getCurrDevicePose().toString());

		ekf.predictFromINS(1, Math.PI / 2);

		// Log.d(TAG, "TestPerfectCase After INS 2: " +
		// ekf.getCurrDevicePose().toString());
		ekf.updateFromReobservedFeatureCoords(0, 0, 5);

		// Log.d(TAG, "TestPerfectCase After Reobserve 2: " +
		// ekf.getCurrDevicePose().toString());

		assertEquals("Device X should be 1", 1.0, round2Decimals(ekf.getCurrDevicePose().get_xPos()));
		assertEquals("Device X should be 2", 2.0, round2Decimals(ekf.getCurrDevicePose().get_yPos()));
	}

	/**
	 * Case where INS is inaccurate but VINS is accurate, to check if the V-INS,
	 * assuming it works perfectly, can correct faulty INS estimates. Actual
	 * movement is sqrt(2) units 45deg, then one unit 90deg. VINS always
	 * re-observes the feature at (0,50) but INS estimates are wrong.
	 */
	public void testBadINSGoodVINSUsingReobservedFeatureCoords() {

		StringBuilder log = new StringBuilder();
		StringBuilder logWithoutEKF = new StringBuilder();

		int iterations = 17;

		/* Calculate correct coordinates after everything is done */
		double correctDistance = 1;
		double correctHeadingRads = Math.toRadians(90);

		PointDouble correctCoords = new PointDouble(0, 0);

		/* Initialize Feature */
		double featureX = 0;
		double featureY = 20;
		PointDouble featureCoords = new PointDouble(featureX, featureY);
		ekf.addFeature(featureCoords.getX(), featureCoords.getY());

		PointDouble deviceCoordsWithoutCorrection = new PointDouble(0, 0);

		/* Some logs */
		// Log.d(TAG, "TestBadINSGoodVINS Start: " +
		// ekf.getCurrDevicePose().toString());
		log.append(ekf.getCurrDevicePose().toString() + "\r\n");
		logWithoutEKF.append(deviceCoordsWithoutCorrection.toString() + "\r\n");

		// Simulate
		for (int i = 1; i <= iterations; i++) {

			correctCoords = correctCoords.add(correctDistance * Math.cos(correctHeadingRads),
					correctDistance * Math.sin(correctHeadingRads));

			double insPredictedDistance = correctDistance + new Random().nextGaussian();
			double insPredictedRadians = correctHeadingRads + new Random().nextGaussian();

			deviceCoordsWithoutCorrection = deviceCoordsWithoutCorrection.add(
					insPredictedDistance * Math.cos(insPredictedRadians),
					insPredictedDistance * Math.sin(insPredictedRadians));

			logWithoutEKF.append(deviceCoordsWithoutCorrection.toString() + "\r\n");

			ekf.predictFromINS(insPredictedDistance, insPredictedRadians);
			// Log.d(TAG, "TestBadINSGoodVINS After INS " + i +
			// ": Covariance\n " + ekf.getP().toString() + "\n");

			double observedDistance = correctCoords.computeDistanceTo(featureCoords);
			double observedHeading = correctCoords.computeRadiansTo(featureCoords);
			ekf.updateFromReobservedFeatureThroughDistanceHeading(0, observedDistance, observedHeading);
			// ekf.updateFromReobservedFeatureCoords(0, featureX, featureY);

			log.append(ekf.getCurrDevicePose().toString() + "\r\n");

		}

		double errorWithEKF = correctCoords.computeDistanceTo(ekf.getDeviceCoords());
		double errorWithoutEKF = correctCoords.computeDistanceTo(deviceCoordsWithoutCorrection);
		double improvement = errorWithoutEKF - errorWithEKF;

		// Log expected error w/o VINS, w/ VINS, and the improvement with VINS
		// over pure INS

		// logEntriesWithoutEKF(logWithoutEKF.toString(), iterations);
		// logEntriesWithEKF(log.toString(), iterations, featureY, improvement);
		System.out.println("Improvement is " + improvement);
		// Improvements should be positive to mean that VINS affected the
		// estimates positively
		assertTrue(improvement > 0);
	}

	public void testBadINSBadVINSUsingReobservedFeatureCoords() {

		StringBuilder log = new StringBuilder();
		StringBuilder logWithoutEKF = new StringBuilder();
		StringBuilder generalLogs = new StringBuilder();

		double noiseStdDev = 0.5;
		int iterations = 70;

		/* Calculate correct coordinates after everything is done */
		double correctDistance = 1;
		double correctHeadingRads = Math.toRadians(30);

		PointDouble correctCoords = new PointDouble(0, 0);

		/* Initialize Feature */
		double featureX = 40;
		double featureY = 70;
		PointDouble featureCoords = new PointDouble(featureX, featureY);
		ekf.addFeature(featureCoords.getX(), featureCoords.getY());

		PointDouble deviceCoordsWithoutCorrection = new PointDouble(0, 0);

		/* Some logs */
		generalLogs.append("Noisy INS + Noisy V-INS\r\n");
		generalLogs.append("Noise Stddev = " + noiseStdDev);
		generalLogs.append("Parameter Config:\r\n" + ekf.getParameterConfigurations() + "\r\n");
		log.append(ekf.getCurrDevicePose().toString() + "\n");
		logWithoutEKF.append(deviceCoordsWithoutCorrection.toString() + "\n");

		// Simulate
		for (int i = 1; i <= iterations; i++) {

			/* Compute correct coords */
			correctCoords = correctCoords.add(correctDistance * Math.cos(correctHeadingRads),
					correctDistance * Math.sin(correctHeadingRads));

			/* Simulate noisy INS distance */
			double insPredictedDistance = correctDistance + random.nextGaussian() * noiseStdDev;
			double insPredictedRadians = correctHeadingRads + random.nextGaussian() * noiseStdDev;

			deviceCoordsWithoutCorrection = deviceCoordsWithoutCorrection.add(
					insPredictedDistance * Math.cos(insPredictedRadians),
					insPredictedDistance * Math.sin(insPredictedRadians));

			logWithoutEKF.append(deviceCoordsWithoutCorrection.toString() + "\n");

			ekf.predictFromINS(insPredictedDistance, insPredictedRadians);

			generalLogs.append("TestBadINSGoodVINS After INS Predict " + i + ": Covariance\n " + ekf.getP().toString()
					+ "\r\n");

			/* Simulate noisy feature measurements */
			double observedDistance = correctCoords.computeDistanceTo(featureCoords) + random.nextGaussian()
					* noiseStdDev;
			double observedHeading = correctCoords.computeRadiansTo(featureCoords) + random.nextGaussian()
					* noiseStdDev;
			ekf.updateFromReobservedFeatureThroughDistanceHeading(0, observedDistance, observedHeading);

			generalLogs.append("TestBadINSGoodVINS After Feature Update " + i + ": Covariance\n "
					+ ekf.getP().toString() + "\r\n");

			log.append(ekf.getCurrDevicePose().toString() + "\n");

		}

		double errorWithEKF = correctCoords.computeDistanceTo(ekf.getDeviceCoords());
		double errorWithoutEKF = correctCoords.computeDistanceTo(deviceCoordsWithoutCorrection);
		double improvement = errorWithoutEKF - errorWithEKF;

		// Log expected error w/o VINS, w/ VINS, and the improvement with VINS
		// over pure INS
		generalLogs.append("TestBadINSBadVINS Without EKF Error = " + errorWithoutEKF);
		generalLogs.append("TestBadINSBadVINS With EKF Error = " + errorWithEKF);
		generalLogs.append("TestBadINSBadVINS With EKF Improvement = " + improvement);

		// logEntriesWithoutEKF(logWithoutEKF.toString(), iterations);
		// logEntriesWithEKF(log.toString(), iterations, featureY, improvement);
		// logEntries(new File(root + "/GeneralLog.txt"),
		// generalLogs.toString());

		System.out.println("Improvement is " + improvement);
		// Improvements should be positive to mean that VINS affected the
		// estimates positively
		assertTrue(improvement > 0);
	}

	private void logEntries(File file, String log) {
		try {
			if (!file.exists())
				file.createNewFile();

			FileWriter fw = new FileWriter(file);
			fw.write(log);
			fw.close();
			// FileOutputStream outputStream = new FileOutputStream(file);
			//
			// outputStream.write(log.getBytes());
			//
			// outputStream.close();

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private void logEntriesWithoutEKF(String log, int numIterations) {
		String fileDirectory = root + "/woekf_" + numIterations + "its_" + System.currentTimeMillis() + ".csv";
		File fileSummarized = new File(fileDirectory);

		logEntries(fileSummarized, log);
	}

	private void logEntriesWithEKF(String log, int numIterations, double featureY, double improvement) {
		String fileDirectory = root + "/ekf_" + numIterations + "its_" + System.currentTimeMillis() + "_featureY"
				+ featureY + "m_imp_" + improvement + ".csv";
		File fileSummarized = new File(fileDirectory);

		logEntries(fileSummarized, log);
	}

	private double round2Decimals(double value) {
		return roundDecimals(value, 2);
	}

	private double roundDecimals(double value, int decimalPlaces) {
		BigDecimal bd = new BigDecimal(value);
		bd = bd.setScale(decimalPlaces, RoundingMode.HALF_UP);
		return bd.doubleValue();
	}

}
