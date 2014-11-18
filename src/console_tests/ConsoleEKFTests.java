package console_tests;

import java.util.Random;

import ekf.EKF;
import ekf.PointDouble;

public class ConsoleEKFTests {
	private EKF ekf;

	public PointDouble perfectTestCase() {
		ekf = new EKF();

		// System.out.println("Start: " + TestUtility.ekfPosition(ekf));

		ekf.addFeature(0, 5);

		ekf.predictFromINS(Math.sqrt(2), Math.PI / 4);
		// System.out.println("After INS 1: " + TestUtility.ekfPosition(ekf));

		ekf.updateFromReobservedFeatureCoords(0, 0, 5);
		// System.out.println("After Reobserve 1: " +
		// TestUtility.ekfPosition(ekf));

		ekf.predictFromINS(1, Math.PI / 2);
		// System.out.println("After INS 2: " + TestUtility.ekfPosition(ekf));

		ekf.updateFromReobservedFeatureCoords(0, 0, 5);
		// System.out.println("After Reobserve 2: " +
		// TestUtility.ekfPosition(ekf));

		PointDouble error = TestUtility.getError(new PointDouble(1, 2), ekf);
		return error;
	}

	public PointDouble perfectRandomTestCase() {
		int iterations = 5;
		Random r = new Random();
		PointDouble fin = new PointDouble(0, 0);

		ekf = new EKF();

		// System.out.println("Start: " + TestUtility.ekfPosition(ekf));

		ekf.addFeature(0, 10);

		for (int i = 1; i <= iterations; i++) {
			double dist = 1 + 0.5 * r.nextGaussian();
			double heading = Math.PI / 2;

			fin = fin.add(dist * Math.cos(heading), dist * Math.sin(heading));

			ekf.predictFromINS(dist, heading);
			// System.out.println("After INS " + i + ": " +
			// TestUtility.ekfPosition(ekf));

			ekf.updateFromReobservedFeatureCoords(0, 0, 10);
			// System.out.println("After Reobs " + i + ": " +
			// TestUtility.ekfPosition(ekf));
			// System.out.println("Actual at " + i + ": " + fin);
		}

		PointDouble error = TestUtility.getError(fin, ekf);
		return error;
	}

	/*
	 * Noisy INS, Perfect VINS INS is ALWAYS off by a FLAT 0.2 meters
	 */
	public void noisyINSFlatDistanceTestCase() {
		int iterations = 5;

		ekf = new EKF();

		PointDouble currPos = new PointDouble(0, 0);
		PointDouble INSPos = new PointDouble(0, 0);

		// System.out.println("Start: " + TestUtility.ekfPosition(ekf));

		ekf.addFeature(0, 10);

		for (int i = 1; i <= iterations; i++) {
			double dist = 1;
			double heading = Math.PI / 2;

			currPos = currPos.add(0, dist);
			INSPos = INSPos.add(0, dist + 0.2);

			ekf.predictFromINS(dist + 0.2, heading);
			// System.out.println("After INS " + i + ": " +
			// TestUtility.ekfPosition(ekf));

			double observedDistance = 10 - currPos.getY();
			double observedHeading = heading;

			ekf.updateFromReobservedFeatureThroughDistanceHeading(0, observedDistance, observedHeading);
			// System.out.println("After Reobs " + i + ": " +
			// TestUtility.ekfPosition(ekf));
			//
			// System.out.println("INS only at " + i + ": " + INSPos);
			// System.out.println("Actual at " + i + ": " + currPos);
			// System.out.println("Heading at " + i + ": " +
			// ekf.getCurrDevicePose().getHeading());
		}

		System.out.println("Actual	: " + currPos);
		System.out.println("INS only: " + INSPos);
		System.out.println("EKF 	: " + ekf);
		// return TestUtility.getError(INSPos, ekf);
	}

	/*
	 * Noisy INS, Perfect VINS INS is off by a random distribution mean = 0,
	 * stdev = 0.01
	 */
	public void noisyINSGaussianDistanceTestCase() {
		int iterations = 5;
		Random r = new Random();

		PointDouble currPos = new PointDouble(0, 0);
		PointDouble INSPos = new PointDouble(0, 0);

		ekf = new EKF();

		// System.out.println("Start: " + TestUtility.ekfPosition(ekf));

		ekf.addFeature(0, 10);

		for (int i = 1; i <= iterations; i++) {
			double dist = 1;
			double heading = Math.PI / 2;
			double rand = 0.01 * r.nextGaussian();

			currPos = currPos.add(0, dist);
			INSPos = INSPos.add(0, dist + rand);

			ekf.predictFromINS(dist + rand, heading);
			// System.out.println("After INS " + i + ": " +
			// TestUtility.ekfPosition(ekf));

			double observedDistance = 10 - currPos.getY();
			double observedHeading = heading;

			ekf.updateFromReobservedFeatureThroughDistanceHeading(0, observedDistance, observedHeading);
			// System.out.println("After Reobs " + i + ": " +
			// TestUtility.ekfPosition(ekf));
			//
			// System.out.println("INS only at " + i + ": " + INSPos);
			// System.out.println("Actual at " + i + ": " + currPos);
			//
			// System.out.println("Heading at " + i + ": " +
			// ekf.getCurrDevicePose().getHeading());
		}

		System.out.println("Actual	: " + currPos);
		System.out.println("INS only: " + INSPos);
		System.out.println("EKF 	: " + ekf);
		// return TestUtility.getError(INSPos, ekf);
	}

	/*
	 * Noisy INS, Perfect VINS INS Heading is ALWAYS off by a FLAT 3 degrees
	 */
	public void noisyINSFlatHeadingTestCase() {
		int iterations = 5;

		ekf = new EKF();

		PointDouble currPos = new PointDouble(0, 0);
		PointDouble INSPos = new PointDouble(0, 0);

		// System.out.println("Start: " + TestUtility.ekfPosition(ekf));

		ekf.addFeature(0, 10);

		for (int i = 1; i <= iterations; i++) {
			double dist = 1;
			double heading = Math.PI / 2;

			currPos = currPos.add(0, dist);
			INSPos = INSPos.add(dist * Math.cos(heading + 3 * Math.PI / 180),
					dist * Math.sin(heading + 3 * Math.PI / 180));

			ekf.predictFromINS(dist, heading + 3 * Math.PI / 180);
			// System.out.println("After INS " + i + ": " +
			// TestUtility.ekfPosition(ekf));

			double observedDistance = 10 - currPos.getY();
			double observedHeading = heading;

			ekf.updateFromReobservedFeatureThroughDistanceHeading(0, observedDistance, observedHeading);
			// System.out.println("After Reobs " + i + ": " +
			// TestUtility.ekfPosition(ekf));
			//
			// System.out.println("INS only at " + i + ": " + INSPos);
			// System.out.println("Actual at " + i + ": " + currPos);
			// System.out.println("Heading at " + i + ": " +
			// ekf.getCurrDevicePose().getHeading());
		}

		System.out.println("Actual	: " + currPos);
		System.out.println("INS only: " + INSPos);
		System.out.println("EKF 	: " + ekf);
		// return TestUtility.getError(INSPos, ekf);
	}

	/*
	 * Noisy INS, Perfect VINS INS is off by a random distribution in RADIANS
	 * mean = 0, stdev = 0.1
	 */
	public void noisyINSGaussianHeadingTestCase() {
		int iterations = 5;
		Random r = new Random();

		PointDouble currPos = new PointDouble(0, 0);
		PointDouble INSPos = new PointDouble(0, 0);

		ekf = new EKF();

		// System.out.println("Start: " + TestUtility.ekfPosition(ekf));

		ekf.addFeature(0, 10);

		for (int i = 1; i <= iterations; i++) {
			double dist = 1;
			double heading = Math.PI / 2;
			double rand = 0.1 * r.nextGaussian();

			currPos = currPos.add(0, dist);
			INSPos = INSPos.add(dist * Math.cos(heading + rand), dist * Math.sin(heading + rand));

			ekf.predictFromINS(dist, heading + rand);
			// System.out.println("After INS " + i + ": " +
			// TestUtility.ekfPosition(ekf));

			double observedDistance = 10 - currPos.getY();
			double observedHeading = heading;

			ekf.updateFromReobservedFeatureThroughDistanceHeading(0, observedDistance, observedHeading);
			// System.out.println("After Reobs " + i + ": " +
			// TestUtility.ekfPosition(ekf));
			//
			// System.out.println("INS only at " + i + ": " + INSPos);
			// System.out.println("Actual at " + i + ": " + currPos);
			//
			// System.out.println("Heading at " + i + ": " +
			// ekf.getCurrDevicePose().getHeading());
		}

		System.out.println("Actual	: " + currPos);
		System.out.println("INS only: " + INSPos);
		System.out.println("EKF 	: " + ekf);
		// return TestUtility.getError(INSPos, ekf);
	}
}