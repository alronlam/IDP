package console_tests;


public class TestDriver {
	static double x, y;

	public static void main(String args[]) {
		ConsoleEKFTests t = new ConsoleEKFTests();

		newTestCase();

		/*
		 * System.out.println("START PERFECT TEST CASE"); for (int i = 0; i < 5;
		 * i++) { PointDouble test = t.perfectTestCase(); x += test.getX() / 5;
		 * y += test.getY() / 5; } System.out.println("Error: " + x + ", " + y);
		 * System.out.println("END PERFECT TEST CASE");
		 * 
		 * newTestCase();
		 * 
		 * System.out.println("START RAND PERFECT TEST CASE"); for (int i = 0; i
		 * < 5; i++) { PointDouble test = t.perfectRandomTestCase(); x +=
		 * test.getX() / 5; y += test.getY() / 5; } System.out.println("Error: "
		 * + x + ", " + y); System.out.println("END RAND PERFECT TEST CASE");
		 * 
		 * newTestCase();
		 */

		System.out.println("START FLAT NOISY INS DISTANCE TEST CASE");
		for (int i = 1; i <= 5; i++) {
			System.out.println("Test " + i);
			t.noisyINSFlatDistanceTestCase();
		}
		System.out.println("END FLAT NOISY INS DISTANCE TEST CASE");

		newTestCase();

		System.out.println("START RAND NOISY INS DISTANCE TEST CASE");
		for (int i = 1; i <= 5; i++) {
			System.out.println("Test " + i);
			t.noisyINSGaussianDistanceTestCase();
		}
		System.out.println("END RAND NOISY INS DISTANCE TEST CASE");

		newTestCase();

		System.out.println("START FLAT NOISY INS HEADING TEST CASE");
		for (int i = 1; i <= 5; i++) {
			System.out.println("Test " + i);
			t.noisyINSFlatHeadingTestCase();
		}
		System.out.println("END FLAT NOISY INS HEADING TEST CASE");

		newTestCase();

		System.out.println("START RAND NOISY INS HEADING TEST CASE");
		for (int i = 1; i <= 5; i++) {
			System.out.println("Test " + i);
			t.noisyINSGaussianDistanceTestCase();
		}
		System.out.println("END RAND NOISY INS HEADING TEST CASE");
	}

	public static void newTestCase() {
		x = y = 0;
		System.out.println("\n");
	}
}