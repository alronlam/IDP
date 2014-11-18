package console_tests;

import java.io.File;
import java.io.FileWriter;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.Random;

import junit.framework.TestCase;
import ekf.Camera;
import ekf.EKF;
import ekf.PointTriple;

public class EKFTests extends TestCase {

	private EKF ekf;
	private final String TAG = "EKFTests";
	private Random random;
	private Camera camera;

	// just for logging
	private String root = "data";

	@Override
	protected void setUp() throws Exception {
		ekf = new EKF();
		random = new Random();
		camera = new Camera();
		// Pre-condition. Should assert these first or else following test is
		// invalid
		// assertTrue(EKF.P_DIAGONAL_INITIAL == 0.1);
		// assertTrue(EKF.VRV_VARIANCE == 0.01);
	}

	@Override
	protected void tearDown() throws Exception {
		ekf = null;
	}

	public void testAddFeature() {

		// do some random prediction first

		double vxP = random.nextGaussian();
		double vyP = random.nextGaussian();
		double vzP = random.nextGaussian();
		PointTriple vP = new PointTriple(vxP, vyP, vzP);

		double wxP = random.nextGaussian();
		double wyP = random.nextGaussian();
		double wzP = random.nextGaussian();
		PointTriple wP = new PointTriple(wxP, wyP, wzP);

		ekf.predict(vP, wP, 0.333);

		StringBuilder log = new StringBuilder();

		log.append(ekf.getStateVector() + "\r\n");
		log.append(ekf.getCovarianceMatrix() + "\r\n\n");

		// add a few features
		ekf.addFeature(5, 10, camera);

		log.append(ekf.getStateVector() + "\r\n");
		log.append(ekf.getCovarianceMatrix() + "\r\n\n");

		logEntries(new File("AddFeature.txt"), log.toString());

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
