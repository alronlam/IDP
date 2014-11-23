import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import features.FeatureManager;
import features.FeatureUpdate;
import util.ImageIO;
import util.Log;

public class Test {
	static {
		System.loadLibrary( Core.NATIVE_LIBRARY_NAME );
	}
	
	// Makes sure OpenCV is working
	public static void main(String[] args) {
		new Test().opencvLoading();
		//new Test().continuousLoading();
		//new Test().opflow();
	}
	
	void opencvLoading() {
		Mat mat = Mat.eye( 3, 3, CvType.CV_8UC1 );
	    System.out.println( "mat = " + mat.dump() );
	}
	
	
	void continuousLoading() {
		ImageIO io = new ImageIO();
		while (io.hasNext()) {
			io.loadNext();
		}
	}
	
	
	void opflow() {
		ImageIO io = new ImageIO(false);
		FeatureManager fm = new FeatureManager();
		
		while (io.hasNext()) {
			Log.d("----------------");
			Mat image = io.loadNext();
			FeatureUpdate update = fm.getFeatureUpdate(image);
			Log.d("Bad Points: " + update.getBadPointsIndex().size());
			Log.d("Cur Points: " + update.getCurrentPoints().size());
			Log.d("New Points: " + update.getNewPoints().size());
			

		}	
	}
}
