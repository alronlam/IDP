package features;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;

public class FeatureManager {
	private static final String TAG = "Feature Manager";
	
	private Mat previousImage = new Mat();
	private int frames = 0;
	private MatOfPoint2f prevFeatures = new MatOfPoint2f();
	private OpticalFlow opticalFlow = new OpticalFlow();
	
	public FeatureManager() {}
	
	public FeatureUpdate getFeatureUpdate(Mat currentImage) {
		OpticalFlowResult opflowresult = opticalFlow.getFeatures(previousImage, currentImage, prevFeatures);
		
		MatOfPoint2f nextFeatures = new MatOfPoint2f();
		nextFeatures.push_back(opflowresult.getCurrentFeatures());
		nextFeatures.push_back(opflowresult.getNewFeatures());
		nextFeatures.copyTo(prevFeatures);
		currentImage.copyTo(previousImage);
		frames++;
		
		return new FeatureUpdate(opflowresult);
	}
}
