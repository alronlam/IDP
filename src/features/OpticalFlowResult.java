package features;

import java.util.List;

import org.opencv.core.MatOfPoint2f;

class OpticalFlowResult {
	private MatOfPoint2f currentFeatures;
	private MatOfPoint2f newFeatures;
	private List<Integer> badPointsIndex;
	
	
	OpticalFlowResult(MatOfPoint2f currentFeatures, MatOfPoint2f newFeatures, List<Integer> badPointsIndex) {
		this.currentFeatures = currentFeatures;
		this.newFeatures = newFeatures;
		this.badPointsIndex = badPointsIndex;
	}

	
	MatOfPoint2f getCurrentFeatures() {
		return currentFeatures;
	}

	
	MatOfPoint2f getNewFeatures() {
		return newFeatures;
	}

	
	List<Integer> getBadPointsIndex() {
		return badPointsIndex;
	}
	
	
	boolean isEmpty() {
		return currentFeatures.empty();
	}
}
