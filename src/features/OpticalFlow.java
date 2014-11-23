package features;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.KeyPoint;
import org.opencv.video.Video;


class OpticalFlow {
	private final Scalar BLACK = new Scalar(0);
	private final Scalar WHITE = new Scalar(255);
	private FeatureDetector detector = FeatureDetector.create(FeatureDetector.FAST);

	OpticalFlowResult getFeatures(Mat previousImage, Mat currentImage, MatOfPoint2f previousFeatures) {
		
		
		MatOfPoint2f currentFeatures = new MatOfPoint2f();
		MatOfByte statusMat = new MatOfByte();
		MatOfFloat errorMat = new MatOfFloat();
		
		Mat detectMask = currentImage.clone();
		detectMask.setTo(WHITE);
		List<Point> currentFeaturesList = new ArrayList<>();
		
		if (previousFeatures.size().height > 0) {
			Video.calcOpticalFlowPyrLK(previousImage, currentImage, previousFeatures, currentFeatures, statusMat, errorMat);
			currentFeaturesList = currentFeatures.toList();
			
			// draw mask for detection
			int index = 0;
			for (Byte item : statusMat.toList()) {
				if (item.intValue() == 1) {
					Core.circle(detectMask, currentFeaturesList.get(index), 10, BLACK, -1);
				}
				index++;
			}
		}
		
		// Detect new features
		
		MatOfKeyPoint rawNewFeatures = new MatOfKeyPoint();
		MatOfPoint2f newFeatures = new MatOfPoint2f();
		detector.detect(currentImage, rawNewFeatures, detectMask);
		if (rawNewFeatures.size().height > 0) {
			newFeatures = convert(rawNewFeatures);
		}
		
		// Find good features and bad features index
		
		MatOfPoint2f goodFeatures = new MatOfPoint2f();
		List<Integer> badPointsIndex = new ArrayList<>();
		
		if (previousFeatures.size().height > 0) {
			List<Byte> statusList = statusMat.toList();
			List<Point> goodFeaturesList = new ArrayList<>();
					
			for (int index = 0; index < statusList.size(); index++) {
				Byte status = statusList.get(index);
				if (status.intValue() == 1) {
					goodFeaturesList.add( currentFeaturesList.get(index) );
				} else {
					badPointsIndex.add(Integer.valueOf(index));
				}
			}
			goodFeatures.fromList(goodFeaturesList);
		}
		
		OpticalFlowResult result = new OpticalFlowResult(goodFeatures, newFeatures, badPointsIndex);
		return result;
	}
	
	private MatOfPoint2f convert(MatOfKeyPoint keyPoints) {
		KeyPoint[] keyPointsArray = keyPoints.toArray();
		Point[] pointsArray = new Point[keyPointsArray.length];

		for (int i = 0; i < keyPointsArray.length; i++) {
			pointsArray[i] = (Point) keyPointsArray[i].pt;
		}

		return new MatOfPoint2f(pointsArray);
	}
}
