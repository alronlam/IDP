package features;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Point;

import ekf.PointDouble;

public class FeatureUpdate {
	private List<Integer> badPointsIndex;
	private List<PointDouble> currentPoints;
	private List<PointDouble> newPoints;
	
	public FeatureUpdate(){
		badPointsIndex = new ArrayList<>();
		currentPoints = new ArrayList<>();
		newPoints = new ArrayList<>();
	}
	
	
	FeatureUpdate(OpticalFlowResult opflowResult) {
		this();
		badPointsIndex = opflowResult.getBadPointsIndex();
		
		List<Point> currentFeaturesList = opflowResult.getCurrentFeatures().toList();
		for (int i = 0; i < currentFeaturesList.size(); i++) {
			Point point = currentFeaturesList.get(i);
			currentPoints.add(new PointDouble(point.x, point.y));
		}
		
		List<Point> newFeaturesList = opflowResult.getNewFeatures().toList();
		for (int i = 0; i < newFeaturesList.size(); i++) {
			Point point = newFeaturesList.get(i);
			newPoints.add(new PointDouble(point.x, point.y));
		}
	}
	
	
	public List<Integer> getBadPointsIndex() {
		return badPointsIndex;
	}
	
	
	public List<PointDouble> getCurrentPoints() {
		return currentPoints;
	}
	
	
	public List<PointDouble> getNewPoints() {
		return newPoints;
	}
	
	
	void setBadPointsIndex(List<Integer> badPointsIndex){
		this.badPointsIndex = badPointsIndex;
	}
	
	
	void setCurrentPoints(List<PointDouble> currentPoints){
		this.currentPoints = currentPoints;
	}
	
	
	void setNewPoints(List<PointDouble> newPoints){
		this.newPoints = newPoints;
	}
}
