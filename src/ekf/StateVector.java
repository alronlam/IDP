package ekf;

import java.util.ArrayList;

import Jama.Matrix;

public class StateVector {

	private ArrayList<Double> X;
	private int numFeatures;
	private int featureSize;
	private int stateVarsOfInterest;

	public StateVector(int stateVarsOfInterest, int featureSize) {
		this.stateVarsOfInterest = stateVarsOfInterest;

		this.featureSize = featureSize;
		this.numFeatures = 0;

		X = new ArrayList<Double>();
		for (int i = 0; i < stateVarsOfInterest; i++)
			X.add(Helper.EPS);
	}

	private StateVector() {
		// empty constructor used in clone()
	}

	/*** Feature manipulation Methods ***/
	public void addFeature(IDPFeature newFeature) {
		double x = newFeature.getX();
		double y = newFeature.getY();
		double z = newFeature.getZ();
		double azimuth = newFeature.getAzimuth();
		double elevation = newFeature.getElevation();
		double p = newFeature.getP();

		X.add(x);
		X.add(y);
		X.add(z);
		X.add(azimuth);
		X.add(elevation);
		X.add(p);

		numFeatures++;
	}

	public void deleteFeature(int featureIndex) {
		int targetIndexStart = this.getStartingIndexInStateVector(featureIndex);

		X.remove(targetIndexStart);
		X.remove(targetIndexStart);

		numFeatures--;
	}

	/*** Getters ***/

	public String toString() {
		return X.toString();
	}

	@SuppressWarnings("unchecked")
	public StateVector clone() {
		StateVector clone = new StateVector();
		clone.X = (ArrayList<Double>) X.clone();
		clone.numFeatures = numFeatures;
		clone.featureSize = featureSize;
		clone.stateVarsOfInterest = stateVarsOfInterest;

		return clone;
	}

	public int getStartingIndexInStateVector(int featureIndex) {
		return stateVarsOfInterest + featureSize * featureIndex;
	}

	// Just converts the current state vector to a Matrix object
	public Matrix toMatrix() {
		double[][] x = new double[X.size()][1];
		for (int i = 0; i < X.size(); i++)
			x[i][0] = X.get(i);

		return new Matrix(x);
	}

	public PointTriple getCurrentXYZPosition() {
		double x = X.get(0);
		double y = X.get(1);
		double z = X.get(2);

		return new PointTriple(x, y, z);
	}

	public Quaternion getCurrentQuaternion() {

		double r = X.get(3);
		double x = X.get(4);
		double y = X.get(5);
		double z = X.get(6);

		Quaternion quaternion = new Quaternion(x, y, z, r);
		return quaternion;
	}

	public PointTriple getCurrentV() {
		double x = X.get(7);
		double y = X.get(8);
		double z = X.get(9);

		return new PointTriple(x, y, z);
	}

	public PointTriple getCurrentOmega() {
		double x = X.get(10);
		double y = X.get(11);
		double z = X.get(12);

		return new PointTriple(x, y, z);
	}

	/*** Setters ***/
	public void setXYZPosition(PointTriple newXYZ) {
		X.set(0, newXYZ.getX());
		X.set(1, newXYZ.getY());
		X.set(2, newXYZ.getZ());
	}

	public void setQuaternion(Quaternion q) {
		X.set(3, q.getR());
		X.set(4, q.getX());
		X.set(5, q.getY());
		X.set(6, q.getZ());
	}

	public void setV(PointTriple newV) {
		X.set(7, newV.getX());
		X.set(8, newV.getY());
		X.set(9, newV.getZ());
	}

	public void setOmega(PointTriple newOmega) {
		X.set(10, newOmega.getX());
		X.set(11, newOmega.getY());
		X.set(12, newOmega.getZ());
	}

	public int getTotalStateSize() {
		return X.size();
	}

	public ArrayList<Double> getX() {
		return X;
	}

	public void setXBasedOnMatrix(Matrix m) {
		X.clear();
		double[][] x = m.getArray();
		for (int i = 0; i < x.length; i++)
			X.add(x[i][0]);
	}

	public int getNumFeatures() {
		return numFeatures;
	}

	public void setNumFeatures(int numFeatures) {
		this.numFeatures = numFeatures;
	}

	public int getFeatureSize() {
		return featureSize;
	}

	public void setFeatureSize(int featureSize) {
		this.featureSize = featureSize;
	}
}
