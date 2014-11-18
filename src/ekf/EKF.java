package ekf;

import java.util.ArrayList;

import Jama.Matrix;

public class EKF {

	private ArrayList<Double> X; // State Vector
	private ArrayList<ArrayList<Double>> P; // Covariance Matrix
	private int numFeatures;

	/*
	 * These are jacobians of the prediction model used when adding a new
	 * feature in the covariance matrix. They are updated every INS update
	 * because they are based on displacement and heading.
	 */
	private Matrix jrMatrix;
	private Matrix jzMatrix;

	/* Constants */
	public static final int FEATURE_SIZE = 3;
	public static final int STATE_VARS_OF_INTEREST = 13;

	public static final double P_DIAGONAL_INITIAL = 0;

	// used for linear velocity sd. in m/s^2
	public static final double SD_A_component_filter = 4;
	// used for angular velocity sd. in rad/s^2
	public static final double SD_alpha_component_filter = 6;

	public static final double VRV_DISTANCE_VARIANCE = 0.1;
	public static final double VRV_HEADING_NOISE = 0.1;

	public EKF() {
		X = createInitialX();
		P = createInitialP();

		jrMatrix = this.createJRMatrix(0, 0);
		jzMatrix = this.createJZMatrix(0, 0, 0);
	}

	/********** Getters **********/
	public ArrayList<ArrayList<Double>> getP() {
		return (ArrayList<ArrayList<Double>>) P.clone();
	}

	public ArrayList<Double> getX() {
		return (ArrayList<Double>) X.clone();
	}

	public PointDouble getDeviceCoords() {
		PointDouble point = new PointDouble(X.get(0), X.get(1));
		return point;
	}

	private PointDouble getFeatureCoordsFromStateVector(int featureIndex) {

		int stateVectorIndexOfFeature = 3 + featureIndex * 2;
		double targetFeatureX = X.get(stateVectorIndexOfFeature);
		double targetFeatureY = X.get(stateVectorIndexOfFeature + 1);

		PointDouble point = new PointDouble(targetFeatureX, targetFeatureY);

		return point;
	}

	public String getParameterConfigurations() {

		StringBuilder sb = new StringBuilder();
		sb.append("P_DIAG_INIT=" + P_DIAGONAL_INITIAL + "\r\n");
		sb.append(" SD_A_component_filter = " + SD_A_component_filter);
		sb.append(" SD_alpha_component_filter = " + SD_alpha_component_filter);

		return sb.toString();
	}

	public PointTriple getCurrentXYZPosition() {
		double x = X.get(0);
		double y = X.get(1);
		double z = X.get(2);

		return new PointTriple(x, y, z);
	}

	public Quaternion getCurrentQuaternion() {
		double x = X.get(3);
		double y = X.get(4);
		double z = X.get(5);
		double r = X.get(6);

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

	public int getTotalStateSize() {
		return STATE_VARS_OF_INTEREST + numFeatures * FEATURE_SIZE;
	}

	public int getStartingIndexInStateVector(int featureIndex) {
		return STATE_VARS_OF_INTEREST + FEATURE_SIZE * featureIndex;
	}

	public double getHeadingRadians() {
		return 0; // method stub
	}

	/********** Setters **********/

	private void setXYZPosition(PointTriple newXYZ) {
		X.set(0, newXYZ.getX());
		X.set(1, newXYZ.getY());
		X.set(2, newXYZ.getZ());
	}

	private void setQuaternion(Quaternion q) {
		X.set(3, q.getX());
		X.set(4, q.getY());
		X.set(5, q.getZ());
		X.set(6, q.getR());
	}

	private void setV(PointTriple newV) {
		X.set(7, newV.getX());
		X.set(8, newV.getY());
		X.set(9, newV.getZ());
	}

	private void setOmega(PointTriple newOmega) {
		X.set(10, newOmega.getX());
		X.set(11, newOmega.getY());
		X.set(12, newOmega.getZ());
	}

	private void setPxx(Matrix matrix) {
		for (int i = 0; i < matrix.getRowDimension(); i++) {
			ArrayList<Double> row = P.get(i);
			for (int j = 0; j < matrix.getColumnDimension(); j++) {
				row.set(j, matrix.get(i, j));
			}
		}
	}

	/********** INS Update **********/

	public void predict(PointTriple linearImpulse, PointTriple angularImpulse, double deltaTime) {
		PointTriple xyzPositionOld = this.getCurrentXYZPosition();
		Quaternion quaternionOld = this.getCurrentQuaternion();
		PointTriple vOld = this.getCurrentV();
		PointTriple omegaOld = this.getCurrentOmega();

		// Not sure if correct, but this is what's written in MonoSLAM code
		PointTriple xyzPositionNew = xyzPositionOld.plus(vOld.times(deltaTime));
		Quaternion qwt = QuaternionHelper.calculateQWT(omegaOld, deltaTime);
		Quaternion quaternionNew = quaternionOld.times(qwt);
		PointTriple vNew = vOld.plus(linearImpulse);
		PointTriple omegaNew = omegaOld.plus(angularImpulse);

		/* Update the state vector according to predictions */
		this.setXYZPosition(xyzPositionNew);
		this.setQuaternion(quaternionNew);
		this.setV(vNew);
		this.setOmega(omegaNew);

		/* Update the covariance matrices based on this prediction */
		Matrix A_Matrix = this.createA(vOld, omegaOld, deltaTime);
		Matrix Q_Matrix = this.createQ(deltaTime, quaternionOld, omegaOld);

		Matrix Pxx_Matrix = this.extractPxx();
		Pxx_Matrix = A_Matrix.times(Pxx_Matrix).times(A_Matrix.transpose()).plus(Q_Matrix);
		this.setPxx(Pxx_Matrix);

		// Update the first 3 columns of P (device to feature correlation) P_ri
		// = A * P_ri

		for (int i = 0; i < numFeatures; i++) {
			Matrix PriMatrix = extractPri(i);
			PriMatrix = A_Matrix.times(PriMatrix);

			int targetStartRowIndex = 0;
			int targetStartColIndex = this.getStartingIndexInStateVector(i);

			for (int j = 0; j < PriMatrix.getRowDimension(); j++)
				for (int k = 0; k < PriMatrix.getColumnDimension(); k++)
					P.get(targetStartRowIndex + j).set(targetStartColIndex + k, PriMatrix.get(j, k));

			// Also update the transpose
			Matrix PriMatrixTranspose = PriMatrix.transpose();

			// swap row and col
			int temp = targetStartRowIndex;
			targetStartRowIndex = targetStartColIndex;
			targetStartColIndex = temp;

			for (int j = 0; j < PriMatrixTranspose.getRowDimension(); j++)
				for (int k = 0; k < PriMatrixTranspose.getColumnDimension(); k++)
					P.get(targetStartRowIndex + j).set(targetStartColIndex + k, PriMatrixTranspose.get(j, k));
		}
	}

	private Matrix extractPri(int index) {
		int startIndex = STATE_VARS_OF_INTEREST + index * FEATURE_SIZE;

		return this.extractSubMatrix(0, STATE_VARS_OF_INTEREST, startIndex, startIndex + FEATURE_SIZE - 1);
	}

	private Matrix extractPxx() {
		return this.extractSubMatrix(0, STATE_VARS_OF_INTEREST - 1, 0, STATE_VARS_OF_INTEREST - 1);
	}

	private Matrix extractSubMatrix(int startRow, int endRow, int startCol, int endCol) {
		double[][] sub = new double[endRow - startRow + 1][endCol - startCol + 1];
		for (int i = startRow; i <= endRow; i++)
			for (int j = startCol; j <= endCol; j++)
				sub[i - startRow][j - startCol] = P.get(i).get(j);
		return new Matrix(sub);
	}

	/********** V-INS Update **********/

	// Method for correcting the state vector based on re-observed features.
	public void updateFromReobservedFeatureThroughDistanceHeading(int featureIndex, double observedDistance,
			double observedHeading) {

		PointDouble deviceCoords = this.getDeviceCoords();
		PointDouble featureCoords = this.getFeatureCoordsFromStateVector(featureIndex);

		/* Predict the distance and heading to the specified feature */
		double predictedDistance = deviceCoords.computeDistanceTo(featureCoords);
		double predictedHeadingDelta = deviceCoords.computeRadiansTo(featureCoords) - this.getHeadingRadians();

		/* Calculate the Kalman Gain */
		Matrix hMatrix = this.createH(predictedDistance, featureIndex, featureCoords, deviceCoords);
		Matrix pMatrix = this.extractSubMatrix(0, P.size() - 1, 0, P.size() - 1);
		Matrix hphMatrix = hMatrix.times(pMatrix).times(hMatrix.transpose());
		Matrix vrvMatrix = this.createVRVMatrix(observedDistance);
		Matrix innovationMatrix = hphMatrix.plus(vrvMatrix);
		Matrix kalmanGainMatrix = pMatrix.times(hMatrix.transpose()).times(innovationMatrix.inverse());

		// Still need to add measurement noise to these two variables
		double[][] differenceVector = new double[2][1];
		differenceVector[0][0] = observedDistance - predictedDistance;
		differenceVector[1][0] = observedHeading - predictedHeadingDelta;
		Matrix zMinusHMatrix = new Matrix(differenceVector);

		/* Adjust state vector based on prediction */
		Matrix xMatrix = createStateVectorMatrix();
		xMatrix = xMatrix.plus(kalmanGainMatrix.times(zMinusHMatrix));

		// Log.d("EKFTests", "observedDistance= " + observedDistance +
		// ", predictedDistance " + predictedDistance
		// + " observedHeading=" + observedHeading + " predictedHeadingDelta=" +
		// predictedHeadingDelta);

		// re-populate the state vector based on the result
		X.clear();
		double[][] x = xMatrix.getArray();
		for (int i = 0; i < x.length; i++)
			X.add(x[i][0]);

		// Update covariance
		pMatrix = pMatrix.minus(kalmanGainMatrix.times(innovationMatrix).times(kalmanGainMatrix.transpose()));
		updateCovariance(pMatrix);

		// Log.d("EKFTests", "State Vector: " + X.toString());
	}

	private void updateCovariance(Matrix pMatrix) {
		for (int i = 0; i < P.size(); i++) {
			for (int j = 0; j < P.get(i).size(); j++) {
				P.get(i).set(j, pMatrix.get(i, j));
			}
		}
	}

	public void updateFromReobservedFeatureCoords(int featureIndex, double fX, double fY) {

		PointDouble deviceCoords = this.getDeviceCoords();
		PointDouble observedFeatureCoords = new PointDouble(fX, fY);

		/* Calculate the observed distance and heading */
		double observedDistance = deviceCoords.computeDistanceTo(observedFeatureCoords);
		double observedHeading = deviceCoords.computeRadiansTo(observedFeatureCoords) - this.getHeadingRadians();

		this.updateFromReobservedFeatureThroughDistanceHeading(featureIndex, observedDistance, observedHeading);
	}

	// Method for deleting a feature. Includes removing the feature from the
	// state vector and covariance matrix.
	public void deleteFeature(int featureIndex) {
		int targetIndexStart = 3 + featureIndex * 2;

		X.remove(targetIndexStart);
		X.remove(targetIndexStart);

		P.remove(targetIndexStart);
		P.remove(targetIndexStart);

		for (ArrayList<Double> row : P) {
			row.remove(targetIndexStart);
			row.remove(targetIndexStart);
		}

		numFeatures--;
	}

	// Method for adding a feature to the sate vector and covariance matrix.
	public void addFeature(double x, double y) {

		// add to state vector
		X.add(x);
		X.add(y);

		// add to covariance matrix
		// add 2 rows, then add two columns at the end

		Matrix pPhiMatrix = this.extractPxx();

		ArrayList<Matrix> toAdd = new ArrayList<Matrix>();

		// P^phi * Jxr^T
		Matrix lowerLeftMatrix = pPhiMatrix.times(jrMatrix.transpose()).transpose();

		toAdd.add(lowerLeftMatrix);

		// numFeatures still holds the number of features not counting this new
		// feature to be added
		for (int i = 0; i < numFeatures; i++) {
			// extract the sub-matrix above the new matrix's location
			Matrix subMatrix = this.extractSubMatrix(0, 2, 3 + i * 2, 4 + i * 2);
			Matrix currMatrix = jrMatrix.times(subMatrix);
			toAdd.add(currMatrix);
		}

		// Create vrvMatrix
		PointDouble deviceCoords = this.getDeviceCoords();
		PointDouble featureCoords = new PointDouble(x, y);
		double distance = deviceCoords.computeDistanceTo(featureCoords);
		Matrix vrvMatrix = createVRVMatrix(distance);

		Matrix lowerRightMatrix = jrMatrix.times(pPhiMatrix).times(jrMatrix.transpose())
				.plus(jzMatrix.times(vrvMatrix).times(jzMatrix.transpose()));
		toAdd.add(lowerRightMatrix);

		// This part adds the last 2 rows
		for (int i = 0; i < 2; i++) {
			ArrayList<Double> currRow = new ArrayList<Double>();

			for (Matrix matrix : toAdd) {
				for (int j = 0; j < matrix.getColumnDimension(); j++)
					currRow.add(matrix.get(i, j));
			}

			P.add(currRow);
		}

		// This part adds the new 2 columns
		// Do not include the last entry in toAdd (the lower right matrix)
		// because you're not going to transpose it!
		for (int i = 0, row = 0; i < toAdd.size() - 1; i++) {
			Matrix transpose = toAdd.get(i).transpose();

			for (int j = 0; j < transpose.getRowDimension(); j++) {
				for (int k = 0; k < transpose.getColumnDimension(); k++) {
					P.get(row).add(transpose.get(j, k));
				}
				row++;
			}
		}

		numFeatures++;
	}

	/********** Methods for Creating Matrices **********/

	private Matrix createH(double predictedDistance, int featureIndex, PointDouble featureCoords,
			PointDouble deviceCoords) {
		// Set-up H for the specified feature
		double[][] H = new double[2][3 + numFeatures * 2];

		double r = predictedDistance;
		double A = (deviceCoords.getX() - featureCoords.getX()) / r;
		double B = (deviceCoords.getY() - featureCoords.getY()) / r;
		double C = 0;
		double D = (featureCoords.getY() - deviceCoords.getY()) / (r * r);
		double E = (featureCoords.getX() - deviceCoords.getX()) / (r * r);
		double F = -1;

		H[0][0] = A;
		H[0][1] = B;
		H[0][2] = C;
		H[1][0] = D;
		H[1][1] = E;
		H[1][2] = F;

		int targetFeatureIndex = 3 + 2 * featureIndex;

		H[0][targetFeatureIndex] = -1 * A;
		H[0][targetFeatureIndex + 1] = -1 * B;
		H[1][targetFeatureIndex] = -1 * D;
		H[1][targetFeatureIndex + 1] = -1 * E;

		return new Matrix(H);
	}

	// Initializes the state vector
	private ArrayList<Double> createInitialX() {
		ArrayList<Double> X = new ArrayList<Double>();
		for (int i = 0; i < STATE_VARS_OF_INTEREST; i++) {
			X.add(0.0);
		}
		return X;
	}

	// Initializes the covariance matrix
	private ArrayList<ArrayList<Double>> createInitialP() {
		ArrayList<ArrayList<Double>> P = new ArrayList<ArrayList<Double>>();

		for (int i = 0; i < STATE_VARS_OF_INTEREST; i++) {
			ArrayList<Double> currRow = new ArrayList<Double>();
			for (int j = 0; j < STATE_VARS_OF_INTEREST; j++) {
				if (i != j)
					currRow.add(0.0);
				else
					currRow.add(P_DIAGONAL_INITIAL);
			}
			P.add(currRow);
		}

		return P;
	}

	/** Creates the Jacobian for the Process Model **/
	public Matrix createA(PointTriple linearV, PointTriple angularV, double deltaTime) {
		Matrix A_Matrix = new Matrix(STATE_VARS_OF_INTEREST, STATE_VARS_OF_INTEREST);

		/* Initilize all the Identity sub-matrices */
		Matrix _3x3Identity = Helper.createSameValuedMatrix(1, 3, 3);
		A_Matrix = Helper.setSubMatrixValues(A_Matrix, _3x3Identity, 0, 0);
		A_Matrix = Helper.setSubMatrixValues(A_Matrix, _3x3Identity, 7, 7);
		A_Matrix = Helper.setSubMatrixValues(A_Matrix, _3x3Identity, 10, 10);

		/* Initialize 3x3 deltaTime */
		Matrix deltaTimeMatrix = Helper.createSameValuedMatrix(deltaTime, 3, 3);
		A_Matrix = Helper.setSubMatrixValues(A_Matrix, deltaTimeMatrix, 0, 7);

		/* Initialize 4x4 dqnew_by_domega */
		Quaternion qwt = QuaternionHelper.calculateQWT(this.getCurrentOmega(), deltaTime);
		Matrix dqnew_by_dq = QuaternionHelper.dq3_by_dq2(qwt);
		A_Matrix = Helper.setSubMatrixValues(A_Matrix, dqnew_by_dq, 3, 3);

		/* Initialize 4x3 dqnew_by_domega = d(q x qwt)_by_dqwt . dqwt_by_domega */
		PointTriple omegaOld = this.getCurrentOmega();
		Matrix dqnew_by_domega = QuaternionHelper.dq3_by_dq1(this.getCurrentQuaternion()).times(
				QuaternionHelper.dqomegadt_by_domega(omegaOld, deltaTime));
		A_Matrix = Helper.setSubMatrixValues(A_Matrix, dqnew_by_domega, 3, 10);

		return A_Matrix;
	}

	public Matrix createQ(double deltaTime, Quaternion qold, PointTriple omegaOld) {
		Matrix Pnn_Matrix = this.createPnn(deltaTime);
		Matrix dxnew_by_dn = this.create_dxnew_by_dn(deltaTime, qold, omegaOld);

		Matrix Q_Matrix = dxnew_by_dn.times(Pnn_Matrix).times(dxnew_by_dn.transpose());

		return Q_Matrix;
	}

	public Matrix create_dxnew_by_dn(double deltaTime, Quaternion qOld, PointTriple omegaOld) {
		Matrix deltaTimeMatrix3x3 = Helper.createSameValuedMatrix(deltaTime, 3, 3);
		Matrix identity3x3 = Helper.createSameValuedMatrix(1, 3, 3);
		Matrix dqnew_by_domega = QuaternionHelper.dq3_by_dq1(qOld).times(
				QuaternionHelper.dqomegadt_by_domega(omegaOld, deltaTime));

		Matrix dxnew_by_dn_matrix = new Matrix(13, 6);
		dxnew_by_dn_matrix = Helper.setSubMatrixValues(dxnew_by_dn_matrix, deltaTimeMatrix3x3, 0, 0);
		dxnew_by_dn_matrix = Helper.setSubMatrixValues(dxnew_by_dn_matrix, identity3x3, 7, 0);
		dxnew_by_dn_matrix = Helper.setSubMatrixValues(dxnew_by_dn_matrix, identity3x3, 10, 3);
		dxnew_by_dn_matrix = Helper.setSubMatrixValues(dxnew_by_dn_matrix, dqnew_by_domega, 3, 3);

		return dxnew_by_dn_matrix;
	}

	public Matrix createPnn(double deltaTime) {
		double LINEAR_VELOCITY_NOISE_VARIANCE = SD_A_component_filter * SD_A_component_filter * deltaTime * deltaTime;
		double ANGULAR_VELOCITY_NOISE_VARIANCE = SD_alpha_component_filter * SD_alpha_component_filter * deltaTime
				* deltaTime;

		double[][] pnnArr = new double[6][6];

		for (int i = 0; i < 3; i++)
			pnnArr[i][i] = LINEAR_VELOCITY_NOISE_VARIANCE;

		for (int i = 3; i < 6; i++)
			pnnArr[i][i] = ANGULAR_VELOCITY_NOISE_VARIANCE;

		return new Matrix(pnnArr);
	}

	// The measurement noise matrix
	private Matrix createVRVMatrix(double distance) {
		double[][] vrv = new double[2][2];
		vrv[0][0] = distance * VRV_DISTANCE_VARIANCE;
		vrv[1][1] = VRV_HEADING_NOISE;// Math.toRadians(2);
		return new Matrix(vrv);
	}

	// Just converts the current state vector to a Matrix object
	private Matrix createStateVectorMatrix() {
		double[][] x = new double[X.size()][1];
		for (int i = 0; i < X.size(); i++)
			x[i][0] = X.get(i);

		return new Matrix(x);
	}

	// Creates some Jacobian matrix used when adding a new feature to the
	// covariance matrix
	private Matrix createJRMatrix(double displacementX, double displacementY) {
		double[][] jr = { { 1, 0, -1 * displacementY }, { 0, 1, displacementX } };
		return new Matrix(jr);
	}

	// Creates some Jacobian matrix used when adding a new feature to the
	// covariance matrix
	private Matrix createJZMatrix(double displacementX, double displacementY, double headingRadians) {
		double[][] jz = { { Math.cos(headingRadians), -1 * displacementY }, { Math.sin(headingRadians), displacementX } };
		return new Matrix(jz);
	}
}
