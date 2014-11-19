package ekf;

import Jama.Matrix;

public class EKF {

	private StateVector Xm1;
	private StateVector X;
	private CovarianceMatrix Pm1;
	private CovarianceMatrix P;

	/* Constants */
	public static final int FEATURE_SIZE = 6;
	public static final int STATE_VARS_OF_INTEREST = 13;

	public static final double P_DIAGONAL_INITIAL = 0;

	// used for linear velocity sd. in m/s^2
	// in matlab code this is 0.007
	public static final double SD_A_component_filter = 4;
	// used for angular velocity sd. in rad/s^2
	// in matlab codethis is 0.007
	public static final double SD_alpha_component_filter = 6;
	public static final double SD_IMAGE_NOISE = 1;

	public static final double INITIAL_RHO = 0.1;
	public static final double STDDEV_RHO = 0.5;
	public static final double STDDEV_PXL = 1;

	public static final double VRV_DISTANCE_VARIANCE = 0.1;
	public static final double VRV_HEADING_NOISE = 0.1;

	public EKF() {
		X = createInitialX();
		P = createInitialP();
	}

	/********** Getters **********/

	public PointDouble getDeviceCoords() {
		PointDouble point = new PointDouble(X.getCurrentXYZPosition().getX(), X.getCurrentXYZPosition().getZ());
		return point;
	}

	public String getParameterConfigurations() {

		StringBuilder sb = new StringBuilder();
		sb.append("P_DIAG_INIT=" + P_DIAGONAL_INITIAL + "\r\n");
		sb.append(" SD_A_component_filter = " + SD_A_component_filter);
		sb.append(" SD_alpha_component_filter = " + SD_alpha_component_filter);

		return sb.toString();
	}

	public StateVector getStateVector() {
		return X;
	}

	public CovarianceMatrix getCovarianceMatrix() {
		return P;
	}

	public double getHeadingRadians() {
		return 0; // method stub
	}

	/********** INS Update **********/

	public void predict(PointTriple linearImpulse, PointTriple angularImpulse, double deltaTime) {

		// Keep a record of the old state vector and covariance matrix at this
		// time step
		Xm1 = X.clone();
		Pm1 = P.clone();

		PointTriple xyzPositionOld = X.getCurrentXYZPosition();
		Quaternion quaternionOld = X.getCurrentQuaternion();
		PointTriple vOld = X.getCurrentV();
		PointTriple omegaOld = X.getCurrentOmega();

		// Not sure if correct, but this is what's written in MonoSLAM code
		PointTriple xyzPositionNew = xyzPositionOld.plus(vOld.times(deltaTime));
		Quaternion qwt = QuaternionHelper.calculateQWT(omegaOld, deltaTime);
		Quaternion quaternionNew = quaternionOld.times(qwt);
		PointTriple vNew = vOld.plus(linearImpulse);
		PointTriple omegaNew = omegaOld.plus(angularImpulse);

		/* Update the state vector according to predictions */
		X.setXYZPosition(xyzPositionNew);
		X.setQuaternion(quaternionNew);
		X.setV(vNew);
		X.setOmega(omegaNew);

		/* Update the covariance matrices based on this prediction */
		Matrix A_Matrix = this.createA(vOld, omegaOld, deltaTime);
		Matrix Q_Matrix = this.createQ(deltaTime, quaternionOld, omegaOld);

		Matrix Pxx_Matrix = P.extractPxx();
		Pxx_Matrix = A_Matrix.times(Pxx_Matrix).times(A_Matrix.transpose()).plus(Q_Matrix);
		P.setPxx(Pxx_Matrix);

		// Update the first 3 columns of P (device to feature correlation) P_ri
		// = A * P_ri
		P.updatePri(A_Matrix);
	}

	/********** V-INS Update **********/

	// Method for correcting the state vector based on re-observed features.
	public void updateFromReobservedFeatureThroughDistanceHeading(int featureIndex, double observedDistance,
			double observedHeading) {

		PointDouble deviceCoords = this.getDeviceCoords();
		// Changed the following line to remove the compile error brought about
		// by moving state vector to its own class. Duke is going to change this
		// anyway.
		PointDouble featureCoords = null;

		/* Predict the distance and heading to the specified feature */
		double predictedDistance = deviceCoords.computeDistanceTo(featureCoords);
		double predictedHeadingDelta = deviceCoords.computeRadiansTo(featureCoords) - this.getHeadingRadians();

		/* Calculate the Kalman Gain */
		Matrix hMatrix = this.createH(predictedDistance, featureIndex, featureCoords, deviceCoords);
		Matrix pMatrix = P.toMatrix();
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
		Matrix xMatrix = X.toMatrix();
		xMatrix = xMatrix.plus(kalmanGainMatrix.times(zMinusHMatrix));

		// re-populate the state vector based on the result
		X.setXBasedOnMatrix(xMatrix);

		// Update covariance
		pMatrix = pMatrix.minus(kalmanGainMatrix.times(innovationMatrix).times(kalmanGainMatrix.transpose()));
		P.set(pMatrix);

		// Log.d("EKFTests", "State Vector: " + X.toString());
	}

	public void updateFromReobservedFeatureCoords(int featureIndex, double fX, double fY) {

		PointDouble deviceCoords = this.getDeviceCoords();
		PointDouble observedFeatureCoords = new PointDouble(fX, fY);

		/* Calculate the observed distance and heading */
		double observedDistance = deviceCoords.computeDistanceTo(observedFeatureCoords);
		double observedHeading = deviceCoords.computeRadiansTo(observedFeatureCoords) - this.getHeadingRadians();

		this.updateFromReobservedFeatureThroughDistanceHeading(featureIndex, observedDistance, observedHeading);
	}

	// Method for deleting a feature.
	// Includes removing the feature from the state vector and covariance
	// matrix.
	public void deleteFeature(int featureIndex) {
		X.deleteFeature(featureIndex);
		P.deleteFeature(featureIndex);
	}

	// Method for adding a feature to the sate vector and covariance matrix.
	public void addFeature(int ud, int vd, Camera camera) {
		IDPFeature newFeature = FeatureInitializationHelper.createFeature(X.getCurrentXYZPosition(),
				X.getCurrentQuaternion(), ud, vd, INITIAL_RHO);

		X.addFeature(newFeature);
		P.addFeature(X, ud, vd, STDDEV_PXL, STDDEV_RHO, camera);
	}

	/********** Methods for Creating Matrices **********/

	private Matrix createH(double predictedDistance, int featureIndex, PointDouble featureCoords,
			PointDouble deviceCoords) {
		// Set-up H for the specified feature
		double[][] H = new double[2][3 + X.getNumFeatures() * 2];

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
	private StateVector createInitialX() {
		return new StateVector(STATE_VARS_OF_INTEREST, FEATURE_SIZE);
	}

	private CovarianceMatrix createInitialP() {
		return new CovarianceMatrix(STATE_VARS_OF_INTEREST, FEATURE_SIZE, P_DIAGONAL_INITIAL);
	}

	/** Creates the Jacobian for the Process Model **/
	public Matrix createA(PointTriple linearV, PointTriple angularV, double deltaTime) {
		Matrix A_Matrix = new Matrix(STATE_VARS_OF_INTEREST, STATE_VARS_OF_INTEREST);

		/* Initilize all the Identity sub-matrices */
		Matrix _3x3Identity = Helper.createIdentityMatrix(3);
		A_Matrix = Helper.setSubMatrixValues(A_Matrix, _3x3Identity, 0, 0);
		A_Matrix = Helper.setSubMatrixValues(A_Matrix, _3x3Identity, 7, 7);
		A_Matrix = Helper.setSubMatrixValues(A_Matrix, _3x3Identity, 10, 10);

		/* Initialize 3x3 deltaTime */
		Matrix deltaTimeMatrix = Helper.createSameValuedMatrix(deltaTime, 3, 3);
		A_Matrix = Helper.setSubMatrixValues(A_Matrix, deltaTimeMatrix, 0, 7);

		/* Initialize 4x4 dqnew_by_domega */
		Quaternion qwt = QuaternionHelper.calculateQWT(X.getCurrentOmega(), deltaTime);

		Matrix dqnew_by_dq = QuaternionHelper.dq3_by_dq2(qwt);
		A_Matrix = Helper.setSubMatrixValues(A_Matrix, dqnew_by_dq, 3, 3);

		/* Initialize 4x3 dqnew_by_domega = d(q x qwt)_by_dqwt . dqwt_by_domega */
		PointTriple omegaOld = X.getCurrentOmega();
		Matrix dqnew_by_domega = QuaternionHelper.dq3_by_dq1(X.getCurrentQuaternion()).times(
				QuaternionHelper.dqomegadt_by_domega(omegaOld, deltaTime));

		System.out.println("dqnew_by_domega");
		QuaternionHelper.dq3_by_dq1(X.getCurrentQuaternion()).print(0, 0);
		System.out.println();
		QuaternionHelper.dqomegadt_by_domega(omegaOld, deltaTime).print(0, 0);

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
		Matrix identity3x3 = Helper.createIdentityMatrix(3);
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

}
