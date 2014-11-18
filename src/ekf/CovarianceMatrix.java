package ekf;

import java.util.ArrayList;

import Jama.Matrix;

public class CovarianceMatrix {
	private ArrayList<ArrayList<Double>> P; // Covariance Matrix
	private int numFeatures;
	private int featureSize;
	private int stateVarsOfInterest;

	public CovarianceMatrix(int stateVarsOfInterest, int featureSize, double pDiagonalInitial) {
		this.stateVarsOfInterest = stateVarsOfInterest;
		this.featureSize = featureSize;
		P = this.createInitialP(pDiagonalInitial);
	}

	private CovarianceMatrix() {
		// empty constructor for use only in the clone() method
	}

	// Initializes the covariance matrix
	private ArrayList<ArrayList<Double>> createInitialP(double pDiagonalInitial) {
		ArrayList<ArrayList<Double>> P = new ArrayList<ArrayList<Double>>();

		for (int i = 0; i < stateVarsOfInterest; i++) {
			ArrayList<Double> currRow = new ArrayList<Double>();
			for (int j = 0; j < stateVarsOfInterest; j++) {
				if (i != j)
					currRow.add(0.0);
				else
					currRow.add(pDiagonalInitial);
			}
			P.add(currRow);
		}

		return P;
	}

	/*** Feature manipulation Methods ***/
	public void addFeature(StateVector Xv, int ud, int vd, double std_rho, double std_pxl, Camera camera) {

		Matrix R_wc = Helper.quaternionToRotationMatrix(Xv.getCurrentQuaternion());

		// i'm not sure how to initialize this
		double fku = camera.f * camera.k1;
		double fkv = camera.f * camera.k2;
		Matrix undistortedUV = CameraHelper.undistort(ud, vd, camera);
		double uu = undistortedUV.get(0, 0);
		double vu = undistortedUV.get(1, 0);

		double X_c = (uu - camera.Cx) / fku;// -(U0-uu)/fku;
		double Y_c = (vu - camera.Cy) / fkv;// -(V0-vu)/fkv;
		double Z_c = 1;

		Matrix XYZ_c = new Matrix(3, 1);
		XYZ_c.set(0, 0, X_c);
		XYZ_c.set(1, 0, Y_c);
		XYZ_c.set(2, 0, Z_c);

		Matrix XYZ_w = R_wc.times(XYZ_c);
		double X_w = XYZ_w.get(0, 0);
		double Y_w = XYZ_w.get(1, 0);
		double Z_w = XYZ_w.get(2, 0);

		/* All the matrix pre-calculations */
		Matrix dtheta_dgw = DerivativeHelper.dtheta_dgw(X_w, Z_w); // 3x4
		Matrix dphi_dgw = DerivativeHelper.dphi_dgw(X_w, Y_w, Z_w);
		Matrix dgw_dqwr = DerivativeHelper.dRq_times_a_by_dq(Xv.getCurrentQuaternion(), XYZ_c);

		Matrix dtheta_dqwr = dtheta_dgw.times(dgw_dqwr);
		Matrix dphi_dqwr = dphi_dgw.times(dgw_dqwr);
		Matrix dy_dqwr = DerivativeHelper.dy_dqwr(dtheta_dqwr, dphi_dqwr);
		Matrix dy_drw = DerivativeHelper.dy_drw();
		Matrix dy_dxv = DerivativeHelper.dy_dxv(dy_drw, dy_dqwr);
		Matrix dyprima_dgw = DerivativeHelper.dyprima_dgw(dtheta_dgw, dphi_dgw);
		Matrix dgw_dgc = R_wc;
		Matrix dgc_dhu = DerivativeHelper.dgc_dhu(fku, fkv);
		Matrix dhu_dhd = CameraHelper.jacobianUndistort(camera, ud, vd);

		Matrix dyprima_dhd = dyprima_dgw.times(dgw_dgc).times(dgc_dhu).times(dhu_dhd);
		Matrix dy_dhd = DerivativeHelper.dy_dhd(dyprima_dhd);

		Matrix Padd = DerivativeHelper.Padd(std_rho, std_pxl);

		/* The actual covariance update */
		Matrix P_old = this.toMatrix();
		Matrix P_xv = Helper.extractSubMatrix(P_old, 0, this.stateVarsOfInterest - 1, 0, this.stateVarsOfInterest - 1);

		Matrix P_yxv = Helper.extractSubMatrix(P_old, this.stateVarsOfInterest, P_old.getRowDimension() - 1, 0,
				this.stateVarsOfInterest - 1);

		Matrix P_y = Helper.extractSubMatrix(P_old, this.stateVarsOfInterest, P_old.getRowDimension() - 1,
				this.stateVarsOfInterest, P_old.getColumnDimension() - 1);

		Matrix P_xvy = Helper.extractSubMatrix(P_old, 0, this.stateVarsOfInterest - 1, this.stateVarsOfInterest,
				P_old.getColumnDimension() - 1);

		Matrix P_new = new Matrix(P_old.getRowDimension() + featureSize, P_old.getColumnDimension() + featureSize);

		// Copy the old values
		P_new = Helper.setSubMatrixValues(P_new, P_xv, 0, 0);
		P_new = Helper.setSubMatrixValues(P_new, P_xvy, 0, this.stateVarsOfInterest);
		P_new = Helper.setSubMatrixValues(P_new, P_yxv, this.stateVarsOfInterest, 0);
		P_new = Helper.setSubMatrixValues(P_new, P_y, this.stateVarsOfInterest, this.stateVarsOfInterest);

		// Bottom row

		P_new = Helper.setSubMatrixValues(P_new, dy_dxv.times(P_xv), P_old.getRowDimension(), 0);

		if (numFeatures > 0) // only do this if there were previous features
			P_new = Helper.setSubMatrixValues(P_new, dy_dxv.times(P_xvy), P_old.getRowDimension(),
					this.stateVarsOfInterest);

		Matrix botRight = dy_dxv.times(P_xv).times(dy_dxv.transpose())
				.plus(dy_dhd.times(Padd).times(dy_dhd.transpose()));
		P_new = Helper.setSubMatrixValues(P_new, botRight, P_old.getRowDimension(), P_old.getColumnDimension());

		// Rightmost column
		P_new = Helper.setSubMatrixValues(P_new, P_xv.times(dy_dxv.transpose()), 0, P_old.getColumnDimension());
		if (numFeatures > 0) // only do this if there were previous features
			P_new = Helper.setSubMatrixValues(P_new, P_yxv.times(dy_dxv.transpose()), this.stateVarsOfInterest,
					P_old.getColumnDimension());

		this.set(P_new);

		numFeatures++;
	}

	public void deleteFeature(int featureIndex) {

		int targetIndexStart = this.getStartingIndexInStateVector(featureIndex);
		P.remove(targetIndexStart);
		P.remove(targetIndexStart);

		for (ArrayList<Double> row : P) {
			row.remove(targetIndexStart);
			row.remove(targetIndexStart);
		}

		numFeatures--;
	}

	private int getStartingIndexInStateVector(int featureIndex) {
		return stateVarsOfInterest + featureSize * featureIndex;
	}

	/********** Getters **********/

	public String toString() {
		return Helper.toStringArrayList(P);
	}

	@SuppressWarnings("unchecked")
	public CovarianceMatrix clone() {
		CovarianceMatrix clone = new CovarianceMatrix();
		clone.P = (ArrayList<ArrayList<Double>>) P.clone();
		clone.numFeatures = numFeatures;
		clone.featureSize = featureSize;
		clone.stateVarsOfInterest = stateVarsOfInterest;
		return clone;
	}

	public int getSize() {
		return P.size();
	}

	public Matrix toMatrix() {
		Matrix m = new Matrix(P.size(), P.size());

		for (int i = 0; i < P.size(); i++)
			for (int j = 0; j < P.size(); j++)
				m.set(i, j, P.get(i).get(j));

		return m;
	}

	public Matrix extractPri(int index) {
		int startIndex = stateVarsOfInterest + index * featureSize;

		return this.extractSubMatrix(0, stateVarsOfInterest, startIndex, startIndex + featureSize - 1);
	}

	public Matrix extractPxx() {
		return this.extractSubMatrix(0, stateVarsOfInterest - 1, 0, stateVarsOfInterest - 1);
	}

	private Matrix extractSubMatrix(int startRow, int endRow, int startCol, int endCol) {
		double[][] sub = new double[endRow - startRow + 1][endCol - startCol + 1];
		for (int i = startRow; i <= endRow; i++)
			for (int j = startCol; j <= endCol; j++)
				sub[i - startRow][j - startCol] = P.get(i).get(j);
		return new Matrix(sub);
	}

	/********** Setters **********/

	public void updatePri(Matrix A_Matrix) {
		for (int i = 0; i < numFeatures; i++) {
			Matrix PriMatrix = this.extractPri(i);
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
					P.get(targetStartRowIndex + j).set(targetStartColIndex + k, PriMatrix.get(j, k));
		}
	}

	public void set(Matrix pMatrix) {

		ArrayList<ArrayList<Double>> P_new = new ArrayList<ArrayList<Double>>();

		for (int i = 0; i < pMatrix.getRowDimension(); i++) {
			ArrayList<Double> row = new ArrayList<Double>();
			for (int j = 0; j < pMatrix.getColumnDimension(); j++) {
				row.add(pMatrix.get(i, j));
			}
			P_new.add(row);
		}

		this.P = P_new;
	}

	public void setVal(int row, int col, double val) {
		P.get(row).set(col, val);
	}

	public void setPxx(Matrix matrix) {
		for (int i = 0; i < matrix.getRowDimension(); i++) {
			ArrayList<Double> row = P.get(i);
			for (int j = 0; j < matrix.getColumnDimension(); j++) {
				row.set(j, matrix.get(i, j));
			}
		}
	}
}
