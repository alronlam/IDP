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
	public void addFeature(StateVector Xv, double u, double v, double std_rho, double std_pxl) {

		// temporary while there are no camera parameters here
		double fku = 1;
		double fkv = 1;

		Matrix R_wc = Helper.quaternionToRotationMatrix(Xv.getCurrentQuaternion());

		// undistorted. temporarily just the same as u,v
		double uu = u;
		double vu = u;

		double X_c = u;// -(U0-uu)/fku;
		double Y_c = u;// -(V0-vu)/fkv;
		double Z_c = u;// 1;

		Matrix XYZ_c = new Matrix(3, 1);
		XYZ_c.set(0, 0, X_c);
		XYZ_c.set(1, 0, Y_c);
		XYZ_c.set(2, 0, Z_c);

		Matrix XYZ_w = R_wc.times(XYZ_c);
		double X_w = XYZ_w.get(0, 0);
		double Y_w = XYZ_w.get(1, 0);
		double Z_w = XYZ_w.get(2, 0);

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
		for (int i = 0; i < P.size(); i++) {
			for (int j = 0; j < P.get(i).size(); j++) {
				P.get(i).set(j, pMatrix.get(i, j));
			}
		}
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
