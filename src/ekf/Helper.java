package ekf;

import java.util.ArrayList;

import Jama.Matrix;

public class Helper {

	/*** Sets the list to contain the values in the sub-matrix specified ***/
	public static void setArrayListValues(ArrayList<ArrayList<Double>> list, Matrix matrix, int startRow, int startCol) {
		for (int i = startRow; i < matrix.getRowDimension(); i++)
			for (int j = startCol; j < matrix.getColumnDimension(); j++)
				list.get(i).set(j, matrix.get(i, j));
	}

	public static Matrix convertToMatrix(ArrayList<ArrayList<Double>> list) {

		int rows = list.size();
		int cols = list.get(0).size();
		Matrix matrix = new Matrix(rows, cols);

		for (int i = 0; i < list.size(); i++) {
			ArrayList<Double> row = list.get(i);
			for (int j = 0; j < row.size(); j++) {
				matrix.set(i, j, row.get(j));
			}
		}

		return matrix;
	}

	public static Matrix extractSubMatrix(Matrix parentMatrix, int startRow, int endRow, int startCol, int endCol) {
		double[][] sub = new double[endRow - startRow + 1][endCol - startCol + 1];
		for (int i = startRow; i <= endRow; i++)
			for (int j = startCol; j <= endCol; j++)
				sub[i - startRow][j - startCol] = parentMatrix.get(i, j);
		return new Matrix(sub);
	}

	public static PointTriple extractXYZPosition(Matrix X) {
		double x = X.get(0, 0);
		double y = X.get(1, 0);
		double z = X.get(2, 0);

		return new PointTriple(x, y, z);
	}

	public static Quaternion extractQuaternion(Matrix X) {
		double x = X.get(3, 0);
		double y = X.get(4, 0);
		double z = X.get(5, 0);
		double r = X.get(6, 0);

		Quaternion quaternion = new Quaternion(x, y, z, r);
		return quaternion;
	}

	public static PointTriple extractV(Matrix X) {
		double x = X.get(7, 0);
		double y = X.get(8, 0);
		double z = X.get(9, 0);

		return new PointTriple(x, y, z);
	}

	public static PointTriple extractOmega(Matrix X) {
		double x = X.get(10, 0);
		double y = X.get(11, 0);
		double z = X.get(12, 0);

		return new PointTriple(x, y, z);
	}

	public static Matrix createSameValuedMatrix(double val, int rows, int cols) {
		double[][] matrixArr = new double[rows][cols];
		for (int i = 0; i < rows; i++)
			for (int j = 0; j < cols; j++)
				matrixArr[i][j] = val;

		return new Matrix(matrixArr);
	}

	public static Matrix setSubMatrixValues(Matrix parentMatrix, Matrix newValues, int startRow, int startCol) {

		for (int i = startRow; i < newValues.getRowDimension(); i++)
			for (int j = startCol; j < newValues.getColumnDimension(); j++)
				parentMatrix.set(i, j, newValues.get(i - startRow, j - startCol));

		return parentMatrix;
	}

	public static Matrix quaternionToRotationMatrix(Quaternion quaternion) {
		double x = quaternion.getX();
		double y = quaternion.getY();
		double z = quaternion.getZ();
		double r = quaternion.getR();

		double[][] arr = { { r * r + x * x - y * y - z * z, 2 * (x * y - r * z), 2 * (z * x + r * y) },
				{ 2 * (x * y + r * z), r * r - x * x + y * y - z * z, 2 * (y * z - r * x) },
				{ 2 * (z * x - r * y), 2 * (y * z + r * x), r * r - x * x - y * y + z * z } };

		return new Matrix(arr);
	}

}
