package ekf;

import Jama.Matrix;

public class DerivativeHelper {

	public static Matrix dtheta_dgw(double X_w, double Z_w) {
		double[][] arr = { { Z_w / (X_w * X_w + Z_w * Z_w), 0, -X_w / (X_w * X_w + Z_w * Z_w) } };
		return new Matrix(arr);
	}

	public static Matrix dphi_dgw(double X_w, double Y_w, double Z_w) {
		double X_w2 = X_w * X_w;
		double Y_w2 = Y_w * Y_w;
		double Z_w2 = Z_w * Z_w;

		double[][] arr = { { (X_w * Y_w) / ((X_w2 + Y_w2 + Z_w2) * Math.sqrt(X_w2 + Z_w2)),
				-Math.sqrt(X_w2 + Z_w2) / (X_w2 + Y_w2 + Z_w2),
				(Z_w * Y_w) / ((X_w2 + Y_w2 + Z_w2) * Math.sqrt(X_w2 + Z_w2)) } };

		return new Matrix(arr);
	}

	public static Matrix dy_dhd(Matrix dyprima_dhd) {
		Matrix _05x1 = Helper.createSameValuedMatrix(0, 5, 1);
		Matrix _01x2 = Helper.createSameValuedMatrix(0, 1, 2);

		Matrix dy_dhd = new Matrix(6, 3);
		dy_dhd = Helper.setSubMatrixValues(dy_dhd, dyprima_dhd, 0, 0);
		dy_dhd = Helper.setSubMatrixValues(dy_dhd, _05x1, 0, 2);
		dy_dhd = Helper.setSubMatrixValues(dy_dhd, _01x2, 5, 0);
		dy_dhd.set(5, 2, 1);

		return dy_dhd;
	}

	public static Matrix dgc_dhu(double fku, double fkv) {
		Matrix dgc_dhu = new Matrix(3, 2);
		dgc_dhu.set(0, 0, 1 / fku);
		dgc_dhu.set(1, 1, 1 / fkv);
		return dgc_dhu;
	}

	public static Matrix dyprima_dgw(Matrix dtheta_dgw, Matrix dphi_dgw) {

		Matrix _03x3 = Helper.createSameValuedMatrix(0, 3, 3);

		Matrix dyprima_dgw = new Matrix(5, 3);
		dyprima_dgw = Helper.setSubMatrixValues(dyprima_dgw, _03x3, 0, 0);
		dyprima_dgw = Helper.setSubMatrixValues(dyprima_dgw, dtheta_dgw, 3, 0);
		dyprima_dgw = Helper.setSubMatrixValues(dyprima_dgw, dphi_dgw, 4, 0);

		return dyprima_dgw;

	}

	public static Matrix dy_dxv(Matrix dy_drw, Matrix dy_dqwr) {

		Matrix _06x6 = Helper.createSameValuedMatrix(0, 6, 6);

		Matrix dy_dxv = new Matrix(6, 6);
		dy_dxv = Helper.setSubMatrixValues(dy_dxv, dy_drw, 0, 0);
		dy_dxv = Helper.setSubMatrixValues(dy_dxv, dy_dqwr, 3, 0);
		dy_dxv = Helper.setSubMatrixValues(dy_dxv, _06x6, 7, 0);

		return dy_dxv;
	}

	public static Matrix dy_dqwr(Matrix dtheta_dqwr, Matrix dphi_dqwr) {
		Matrix _03x4 = Helper.createSameValuedMatrix(0, 3, 4);
		Matrix _01x4 = Helper.createSameValuedMatrix(0, 1, 4);

		Matrix dy_dqwr = new Matrix(6, 4);
		dy_dqwr = Helper.setSubMatrixValues(dy_dqwr, _03x4, 0, 0);
		dy_dqwr = Helper.setSubMatrixValues(dy_dqwr, dtheta_dqwr, 3, 0);
		dy_dqwr = Helper.setSubMatrixValues(dy_dqwr, dphi_dqwr, 4, 0);
		dy_dqwr = Helper.setSubMatrixValues(dy_dqwr, _01x4, 5, 0);

		return dy_dqwr;
	}

	public static Matrix dy_drw() {
		Matrix I3x3 = Helper.createIdentityMatrix(3);
		Matrix _03x3 = Helper.createSameValuedMatrix(0, 3, 3);
		Matrix dy_drw = new Matrix(4, 3);
		dy_drw = Helper.setSubMatrixValues(dy_drw, I3x3, 0, 0);
		dy_drw = Helper.setSubMatrixValues(dy_drw, _03x3, 1, 0);
		return dy_drw;
	}

	public static Matrix dRq_times_a_by_dq(Quaternion q_wc, Matrix XYZ_c) {

		Matrix dRq_times_a_by_dq = new Matrix(3, 4);
		Matrix dR_by_dq0_times_a = dR_by_dq0(q_wc).times(XYZ_c);
		Matrix dR_by_dqx_times_a = dR_by_dqx(q_wc).times(XYZ_c);
		Matrix dR_by_dqy_times_a = dR_by_dqy(q_wc).times(XYZ_c);
		Matrix dR_by_dqz_times_a = dR_by_dqz(q_wc).times(XYZ_c);

		dRq_times_a_by_dq = Helper.setSubMatrixValues(dRq_times_a_by_dq, dR_by_dq0_times_a, 0, 0);
		dRq_times_a_by_dq = Helper.setSubMatrixValues(dRq_times_a_by_dq, dR_by_dqx_times_a, 0, 1);
		dRq_times_a_by_dq = Helper.setSubMatrixValues(dRq_times_a_by_dq, dR_by_dqy_times_a, 0, 2);
		dRq_times_a_by_dq = Helper.setSubMatrixValues(dRq_times_a_by_dq, dR_by_dqz_times_a, 0, 3);

		return dRq_times_a_by_dq;
	}

	private static Matrix dR_by_dq0(Quaternion q) {
		double q0 = q.getR();
		double qx = q.getX();
		double qy = q.getY();
		double qz = q.getZ();

		double[][] arr = { { 2 * q0, -2 * qz, 2 * qy }, { 2 * qz, 2 * q0, -2 * qx }, { -2 * qy, 2 * qx, 2 * q0 } };

		return new Matrix(arr);
	}

	private static Matrix dR_by_dqx(Quaternion q) {
		double q0 = q.getR();
		double qx = q.getX();
		double qy = q.getY();
		double qz = q.getZ();

		double[][] arr = { { 2 * qx, 2 * qy, 2 * qz }, { 2 * qy, -2 * qx, -2 * q0 }, { 2 * qz, 2 * q0, -2 * qx } };

		return new Matrix(arr);
	}

	private static Matrix dR_by_dqy(Quaternion q) {
		double q0 = q.getR();
		double qx = q.getX();
		double qy = q.getY();
		double qz = q.getZ();

		double[][] arr = { { -2 * qy, 2 * qx, 2 * q0 }, { 2 * qx, 2 * qy, 2 * qz }, { -2 * q0, 2 * qz, -2 * qy } };

		return new Matrix(arr);
	}

	private static Matrix dR_by_dqz(Quaternion q) {
		double q0 = q.getR();
		double qx = q.getX();
		double qy = q.getY();
		double qz = q.getZ();

		double[][] arr = { { -2 * qz, -2 * q0, 2 * qx }, { 2 * q0, -2 * qz, 2 * qy }, { 2 * qx, 2 * qy, 2 * qz } };

		return new Matrix(arr);
	}

	public static Matrix Padd(double std_rho, double std_pxl) {
		Matrix Ri = Helper.createIdentityMatrix(2).times(std_pxl * std_pxl);
		Matrix _02x1 = Helper.createSameValuedMatrix(0, 2, 1);
		Matrix _01x2 = Helper.createSameValuedMatrix(0, 1, 2);

		Matrix Padd = new Matrix(3, 3);

		Padd = Helper.setSubMatrixValues(Padd, Ri, 0, 0);
		Padd = Helper.setSubMatrixValues(Padd, _02x1, 0, 2);
		Padd = Helper.setSubMatrixValues(Padd, _01x2, 2, 0);
		Padd.set(2, 2, std_rho * std_rho);

		return Padd;
	}

}
