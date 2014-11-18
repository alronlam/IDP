package ekf;

import Jama.Matrix;

public class CameraHelper {

	public static Matrix undistort(int ud, int vd, Camera camera) {
		double Cx = camera.Cx;
		double Cy = camera.Cy;
		double k1 = camera.k1;
		double k2 = camera.k2;
		double dx = camera.dx;
		double dy = camera.dy;

		double xd = (ud - Cx) * dx;
		double yd = (vd - Cy) * dy;

		double rd = Math.sqrt(xd * xd + yd * yd);

		double D = 1 + k1 * rd * rd + k2 * Math.pow(rd, 4);
		double xu = xd * D;
		double yu = yd * D;

		double[][] arr = { { xu / dx + Cx }, { yu / dy + Cy } };
		return new Matrix(arr);
	}

	public static Matrix jacobianUndistort(Camera camera, int ud, int vd) {
		double Cx = camera.Cx;
		double Cy = camera.Cy;
		double k1 = camera.k1;
		double k2 = camera.k2;
		double dx = camera.dx;
		double dy = camera.dy;

		double xd = (ud - Cx) * dx;
		double yd = (vd - Cy) * dy;

		double rd2 = xd * xd + yd * yd;
		double rd4 = rd2 * rd2;

		double uu_ud = (1 + k1 * rd2 + k2 * rd4) + (ud - Cx) * (k1 + 2 * k2 * rd2) * (2 * (ud - Cx) * dx * dx);
		double vu_vd = (1 + k1 * rd2 + k2 * rd4) + (vd - Cy) * (k1 + 2 * k2 * rd2) * (2 * (vd - Cy) * dy * dy);

		double uu_vd = (ud - Cx) * (k1 + 2 * k2 * rd2) * (2 * (vd - Cy) * dy * dy);
		double vu_ud = (vd - Cy) * (k1 + 2 * k2 * rd2) * (2 * (ud - Cx) * dx * dx);

		double[][] arr = { { uu_ud, uu_vd }, { vu_ud, vu_vd } };
		return new Matrix(arr);
	}

}
