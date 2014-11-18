package ekf;

import java.util.ArrayList;

import Jama.Matrix;

public class FeatureInitializationHelper {

	public static IDPFeature createFeature(PointTriple xyzPosition, Quaternion quaternion, int u, int v,
			double initialRho) {

		double x = xyzPosition.getX();
		double y = xyzPosition.getY();
		double z = xyzPosition.getZ();

		Matrix rotationMatrix = Helper.quaternionToRotationMatrix(quaternion);
		double nx = rotationMatrix.get(0, 0);
		double ny = rotationMatrix.get(1, 0);
		double nz = rotationMatrix.get(2, 0);

		double azimuth = Math.atan2(nx, nz);
		double elevation = Math.atan2(-ny, Math.sqrt(nx * nx + nz * nz));

		return new IDPFeature(x, y, z, azimuth, elevation, initialRho);
	}

	public static ArrayList<ArrayList<Double>> createNewP(ArrayList<ArrayList<Double>> oldP, StateVector Xv, double u,
			double v, double std_rho, double std_pxl) {
		ArrayList<ArrayList<Double>> newP = (ArrayList<ArrayList<Double>>) oldP.clone();

		return newP;
	}

}
