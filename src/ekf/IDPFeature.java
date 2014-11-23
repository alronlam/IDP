package ekf;

import Jama.Matrix;

public class IDPFeature {

	private double x;
	private double y;
	private double z;
	private double azimuth;
	private double elevation;
	private double p;

	public IDPFeature(double x, double y, double z, double azimuth, double elevation, double p) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.azimuth = azimuth;
		this.elevation = elevation;
		this.p = p;
	}
	
	public IDPFeature(Matrix m) {
		this.x = m.get(0,0);
		this.y = m.get(1,0);
		this.z = m.get(2,0);
		this.azimuth = m.get(3,0);
		this.elevation = m.get(4,0);
		this.p = m.get(5,0);
	}

	public double getX() {
		return x;
	}

	public void setX(double x) {
		this.x = x;
	}

	public double getY() {
		return y;
	}

	public void setY(double y) {
		this.y = y;
	}

	public double getZ() {
		return z;
	}

	public void setZ(double z) {
		this.z = z;
	}

	public double getAzimuth() {
		return azimuth;
	}

	public void setAzimuth(double azimuth) {
		this.azimuth = azimuth;
	}

	public double getElevation() {
		return elevation;
	}

	public void setElevation(double elevation) {
		this.elevation = elevation;
	}

	public double getP() {
		return p;
	}

	public void setP(double p) {
		this.p = p;
	}

}
