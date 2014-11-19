package ekf;

public class Quaternion {
	private double x;
	private double y;
	private double z;
	private double r;

	public Quaternion(PointTriple vector, double theta) {
		r = Math.cos(theta / 2);

		vector = vector.times(Math.sin(theta / 2)).divide(vector.getNorm());

		x = vector.getX();
		y = vector.getY();
		z = vector.getZ();
	}

	public Quaternion(double x, double y, double z, double r) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.r = r;
	}

	public double getR() {
		return r;
	}

	public void setR(double r) {
		this.r = r;
	}

	public double getZ() {
		return z;
	}

	public void setZ(double z) {
		this.z = z;
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

	public Quaternion times(Quaternion other) {
		// this is probably wrong
		return new Quaternion(x * other.x, y * other.y, z * other.z, r * other.r);
	}

}
