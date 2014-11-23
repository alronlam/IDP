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

	public Quaternion times(Quaternion q1) {

		double newX, newY, newZ, newR;

		newR = this.r * q1.r - this.x * q1.x - this.y * q1.y - this.z * q1.z;
		newX = this.r * q1.x + q1.r * this.x + this.y * q1.z - this.z * q1.y;
		newY = this.r * q1.y + q1.r * this.y - this.x * q1.z + this.z * q1.x;
		newZ = this.r * q1.z + q1.r * this.z + this.x * q1.y - this.y * q1.x;

		Quaternion product = new Quaternion(newX, newY, newZ, newR);
		return product;
	}

	public String toString() {
		return "(" + x + ", " + y + ", " + z + ", " + r + ")";
	}
}
