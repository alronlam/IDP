package ekf;

public class PointTriple {
	private double x;
	private double y;
	private double z;

	public PointTriple(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
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

	public PointTriple getCopy() {
		return new PointTriple(x, y, z);
	}

	public PointTriple times(double val) {
		return new PointTriple(x * val, y * val, z * val);
	}

	public PointTriple plus(PointTriple other) {
		return new PointTriple(x + other.x, y + other.y, z + other.z);
	}

	public PointTriple divide(double val) {
		return new PointTriple(x / val, y / val, z / val);
	}

	public double getNorm() {
		return Math.sqrt(x * x + y * y + z * z);
	}

	public String toString() {
		return "(" + x + "," + y + "," + z + ")";
	}
}
