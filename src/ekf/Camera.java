package ekf;

public class Camera {
	// mirror of the camera class from
	// https://svn.openslam.org/data/svn/ekfmonoslam/trunk/matlab_code/initialize_cam.m
	
	public double d;
	public double nRows;
	public double nCols;
	public double Cx;
	public double Cy;
	public double k1;
	public double k2;
	public double f;
	public double dx;
	public double dy;
	public double[][] K;
	
	public Camera() {
		// we have to replace these with our own camera values
		// what these values are, though, is beyond me
		
		d = dx = dy = 0.0112;
		nRows = 320;
		nCols = 240;
		Cx = 1.7945 / d;
		Cy = 1.4433 / d;
		k1 = 6.333e-2;
		k2 = 1.390e-2;
		f = 2.1735;
		
		K = new double[3][3];
		
		K[0][0] = K[1][1] = f/d;
		K[0][2] = Cx;
		K[1][2] = Cy;
		K[2][2] = 1;
	}
}
