package ekf;

import java.util.ArrayList;

import Jama.Matrix;

public class FeatureInfo {
	private int half_patch_size_when_initialized = 20;
	private int half_patch_size_when_matching = 6;
	private int init_frame;
	private int individually_compatible;
	private int low_innovation_inlier;
	private int high_innovation_inlier;
	private int state_size;
	private int measurement_size;
	
	public int times_predicted = 0;
	public int times_measured = 0;
	
	private Matrix patch_when_initialized;
	private Matrix patch_when_matching;
	private Matrix r_wc_when_initialized;
	private Matrix R_wc_when_initialized;
	private Matrix uv_when_initialized;
	private Matrix init_measurement;

	public Matrix z, h, H, S, R;
	
	private IDPFeature yi;
	
	
	public FeatureInfo(Matrix uv, Matrix im_k, Matrix X_RES, ArrayList<FeatureInfo> features_info, 
			int step, IDPFeature newFeature) {
		patch_when_initialized = im_k.getMatrix((int)uv.get(1,0)-half_patch_size_when_initialized, (int)uv.get(1,0) + half_patch_size_when_initialized,
				(int)uv.get(0, 0) - half_patch_size_when_initialized, (int)uv.get(0, 0) + half_patch_size_when_initialized);
		
		patch_when_matching = new Matrix(2 * half_patch_size_when_matching + 1, 2 * half_patch_size_when_matching + 1);

		r_wc_when_initialized = X_RES.getMatrix(0, 2, 0, 0);
		
		Quaternion q = new Quaternion(X_RES.get(0,0),X_RES.get(1,0),X_RES.get(2,0),X_RES.get(3,0));
		R_wc_when_initialized = Helper.quaternionToRotationMatrix(q);
		
		uv_when_initialized = uv.transpose();
		
		init_frame = step;
		
		init_measurement = uv;
		
		yi = newFeature;
		
		individually_compatible = 0;
		low_innovation_inlier = 0;
		high_innovation_inlier = 0;
		
		z = null;
		h = null;
		H = null;
		S = null;
		
		state_size = 6;
		measurement_size = 2;
		
		R = Matrix.identity(measurement_size, measurement_size);
	}
	
	public void update() {
		if (h != null)
			times_predicted++;
//		if (low_innovation_inlier || high_innovation_inlier)
			times_measured++;
			
			individually_compatible = 0;
			low_innovation_inlier = 0;
			high_innovation_inlier = 0;
			
			z = null;
			h = null;
			H = null;
			S = null;
	}
}
