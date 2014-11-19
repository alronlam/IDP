package ekf;

import java.util.ArrayList;

import Jama.Matrix;

public class IDPUtility {
	// this class contains functions ported from MATLAB
	// kept as close to the source as possible
	// yes, this includes comments
	// retrieved from https://svn.openslam.org/data/svn/ekfmonoslam/trunk/matlab_code
	// thank you, monoslam :D
	
	public static void predict_camera_measurements(StateVector x_k_k, Camera cam, ArrayList<FeatureInfo> features_info) {
		// Pinhole Model (whatever that means)
		//t_wc = x_k_k(1:3);
		Matrix t_wc = x_k_k.toMatrix().getMatrix(0, 2, 0, 0);
		
		//r_wc = q2r(x_k_k(4:7));
		Quaternion q = new Quaternion(x_k_k.toMatrix().get(3, 0), x_k_k.toMatrix().get(4, 0), x_k_k.toMatrix().get(5, 0), x_k_k.toMatrix().get(6, 0));
		Matrix r_wc = Helper.quaternionToRotationMatrix(q);
		
		// features = x_k_k(14:end);
		Matrix features = x_k_k.toMatrix().getMatrix(13, x_k_k.toMatrix().getColumnDimension(), 0, 0);
		
		// implying i care about cartesian coords
		for (int i = 0; i < features_info.size(); i++) {
			Matrix yi = features.getMatrix(0, 5, 0, 0);
			IDPFeature y = new IDPFeature(yi.get(0, 0), yi.get(1,0), yi.get(2,0), yi.get(3,0), yi.get(4,0), yi.get(5,0));
			features = features.getMatrix(6, features.getColumnDimension(), 0, 0);
			Matrix hi = hi_inverse_depth(y, t_wc, r_wc, cam, features_info);
			if (hi != null)
				features_info.get(i).h = hi.transpose();
		}
	}
	
	private static Matrix hi_inverse_depth(IDPFeature yinit, Matrix t_wc, Matrix r_wc, Camera cam, ArrayList<FeatureInfo> features_info) {
		// points 3d in camera coordinates
		Matrix r_cw = r_wc.transpose();
		
		double[][] xyz = {{yinit.getX(),yinit.getY(),yinit.getZ()}};
		Matrix yi = new Matrix(xyz);
		double theta = yinit.getAzimuth();
		double phi = yinit.getElevation();
		double rho = yinit.getP();
		
		Matrix mi = Helper.m_function(theta, phi);
		
		// hrl = r_cw*( (yi - t_wc)*rho + mi);
		Matrix hrl = r_cw.times(yi.minus(t_wc).times(rho).plus(mi));
		
		// is in front of camera? [sic]
		double a13 = Math.atan2(hrl.get(0, 0), hrl.get(2, 0)) * 180 / Math.PI;
		double a23 = Math.atan2(hrl.get(1, 0), hrl.get(2, 0)) * 180 / Math.PI;
		
		if (a13 < -60 || a13 > 60 || a23 < -60 || a23 < 60)
			return null;
		
		// image coordinates
		Matrix uv_u = hu(hrl, cam);
		// add distortion
		Matrix uv_d = distort_fm(uv_u, cam);
		
		if (uv_d.get(0,0)>0 && uv_d.get(0,0)<cam.nCols && uv_d.get(1,0)>0 && uv_d.get(1,0)<cam.nRows)
			return uv_d;
		else
			return null;
	}
	
	private static Matrix hu(Matrix yi, Camera cam) {
		double u0 = cam.Cx;
		double v0 = cam.Cy;
		double f = cam.f;
		double ku = 1 / cam.dx;
		double kv = 1 / cam.dy;
		
		double[][] uv_u = new double[2][yi.getColumnDimension()];
		
		for (int i = 0; i < yi.getColumnDimension(); i++) {
			uv_u[0][i] = u0 + (yi.get(0,i)/yi.get(3,i))*f*ku;
			uv_u[1][i] = v0 + (yi.get(0,i)/yi.get(3,i))*f*kv;
		}
		
		return new Matrix(uv_u);
	}
	
	private static Matrix distort_fm(Matrix uv, Camera cam) {
		double xu = (uv.get(0, 0) - cam.Cx) * cam.dx;
		double yu = (uv.get(1, 0) - cam.Cy) * cam.dy;
		
		//ru=sqrt(xu.*xu+yu.*yu);
		//rd=ru./(1+k1*ru.^2+k2*ru.^4);
		double ru = xu * xu + yu * yu;
		double rd = Math.sqrt(ru) / (1 + cam.k1 * ru + cam.k2 * ru * ru);
		
		// this feels like an estimation technique
		for (int i = 0; i < 10; i++) {
			double f = rd + cam.k1*Math.pow(rd, 3) + cam.k2*Math.pow(rd, 5) - ru;
			double f_p = 1 + 3*cam.k1*rd*rd + 5*cam.k2*Math.pow(rd, 4);
			rd = rd - f / f_p;
		}
		
		double D = 1 + cam.k1*rd*rd + cam.k2*rd*rd*rd*rd;
		
		//xd = xu./D;
		//yd = yu./D;
		//uvd = [ xd/dx+Cx; yd/dy+Cy ];
		double[][] out = {{	xu/D/cam.dx + cam.Cx,
							yu/D/cam.dy + cam.Cy}};
		
		return new Matrix(out);
	}
	
	public static void calculate_derivatives(Matrix x_k_km1, Camera cam, ArrayList<FeatureInfo> features_info) {
		Matrix x_v = x_k_km1.getMatrix(0,12,0,0);
		Matrix x_features = x_k_km1.getMatrix(13,x_k_km1.getColumnDimension(),0,0);
		
		for (int i = 0; i < features_info.size(); i++) {
			if (features_info.get(i).h != null) {
				Matrix y = x_features.getMatrix(0,5,0,0);
				x_features = x_features.getMatrix(6,x_features.getColumnDimension(),0,0);
				features_info.get(i).H = calculate_Hi_inverse_depth(x_v, y, cam, i, features_info);
			}
			else {
				x_features = x_features.getMatrix(6,x_features.getColumnDimension(),0,0);
			}
		}
	}
	
	private static Matrix calculate_Hi_inverse_depth(Matrix Xv_km1_k, Matrix yi, Camera cam, int index, ArrayList<FeatureInfo> features_info) {
		Matrix zi = features_info.get(index).h;
		
		int number_of_features = features_info.size();
		// all features in feature_info are inverse depth features
		Matrix Hi = new Matrix(2, 13 + 6 * number_of_features);
		
		Hi.setMatrix(0, 1, 0, 12, dh_dxv(cam,Xv_km1_k,yi,zi));
		
		int insert = 13 + 6 * index;
		Hi.setMatrix(0, 1, insert - 1, insert + 4, dh_dy(cam,Xv_km1_k,yi,zi));
		
		return Hi;
	}
	
	private static Matrix dh_dy(Camera cam, Matrix Xv_km1_k, Matrix yi, Matrix zi) {
		return dh_dhrl(cam,Xv_km1_k,yi,zi).times(dhrl_dy(Xv_km1_k, yi));
	}
	
	private static Matrix dhrl_dy(Matrix Xv_km1_k, Matrix yi) {
		Matrix rw = Xv_km1_k.getMatrix(0,2,0,0);
		
		double[][] a = Xv_km1_k.getMatrix(3,6,0,0).getArray();
		Quaternion q = new Quaternion(a[0][0], a[1][0], a[2][0], a[3][0]);
		Matrix Rrw = Helper.quaternionToRotationMatrix(q).inverse();
		
		double lambda = yi.get(5, 0);
		double phi = yi.get(4, 0);
		double theta = yi.get(3, 0);
		
		double[][] d1 = {{Math.cos(phi) * Math.cos(theta), 0, -Math.cos(phi) * Math.sin(theta)}};
		double[][] d2 = {{-Math.sin(phi) * Math.sin(theta), -Math.cos(phi), -Math.sin(phi) * Math.cos(theta)}};
		
		Matrix dmi_dthetai = Rrw.times(new Matrix(d1).transpose());
		Matrix dmi_dphii = Rrw.times(new Matrix(d2).transpose());
		
		Matrix out = null;

		//a = [lambda*Rrw  dmi_dthetai dmi_dphii Rrw*(yi(1:3)-rw) ];
		
		out.setMatrix(0, 2, 0, 2, Rrw.times(lambda));
		out.setMatrix(3, 3, 0, 2, dmi_dthetai);
		out.setMatrix(4, 4, 0, 2, dmi_dphii);
		out.setMatrix(5, 5, 0, 2, Rrw.times(yi.getMatrix(0,2,0,0).minus(rw)));
		
		return out;
	}
	
	private static Matrix dh_dxv(Camera cam, Matrix Xv_km1_k, Matrix yi, Matrix zi) {
		Matrix Hi1 = new Matrix(2, 12);
		Matrix zero = new Matrix(2, 6);
	    
		//Hi1 = [ dh_drw( camera, Xv_km1_k, yi, zi )  dh_dqwr( camera, Xv_km1_k, yi, zi ) zeros( 2, 6 )];
		Hi1.setMatrix(0, 1, 0, 2, dh_drw(cam, Xv_km1_k, yi, zi));
		Hi1.setMatrix(0, 1, 3, 5, dh_dqwr(cam, Xv_km1_k, yi, zi));
		Hi1.setMatrix(0, 1, 6, 11, zero);
		
		return Hi1;
	}
	
	private static Matrix dh_dqwr(Camera cam, Matrix Xv_km1_k, Matrix yi, Matrix zi) {
		return dh_dhrl(cam,Xv_km1_k,yi,zi).times(dhrl_dqwr(Xv_km1_k,yi));
	}
	
	private static Matrix dhrl_dqwr(Matrix Xv_km1_k, Matrix yi) {
		Matrix rw = Xv_km1_k.getMatrix(0, 2, 0,0);
		Matrix qwr = Xv_km1_k.getMatrix(3,6,0,0);
		
		double lambda = yi.get(5,0);
		double phi = yi.get(4,0);
		double theta = yi.get(3,0);
		
		Matrix mi = Helper.m_function(theta, phi);
		
		//a = dRq_times_a_by_dq( qconj(qwr), ((yi(1:3) - rw)*lambda + mi) )*dqbar_by_dq;
		
		Matrix a = dRq_times_a_by_dq(QuaternionHelper.qconj(qwr), (yi.getMatrix(0,2,0,0).minus(rw).times(lambda).plus(mi)));
		
		// this is dqbar_by_dq
		double[][] out = new double[4][4];
		
		out[0][0] = 1;
		out[1][1] = out[2][2] = out[3][3] = -1;
		
		return a.times(new Matrix(out));
	}
	
	private static Matrix dRq_times_a_by_dq(Matrix q, Matrix aMat) {
		Matrix out = new Matrix(3,4);
		
		Matrix tempR = dR_by_dq0(q);
		Matrix temp31 = tempR.times(aMat);
		out.setMatrix(0, 2, 0, 0, temp31);
		
		tempR = dR_by_dqx(q);
		temp31 = tempR.times(aMat);
		out.setMatrix(0,2,1,1,temp31);
		
		tempR = dR_by_dqy(q);
		temp31 = tempR.times(aMat);
		out.setMatrix(0,2,2,2,temp31);
		
		tempR = dR_by_dqz(q);
		temp31 = tempR.times(aMat);
		out.setMatrix(0,2,3,3,temp31);
		
		return out;
	}
	
	private static Matrix dR_by_dq0(Matrix q) {
		  double q0 = q.get(0,0);
		  double qx = q.get(1,0);
		  double qy = q.get(2,0);
		  double qz = q.get(3,0);
		  
		  double[][] out = {	{2*q0, -2*qz, 2*qy},
				  				{2*qz,  2*q0, -2*qx},
				  				{-2*qy,  2*qx,  2*q0}};
		  
		  return new Matrix(out);
	}
	
	private static Matrix dR_by_dqx(Matrix q) {
		  double q0 = q.get(0,0);
		  double qx = q.get(1,0);
		  double qy = q.get(2,0);
		  double qz = q.get(3,0);
		  
		  double[][] out = {	{2*qx, 2*qy, 2*qz},
				  				{2*qy, -2*qx, -2*q0},
				  				{2*qz, 2*q0,  -2*qx}};
		  
		  return new Matrix(out);
	}
	
	private static Matrix dR_by_dqy(Matrix q) {
		  double q0 = q.get(0,0);
		  double qx = q.get(1,0);
		  double qy = q.get(2,0);
		  double qz = q.get(3,0);
		  
		  double[][] out = {	{-2*qy, 2*qx,  2*q0},
				  				{2*qx, 2*qy,  2*qz},
				  				{-2*q0, 2*qz, -2*qy}};
		  
		  return new Matrix(out);
	}
	
	private static Matrix dR_by_dqz(Matrix q) {
		  double q0 = q.get(0,0);
		  double qx = q.get(1,0);
		  double qy = q.get(2,0);
		  double qz = q.get(3,0);
		  
		  double[][] out = {	{-2*qz, -2*q0, 2*qx},
				  				{2*q0, -2*qz, 2*qy},
				  				{2*qx,  2*qy, 2*qz}};
		  
		  return new Matrix(out);
	}
			  
	private static Matrix dh_drw(Camera cam, Matrix Xv_km1_k, Matrix yi, Matrix zi) {
		return dh_dhrl(cam, Xv_km1_k, yi, zi).times(dhrl_drw(Xv_km1_k, yi));
	}
	
	// the ride never ends
	private static Matrix dhrl_drw(Matrix Xv_km1_k, Matrix yi) {
		Quaternion q = new Quaternion(Xv_km1_k.get(3,0),Xv_km1_k.get(4,0),Xv_km1_k.get(5,0),Xv_km1_k.get(6,0));
		return Helper.quaternionToRotationMatrix(q).inverse().times(-1).times(yi.get(5,0));
	}

	private static Matrix dh_dhrl(Camera cam, Matrix Xv_km1_k, Matrix yi, Matrix zi) {
		return dhd_dhu(cam, zi).times(dhu_dhrl(cam,Xv_km1_k,yi,zi));
	}

	private static Matrix dhd_dhu(Camera cam, Matrix zi) {
		return jacob_undistor_fm(cam, zi).inverse();
	}
	
	private static Matrix jacob_undistor_fm(Camera cam, Matrix uvd) {
		double ud = uvd.get(0,0);
		double vd = uvd.get(1,0);
		double xd = (ud-cam.Cx)*cam.dx;
		double yd = (vd-cam.Cy)*cam.dy;
		
		double rd2 = xd * xd + yd * yd;
		double rd4 = rd2 * rd2;
		
		double[][] a = {	{(1+cam.k1*rd2+cam.k2*rd4)+(ud-cam.Cx)*(cam.k1+2*cam.k2*rd2)*(2*(ud-cam.Cx)*cam.dx*cam.dx),	(ud-cam.Cx)*(cam.k1+2*cam.k2*rd2)*(2*(vd-cam.Cy)*cam.dy*cam.dy)},
							{(vd-cam.Cy)*(cam.k1+2*cam.k2*rd2)*(2*(ud-cam.Cx)*cam.dx*cam.dx), (1+cam.k1*rd2+cam.k2*rd4)+(vd-cam.Cy)*(cam.k1+2*cam.k2*rd2)*(2*(vd-cam.Cy)*cam.dy*cam.dy)}};
		return new Matrix(a);
	}
	
	private static Matrix dhu_dhrl(Camera cam, Matrix Xv_km1_k, Matrix yi, Matrix zi) {
		double ku = 1 / cam.dx;
		double kv = 1 / cam.dy;
		
		Matrix rw = Xv_km1_k.getMatrix(0,2,0,0);
		Quaternion q = new Quaternion(Xv_km1_k.get(3,0),Xv_km1_k.get(4,0),Xv_km1_k.get(5,0),Xv_km1_k.get(6,0));
		Matrix Rrw = Helper.quaternionToRotationMatrix(q).inverse();
		
		double theta = yi.get(3,0);
		double phi = yi.get(4,0);
		double rho = yi.get(5,0);
		
		Matrix mi = Helper.m_function(theta, phi);
		
		Matrix hc = Rrw.times(yi.getMatrix(0, 2,0,0).minus(rw).times(rho).plus(mi));
		
		double hcx = hc.get(0, 0);
		double hcy = hc.get(1, 0);
		double hcz = hc.get(2, 0);
		
		double[][] a = {	{cam.f*ku/hcz, 0, -hcx*cam.f*ku/(hcz*hcz)},
							{0, cam.f*kv/hcz, -hcy*cam.f*kv/(hcz*hcz)}};
		
		return new Matrix(a);
	}
	
}