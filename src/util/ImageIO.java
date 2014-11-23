package util;

import java.io.File;

import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.highgui.Highgui;
import org.opencv.imgproc.Imgproc;

public class ImageIO {
	private String TAG = "ImageIO";
	
	private boolean DEBUG = true;
	
	private final File PATH = new File("input");
	private final String DEFAULT_FILENAME = "00000";
	private final int ZERO_PADDING = 5;
	private final String EXTENSION = ".jpg";
	private final int QUALITY = 100;
	private final MatOfInt PARAMS;
	
	private int lastDigitSaved = -1;
	private int lastDigitLoaded = -1;
	
	
	public ImageIO() {
		int paramInt[] = new int[] {Highgui.CV_IMWRITE_JPEG_QUALITY, QUALITY};
		PARAMS = new MatOfInt(paramInt);
		
		// Create directory
		PATH.mkdir();
		
		// Retrieve digits for continuous saving and loading
		String[] files = PATH.list();
		if (files.length != 0) {
			String firstFile = files[0];
			String digitString = firstFile.split("\\.")[0];
			lastDigitLoaded = Integer.parseInt(digitString);
			
			String lastfile = files[files.length - 1];
			digitString = lastfile.split("\\.")[0];
			lastDigitSaved = Integer.parseInt(digitString);
		}
	}
	
	
	public ImageIO(boolean debug) {
		this();
		DEBUG = debug;
	}
	
	
	public void deletePhotos() {
		File[] photos = PATH.listFiles();
		for (int i = 0; i < photos.length; i++) {
			photos[i].delete();
		}
		lastDigitSaved = -1;
	}
	
	
	private String formatFilename(int digit) {
		String format = "%0" + ZERO_PADDING + "d";
		return String.format(format, digit) + EXTENSION;
	}


	/**
	 * Saves the image using the default filename.
	 */
	public void save(Mat image) {
		save(DEFAULT_FILENAME + EXTENSION, image);
	}
	
	
	/**
	 * Saves the image using the next digit as the filename.
	 */
	public void saveNext(Mat image) {
		lastDigitSaved++;
		save(formatFilename(lastDigitSaved), image);
	}
	
	
	/**
	 * Saves the image in grayscale.
	 * 
	 * @param filename  Requires a file extension.
	 * @param image
	 */
	public void save(String filename, Mat image) {
		File file = new File(PATH, filename);
		filename = file.toString();
		Mat grayImage = new Mat();
		Imgproc.cvtColor(image, grayImage, Imgproc.COLOR_BGR2GRAY);
	    
		boolean result = Highgui.imwrite(filename, grayImage, PARAMS);
		
		if (DEBUG) {
			if (result)
		    	Log.d(TAG, "SUCCESS writing image to " + filename);
		    else
			    Log.d(TAG, "Fail writing to external storage: " + filename);
		}
	}
	
	
	/**
	 * @return The default image.
	 */
	public Mat load() {
		return load(DEFAULT_FILENAME);
	}
	
	
	public Mat loadNext() {
		return load(formatFilename(lastDigitLoaded++));
	}
	
	
	public boolean hasNext() {
		if (lastDigitLoaded <= lastDigitSaved)
			return true;
		else
			return false;
	}
	
	
	/**
	 * @param filename  Requires a file extension.
	 * @return The image in grayscale.
	 */
	public Mat load(String filename) {
		File file = new File(PATH, filename);
		filename = file.toString();
		Mat image = Highgui.imread(filename, Highgui.CV_LOAD_IMAGE_GRAYSCALE);
		
		if (DEBUG) {
			if (!image.empty())
		    	Log.d(TAG, "SUCCESS loading image " + filename);
		    else
			    Log.d(TAG, "Fail loading image " + filename);
		}
		return image;
	}
}
