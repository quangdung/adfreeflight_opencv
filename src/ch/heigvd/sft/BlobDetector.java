package ch.heigvd.sft;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import com.parrot.freeflight.activities.ControlDroneActivity;
import com.parrot.freeflight.drone.DroneProxy;
import com.parrot.freeflight.service.DroneControlService;

import android.util.Log;

public class BlobDetector {
	private final static String TAG = "BlobDetector";
	
	ControlDroneActivity activity;
	
	Mat frame = new Mat();
	Mat hsv_image = new Mat();
	Mat thresholded = new Mat();
	Mat thresholded2 = new Mat();
	Mat circles = new Mat();

	Mat array255 = null;
	
//	private DroneProxy droneProxy;
	DroneControlService dcs;
	double temp;
	

	/**
	 * Afin de détecter les objets de couleur rouge, utiliser l’espace HSV 
	 * au lieu de l’espace RGB vu que l’intervalle de couleur est bien défini 
	 * pour les couleurs de base
	 * 
	 * Intervalle de variance des valeurs Hue pour certaines couleurs :
	 * 
	 * Orange  0-22
	 * Jaune 22- 38
	 * Vert 38-75
	 * Bleu 75-130
	 * Violet 130-160
	 * Rouge 160-179 et 0-7
	 */
	Mat distance ;
	List<Mat> lhsv = new ArrayList<Mat>(3);
	
	
	Scalar hsv_min = new Scalar(0, 50, 50, 0);
	Scalar hsv_max = new Scalar(6, 255, 255, 0);
	
	Scalar hsv_min2 = new Scalar(175, 50, 50, 0);
	Scalar hsv_max2 = new Scalar(179, 255, 255, 0);
	
	static class Dimension {
		double x;
		double y;
		
		Dimension(double x, double y) {
			this.x = x;
			this.y = y;
		}
	}
	
	LinkedList<Dimension> position = new LinkedList<Dimension>();
	

	public BlobDetector(ControlDroneActivity activity) {
		this.activity = activity;
		
		position.add(new Dimension(0, 0));
		position.add(new Dimension(0, 0));
	}
	
	public void process(Mat frame) {
		if (dcs == null) {
			dcs = activity.getDCS();
		}
		
		if (array255 == null) {
			array255 = new Mat(frame.rows(), frame.cols(), CvType.CV_8UC1);
			array255.setTo(new Scalar(255));
			distance = new Mat(frame.rows(), frame.cols(), CvType.CV_32F);
		}
		Core.circle(frame, new Point(50, 50), 10, new Scalar(255, 0, 0), -1);
		
		Imgproc.cvtColor(frame, hsv_image, Imgproc.COLOR_RGB2HSV);
		Core.inRange(hsv_image, hsv_min, hsv_max, thresholded);
		Core.inRange(hsv_image, hsv_min2, hsv_max2, thresholded2);
		Core.bitwise_or(thresholded, thresholded2, thresholded);

		/**
		 *  Amélioration de la détection : plus rapide, plus précis
		 */
		Core.split(hsv_image, lhsv);

		Mat H = lhsv.get(0);
		Mat S = lhsv.get(1);
		Mat V = lhsv.get(2);
		
		//Imgproc.cvtColor(H, frame, Imgproc.COLOR_GRAY2RGB);

		Core.subtract(array255, S, S);
		Core.subtract(array255, V, V);

		S.convertTo(S, CvType.CV_32F);
		V.convertTo(V, CvType.CV_32F);

		Core.magnitude(S, V, distance);
		
//		//Imgproc.cvtColor(distance, frame, Imgproc.COLOR_GRAY2RGB);

		Core.inRange(distance, new Scalar(0.0), new Scalar(200.0),
				thresholded2);

		
		Mat thresholded3 = new Mat();
//		Core.bitwise_and(thresholded, thresholded2, thresholded);
		Core.bitwise_and(thresholded, thresholded2, thresholded3);
		
		
//		Imgproc.GaussianBlur(thresholded, thresholded, new Size(9, 9), 0, 0);
		Imgproc.GaussianBlur(thresholded3, thresholded3, new Size(9, 9), 0, 0);
		
		/**
		 * Pour la détection des objets ayant une forme circulaire, 
		 * utilser HoughCircles.
		 */
//		Imgproc.HoughCircles(thresholded, circles,
//				Imgproc.CV_HOUGH_GRADIENT, 2,
//				thresholded.height() / 4, 500, 50, 0, 0);
		
		Imgproc.HoughCircles(thresholded3, circles,
				Imgproc.CV_HOUGH_GRADIENT, 2,
				thresholded3.height() / 4, 500, 50, 0, 0);
		
		int rows = circles.rows();
		int elemSize = (int) circles.elemSize();
		float[] data2 = new float[rows * elemSize / 4];
		if (data2.length > 0) {
			circles.get(0, 0, data2);
			
			for (int i = 0; i < data2.length; i = i + 3) {
				Point center = new Point(data2[i], data2[i + 1]);
				
				Core.ellipse(frame, center, new Size(
						(double) data2[i + 2],
						(double) data2[i + 2]), 0, 0, 360,
						new Scalar(255, 0, 255), 4, 8, 0);
			}
//			
//			Rect r = detect_red_ball(thresholded);
//			/*
//			if (r != null) {
//				Core.rectangle(frame, r.tl(), r.br(), new Scalar(0,
//						255, 0), 2);
//			}
//			
//			Core.putText(
//					frame,
//					"( x = " + 0.5 * (r.tl().x + r.br().x)
//							+ ", y = " + 0.5
//							* (r.tl().y + r.br().y) + " )",
//					new Point(0.5 * (r.tl().x + r.br().x), 0.5 * (r
//							.tl().y + r.br().y)), 1, 1, new Scalar(
//							255, 255, 255));
//			 */
//			
//			position.get(1).x = r.tl().x + r.br().x;
//			position.get(1).y = r.tl().y + r.br().y;

			double difX = position.get(1).x - position.get(0).x;
			double difY = position.get(1).y - position.get(0).y;
			
			if (dcs != null) {
				if (Math.abs(difX) > 50 || Math.abs(difY) > 50) {
					if (difX > 50) { 
						System.out.println("à droite : " + difX);
						dcs.setRoll(1);
					}
					else if (difX < -50) {
						System.out.println("à gauche : " + difX);
						dcs.setRoll(-1);
					}
					
					if (difY > 50) {
						System.out.println("en arrière : " + difY);
						dcs.setPitch(1);
					}
					else if (difY < -50) {
						System.out.println("en avant : " + difY);
						Log.i(TAG, "setPitch");
						dcs.setPitch(-1);
					}
				}
			} else {
				Log.i(TAG, "DroneControlService not available");
			}
			
			position.get(0).x = position.get(1).x;
			position.get(0).y = position.get(1).y;
		}

	}
	
	
	public static Rect detect_red_ball(Mat outmat) {

		Mat v = new Mat();
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		Imgproc.findContours(outmat, contours, v, Imgproc.RETR_LIST,
				Imgproc.CHAIN_APPROX_SIMPLE);

		double maxArea = -1;
		int maxAreaIdx = -1;
		Rect r = null;

		for (int idx = 0; idx < contours.size(); idx++) {
			Mat contour = contours.get(idx);
			double contourarea = Imgproc.contourArea(contour);
			if (contourarea > maxArea) {
				maxArea = contourarea;
				maxAreaIdx = idx;
				r = Imgproc.boundingRect(contours.get(maxAreaIdx));
			}

		}

		v.release();

		return r;
	}
}
