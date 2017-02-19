import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.wpilibj.tables.*;
import edu.wpi.cscore.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.*;

public class Main {
  public static void main(String[] args) {
    // Loads our OpenCV library. This MUST be included
    System.loadLibrary("opencv_java310");
    
    String local_ip = "127.0.0.1";
    InetAddress IP = null;
	try {
		IP = InetAddress.getLocalHost();
	} catch (UnknownHostException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	if(IP != null) {
		local_ip = IP.getHostAddress();
		System.out.println("IP of my system is := "+IP.getHostAddress());
	}

    // Connect NetworkTables, and get access to the publishing table
    NetworkTable.setClientMode();
    // Set your team number here
    //NetworkTable.setTeam(3707);
    NetworkTable.setIPAddress("127.0.0.1");
    NetworkTable.initialize();
    
    //windows-x86_64_2015

    VideoCapture camera1 = null;
    CvSource imageSource1 = startStream("rear", local_ip, 1188);
    
    VideoCapture camera2 = null;
    CvSource imageSource2 = startStream("front", local_ip, 1187);

    // All Mats and Lists should be stored outside the loop to avoid allocations
    // as they are expensive to create
    Mat inputImage = new Mat();
    GripPipeline pipeline = new GripPipeline();
    
    boolean camerasConnected = false;
    boolean cameraReading = false;
    
    // Infinitely process image
    while (true) {
    	
    	if(NetworkTable.connections().length > 0) {
	    	if(!camerasConnected) {
	    		System.out.println("GO");
	    		camera1 = startCamera(0);
	    		camera2 = startCamera(1);
	    		camerasConnected = true;
	    	}
	    	else {
	    		cameraReading = readCameras(camera1, camera2, imageSource1, imageSource2, inputImage, pipeline);
	    	}
	    	
	    	if(!cameraReading) {
	    		//System.out.println("NOPE");
	    	}
    	}

    }
    
  }
  
  
  private static boolean readCameras(VideoCapture camera1, VideoCapture camera2, CvSource imageSource1, CvSource imageSource2, Mat inputImage, GripPipeline pipeline) {
	boolean cam1 = false;
	boolean cam2 = false;
	
	cam1 = camera1.read(inputImage);
	if(cam1) {
		pipeline.process(inputImage);
		Imgproc.drawContours(inputImage, pipeline.filterContoursOutput(), -1, new Scalar(255, 0, 0), 3);
		
		calculateGear(inputImage, pipeline.filterContoursOutput());
		
		drawBackupCamera(inputImage);
		
		imageSource1.putFrame(inputImage);
	}
	
	cam2 = camera2.read(inputImage);
	if(cam2) {
		pipeline.process(inputImage);
		Imgproc.drawContours(inputImage, pipeline.filterContoursOutput(), -1, new Scalar(255, 0, 0), 3);
		
		calculateGear(inputImage, pipeline.filterContoursOutput());
		
		drawBackupCamera(inputImage);
		
		imageSource2.putFrame(inputImage);
	}
	
	return cam1 && cam2;
  }
  
  private static void drawBackupCamera(Mat inputImage) {
	  Imgproc.line(inputImage, new Point(140,40), new Point(180,40), new Scalar(255, 255, 255));
	  Imgproc.line(inputImage, new Point(145,60), new Point(175,60), new Scalar(255, 255, 255));
	  Imgproc.line(inputImage, new Point(150,80), new Point(170,80), new Scalar(255, 255, 255));
	  
	  Imgproc.line(inputImage, new Point(150,160), new Point(170,160), new Scalar(0, 255, 0));
	  Imgproc.line(inputImage, new Point(145,180), new Point(175,180), new Scalar(0, 255, 255));
	  Imgproc.line(inputImage, new Point(140,200), new Point(180,200), new Scalar(0, 0, 255));
	  
	  Imgproc.line(inputImage, new Point(20,220), new Point(100,150), new Scalar(255, 255, 255));
	  Imgproc.line(inputImage, new Point(300,220), new Point(220,150), new Scalar(255, 255, 255));
  }
  
  private static void calculateGear(Mat inputImage, ArrayList<MatOfPoint> contours) {
	  
	  Tape bestTape = null;
	  MatOfPoint bestContour = null;
	  
	  for (MatOfPoint c : contours) {
		  
		  Rect cRect = Imgproc.boundingRect(c);
		  
		  //check if its a tall piece of tape
		  if(Tape.isValidGearTape(cRect)) { 
			  //System.out.println("*****GOOD ratio");
			  
			  Tape tape1 = new Tape();
			  tape1.setRect(cRect);
			  
			  //see if there is another valid tape to the right
			  Tape tape2 = new Tape();
			  for(MatOfPoint c2 : contours) {
				  if(c != c2) {
					  tape2.setRect(Imgproc.boundingRect(c2));
					  if(tape1.checkNeighborGearTape(tape2)) {
						  break;
					  }
				  }
			  }
			  
			  //if we have a neighbor tape, set this tape to best and stop checking others
			  if(tape1.hasNeighbor) {
				  bestTape = tape1;
				  bestContour = c;
				  break;
			  }
			  //else if this is the first validTape, use it but keep looking
			  else if(bestTape == null) {
				  bestTape = tape1;
				  bestContour = c;
			  }
		  }
		  
	  }
	  
	  if(bestTape != null) {
		  bestTape.calculateDistanceGear();
		  
		  //write the contour that we thought was best to image
		  ArrayList<MatOfPoint> best = new ArrayList<MatOfPoint>();
		  best.add(bestContour);
		  Imgproc.drawContours(inputImage, best, -1, new Scalar(0, 255, 0), 3);
		  
		  int cnt = 1;
		  if(bestTape.hasNeighbor) {
			  cnt = 2;
		  }
		  
		  //write to network tables
		  writeNetworkTables("gear", bestTape.distance, bestTape.centerX, bestTape.centerY, cnt, true);
	  }
	  else {
		  writeNetworkTables("gear", 0, 0, 0, 0, false);
	  }
	  
	  
  }
  private static void writeNetworkTables(String target, int distance, int centerX, int centerY, int tapeCount, boolean locked) {
	  NetworkTable table = NetworkTable.getTable("DogVision");
	  ITable calcTable = table.getSubTable(target);
	  
	  calcTable.putNumber("distance", (double)distance);
	  calcTable.putNumber("centerX", (double)centerX);
	  calcTable.putNumber("centerY", (double)centerY);
	  calcTable.putNumber("tapeCount", (double)tapeCount);
	  calcTable.putBoolean("locked", locked);
  }
  
  private static VideoCapture startCamera(int device) {
	VideoCapture camera = new VideoCapture(device);
	camera.set(Videoio.CAP_PROP_FRAME_WIDTH, 320);
	camera.set(Videoio.CAP_PROP_FRAME_HEIGHT, 240);
	
	return camera;
  }
  private static CvSource startStream(String webcam_name, String local_ip, int local_port) {
	CvSource imageSource = new CvSource(webcam_name, VideoMode.PixelFormat.kMJPEG, 320, 240, 30);
	MjpegServer cvStream = new MjpegServer(webcam_name, local_port);
	cvStream.setSource(imageSource);
	
	// write camera info
	NetworkTable table = NetworkTable.getTable("CameraPublisher");
	ITable camTable = table.getSubTable(webcam_name);
	
	String[] streams = new String[1];
	streams[0] = "mjpg:http://" + local_ip + ":" + Integer.toString(local_port) + "/stream.mjpg?name="+webcam_name;
	camTable.putString("source", "usb:IMAQdx/");
	camTable.putStringArray("streams", streams);
	
	return imageSource;
  }
  public static double[] toDoubleArr(ArrayList<Double> sourceArray)
  {
      double[] ret = new double[sourceArray.size()];
      for (int i=0; i < ret.length; i++)
      {
          ret[i] = sourceArray.get(i).doubleValue();
      }
      return ret;
  }

}