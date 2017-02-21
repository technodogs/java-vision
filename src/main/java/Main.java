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

    // Connect NetworkTables, and get access to the publishing table
    NetworkTable.setClientMode();
    NetworkTable.setTeam(3707);
    //NetworkTable.setIPAddress("127.0.0.1");
    NetworkTable.initialize();
    
    VideoCapture camera1 = null;
    CvSource imageSource1 = null;
    
    VideoCapture camera2 = null;
    CvSource imageSource2 = null;

    // All Mats and Lists should be stored outside the loop to avoid allocations
    // as they are expensive to create
    Mat inputImage = new Mat();
    Mat inputImage2 = new Mat();
    GripPipeline pipeline = new GripPipeline();
    
    boolean camerasConnected = false;
    boolean cameraReading = false;
    
    // Infinitely process image
    while (true) {
    	
    	if(NetworkTable.connections().length > 0) {
	    	if(!camerasConnected) {
	    		System.out.println("Starting Cameras");
	    		
	    		//USB Cameras
	    		camera1 = startCamera(0);
	    		camera2 = startCamera(1);
	    		
	    		//MJPG Streams
	    		local_ip = getMyIP();
	    		imageSource1 = startStream("front", local_ip, 1188);
	    		imageSource2 = startStream("rear", local_ip, 1187);
	    		
	    		camerasConnected = true;
	    	}
	    	else {
	    		cameraReading = readCameras(camera1, camera2, imageSource1, imageSource2, inputImage, inputImage2, pipeline);
	    	}
	    	
	    	if(!cameraReading) {
	    		//System.out.println("NOPE");
	    	}
    	}
    	else {
    		//System.out.println("NOPE");
    	}

    }
    
  }
  
  
  private static boolean readCameras(VideoCapture camera1, VideoCapture camera2, CvSource imageSource1, CvSource imageSource2, Mat inputImage, Mat inputImage2, GripPipeline pipeline) {
	boolean cam1 = false;
	boolean cam2 = false;
	
	cam1 = camera1.read(inputImage);
	if(cam1) {
		pipeline.process(inputImage);
		Imgproc.drawContours(inputImage, pipeline.filterContoursOutput(), -1, new Scalar(255, 0, 0), 1);
		
		//calculateGear(inputImage, pipeline.filterContoursOutput());
		
		drawBackupCamera(inputImage);
		
		imageSource1.putFrame(inputImage);
	}
	else {
		System.out.println("CAM1 BAD");
	}
	
	cam2 = camera2.read(inputImage2);
	if(cam2) {
		pipeline.process(inputImage2);
		Imgproc.drawContours(inputImage2, pipeline.filterContoursOutput(), -1, new Scalar(255, 0, 0), 1);
		
		calculateGear(inputImage2, pipeline.filterContoursOutput());
		
		//drawBackupCamera(inputImage2);
		
		imageSource2.putFrame(inputImage2);
	}
	else {
		System.out.println("CAM2 BAD");
	}
	
	return cam1 && cam2;
  }
  
  private static void drawBackupCamera(Mat inputImage) {
	  Imgproc.line(inputImage, new Point(140,40), new Point(180,40), new Scalar(255, 255, 255));
	  Imgproc.line(inputImage, new Point(145,60), new Point(175,60), new Scalar(255, 255, 255));
	  Imgproc.line(inputImage, new Point(150,80), new Point(170,80), new Scalar(255, 255, 255));
	  
	  Imgproc.line(inputImage, new Point(210,160), new Point(230,160), new Scalar(0, 255, 0));
	  Imgproc.line(inputImage, new Point(205,180), new Point(235,180), new Scalar(0, 255, 255));
	  Imgproc.line(inputImage, new Point(200,200), new Point(240,200), new Scalar(0, 0, 255));
	  
	  Imgproc.line(inputImage, new Point(100,220), new Point(180,150), new Scalar(255, 255, 255));
	  Imgproc.line(inputImage, new Point(360,220), new Point(280,150), new Scalar(255, 255, 255));
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
		  
		  int cnt = 1;
		  if(bestTape.hasNeighbor) {
			  cnt = 2;
			  //write to network tables
			  int peg = bestTape.centerX + ((bestTape.centerXNeighbor - bestTape.centerX) / 2);
			  writeNetworkTables("gear", bestTape.distance, peg, bestTape.centerY, cnt, true);
			  Imgproc.drawContours(inputImage, best, -1, new Scalar(0, 255, 0), 3);
		  }
		  else {
			  writeNetworkTables("gear", 0, 0, 0, 1, false);
		  }
		  
		  
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
  
  public static String getMyIP()
  {
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
	  return local_ip;
  }

}