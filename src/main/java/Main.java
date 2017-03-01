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
    //NetworkTable.setIPAddress("10.37.7.87");
    NetworkTable.initialize();
    
    VideoCapture camera1 = null;
    CvSource imageSource1 = null;
    
    VideoCapture camera2 = null;
    CvSource imageSource2 = null;
    
    // All Mats and Lists should be stored outside the loop to avoid allocations
    // as they are expensive to create
    Mat inputImage = new Mat();
    Mat inputImage2 = new Mat();
//    GripPipelineGreen pipelineGreen = new GripPipelineGreen();
//    GripPipelineRed pipelineRed = new GripPipelineRed();
    NTPipeline ntPipeline = new NTPipeline();
    
    boolean camerasConnected = false;
    boolean cameraReading = false;
    
    double[] defaultHue = {70.06978417266187, 121.22866894197952};
	double[] defaultSat = {93.1025179856115, 255.0};
	double[] defaultVal = {35.773381294964025, 255.0};
    
    // Infinitely process image
    while (true) {
    	
    	if(NetworkTable.connections().length > 0) {
	    	if(!camerasConnected) {
	    		System.out.println("Starting Cameras");
	    		
	    		//USB Cameras
	    		camera1 = startCamera(1);
	    		camera2 = startCamera(0);
	    		
	    		//MJPG Streams
	    		local_ip = getMyIP();
	    		imageSource1 = startStream("front", local_ip, 1188);
	    		imageSource2 = startStream("rear", local_ip, 1187);
	    		
	    		camerasConnected = true;
	    	}
	    	else {
	    		cameraReading = readCameras(camera1, camera2, imageSource1, imageSource2, inputImage, inputImage2, ntPipeline, defaultHue, defaultSat, defaultVal);
	    		
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
  
  
  private static boolean readCameras(VideoCapture camera1, VideoCapture camera2, CvSource imageSource1, CvSource imageSource2, Mat inputImage, Mat inputImage2, NTPipeline ntPipeline, double[] defaultHue, double[] defaultSat, double[] defaultVal) {
	boolean cam1 = false;
	boolean cam2 = false;
	
	NetworkTable smart = NetworkTable.getTable("SmartDashboard");
	double[] hue = orderArrayValues(smart.getNumberArray("HueRange", defaultHue));
	double[] sat = orderArrayValues(smart.getNumberArray("SaturationRange", defaultSat));
	double[] val = orderArrayValues(smart.getNumberArray("ValueRange", defaultVal));
	
//	System.out.print(Double.toString(hue[0]));
//	System.out.print(Double.toString(hue[1]));
//	System.out.print(Double.toString(sat[0]));
//	System.out.print(Double.toString(sat[1]));
//	System.out.print(Double.toString(val[0]));
//	System.out.println(Double.toString(val[1]));
	
	cam1 = camera1.read(inputImage);
	if(cam1) {
		
		ntPipeline.process(inputImage, hue, sat, val, 150, 0, 1000);
		Imgproc.drawContours(inputImage, ntPipeline.filterContoursOutput(), -1, new Scalar(255, 0, 0), 1);
		
		calculateBoiler(inputImage, ntPipeline.filterContoursOutput());
		
		drawBackupCamera(inputImage);
		
		imageSource1.putFrame(inputImage);
	}
	else {
		System.out.println("CAM1 BAD");
	}
	
	cam2 = camera2.read(inputImage2);
	if(cam2) {
		ntPipeline.process(inputImage2, hue, sat, val, 150, 0, 1000);
		Imgproc.drawContours(inputImage2, ntPipeline.filterContoursOutput(), -1, new Scalar(255, 0, 0), 1);
		
		calculateGear(inputImage2, ntPipeline.filterContoursOutput());
		
		//drawBackupCamera(inputImage2);
		
		imageSource2.putFrame(inputImage2);
	}
	else {
		System.out.println("CAM2 BAD");
	}
	
	return cam1 && cam2;
  }
  
  private static double[] orderArrayValues(double[] array) {
	  if(array[0] > array[1]) {
		 double temp = array[1];
		 array[1] = array[0];
		 array[0] = temp;
	  }
	  return array;
  }
  
  private static void drawBackupCamera(Mat inputImage) {
	  Imgproc.line(inputImage, new Point(140,40), new Point(180,40), new Scalar(255, 255, 255));
	  Imgproc.line(inputImage, new Point(145,60), new Point(175,60), new Scalar(255, 255, 255));
	  Imgproc.line(inputImage, new Point(150,80), new Point(170,80), new Scalar(255, 255, 255));
	  
	  Imgproc.line(inputImage, new Point(210,160), new Point(230,160), new Scalar(0, 255, 0));
	  Imgproc.line(inputImage, new Point(205,180), new Point(235,180), new Scalar(0, 255, 255));
	  Imgproc.line(inputImage, new Point(200,200), new Point(240,200), new Scalar(0, 0, 255));
	  
	  Imgproc.line(inputImage, new Point(100,220), new Point(180,150), new Scalar(255, 255, 255),2);
	  Imgproc.line(inputImage, new Point(360,220), new Point(280,150), new Scalar(255, 255, 255),2);
  }
  private static void calculateBoiler(Mat inputImage, ArrayList<MatOfPoint> contours) {
	  int centerX = 0;
	  if(contours.size() > 0) {
		  Rect cRect1 = Imgproc.boundingRect(contours.get(0));
		  
		  centerX = (cRect1.width / 2) + cRect1.x;
		  writeNetworkTables("boiler", 0, centerX, 0, contours.size(), true);
	  }
	  else {
		  writeNetworkTables("boiler", 0, 0, 0, 0, false);
	  }
  }
  private static void calculateGear(Mat inputImage, ArrayList<MatOfPoint> contours) {
	  
	  if(contours.size() == 2) {
		  Rect cRect1 = Imgproc.boundingRect(contours.get(0));
		  Rect cRect2 = Imgproc.boundingRect(contours.get(1));
		  
		  //if there are around the same width
		  //if(cRect1.width + (cRect1.width * 0.3) > cRect2.width && cRect1.width - (cRect1.width * 0.3) < cRect2.width) {
		  //and the same height
		  //if(cRect1.y + (cRect1.y * 0.2) > cRect2.y && cRect1.y - (cRect1.y * 0.2) < cRect2.y) {
			  if(cRect1.x > cRect2.x) {
				  getPegPosition(cRect2,cRect1, inputImage);
				  return;
			  }
			  else {
				  getPegPosition(cRect1,cRect2, inputImage);
				  return;
			  }
		  //}
		  //}
	  }
	  writeNetworkTables("gear", 0, 0, 0, 0, false);	  
	  
  }
  public static int calculateDistanceGear(Rect rect) {
	  return (int) Math.round( (5 * 240) / (2 * rect.height * Math.tan(50)) ) * -1;
  }
  private static void getPegPosition(Rect rLeft, Rect rRight, Mat inputImage) {
	  
	  int centerX = ((rRight.x - rLeft.x) / 2) + rLeft.x + (rLeft.width / 2);
	  int centerY = ((rLeft.y - rLeft.height) / 2) + rLeft.y + (rLeft.height / 2);
	  int distance = calculateDistanceGear(rLeft);
	  
	  writeNetworkTables("gear", distance, centerX, centerY, 2, true);
	  
	  Imgproc.rectangle(inputImage, new Point(rLeft.x,rLeft.y), new Point(rLeft.x + (rRight.x - rLeft.x + rRight.width), rLeft.y+rLeft.height), new Scalar(0, 255, 0), 3);
	  
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
  
  private static HttpCamera setHttpCamera(String cameraName) {
    // Start by grabbing the camera from NetworkTables
    NetworkTable publishingTable = NetworkTable.getTable("CameraPublisher");
    // Wait for robot to connect. Allow this to be attempted indefinitely
    while (true) {
      try {
        if (publishingTable.getSubTables().size() > 0) {
          break;
        }
        Thread.sleep(500);
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }


    HttpCamera camera = null;
    if (!publishingTable.containsSubTable(cameraName)) {
      return null;
    }
    ITable cameraTable = publishingTable.getSubTable(cameraName);
    String[] urls = cameraTable.getStringArray("streams", null);
    if (urls == null) {
      return null;
    }
    ArrayList<String> fixedUrls = new ArrayList<String>();
    for (String url : urls) {
      if (url.startsWith("mjpg")) {
        fixedUrls.add(url.split(":", 2)[1]);
      }
    }
    camera = new HttpCamera("CoprocessorCamera", fixedUrls.toArray(new String[0]));
    return camera;
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