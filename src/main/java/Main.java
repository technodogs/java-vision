import java.util.ArrayList;

import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.wpilibj.tables.*;
import edu.wpi.cscore.*;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.*;

public class Main {
  public static void main(String[] args) {
    // Loads our OpenCV library. This MUST be included
    System.loadLibrary("opencv_java310");
    
    String local_ip = "127.0.0.1";

    // Connect NetworkTables, and get access to the publishing table
    NetworkTable.setClientMode();
    // Set your team number here
    //NetworkTable.setTeam(9999);
    NetworkTable.setIPAddress("127.0.0.1");
    NetworkTable.initialize();

    VideoCapture camera1 = startCamera(0);
    CvSource imageSource1 = startStream("rear", local_ip, 1186);
    
//    VideoCapture camera2 = startCamera(1);
//    CvSource imageSource2 = startStream("front", local_ip, 1187);

    // All Mats and Lists should be stored outside the loop to avoid allocations
    // as they are expensive to create
    Mat inputImage = new Mat();
    Mat hsv = new Mat();
    int i = 0;
    
    // Infinitely process image
    while (true) {
    	if(camera1.read(inputImage)) {
    		//System.out.println("read" + Integer.toString(i));
    		imageSource1.putFrame(inputImage);
    		//i++;
    	}
//    	if(camera2.read(inputImage)) {
//    		//System.out.println("read" + Integer.toString(i));
//    		imageSource2.putFrame(inputImage);
//    		//i++;
//    	}
    }
    
  }
  
  private static VideoCapture startCamera(int device) {
	VideoCapture camera = new VideoCapture(device);
	camera.set(Videoio.CAP_PROP_FRAME_WIDTH, 640);
	camera.set(Videoio.CAP_PROP_FRAME_HEIGHT, 480);
	
	return camera;
  }
  private static CvSource startStream(String webcam_name, String local_ip, int local_port) {
	CvSource imageSource = new CvSource(webcam_name, VideoMode.PixelFormat.kMJPEG, 640, 480, 30);
	MjpegServer cvStream = new MjpegServer(webcam_name, local_port);
	cvStream.setSource(imageSource);
	
	// setup grip pipeline from grip.py
	//self.__grip_pipeline = GripPipeline()
	
	// write camera info
	NetworkTable table = NetworkTable.getTable("CameraPublisher");
	ITable camTable = table.getSubTable(webcam_name);
	
	String[] streams = new String[1];
	streams[0] = "mjpg:http://" + local_ip + ":" + Integer.toString(local_port) + "/stream.mjpg?name="+webcam_name;
	camTable.putString("source", "usb:IMAQdx/");
	camTable.putStringArray("streams", streams);
	
	return imageSource;
  }

}