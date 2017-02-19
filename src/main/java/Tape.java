import org.opencv.core.Rect;

public class Tape {
	public boolean hasNeighbor = false;
	public int distanceNeighbor = 0;
	
	public int distance = 0;
	public int angle = 0;
	public int centerX = 0;
	public int centerY = 0;
	public double ratio = 0;
	public Rect rect = null;
	
	public void setRect(Rect newRect) {
		this.rect = newRect;
		this.centerX = Math.round((rect.x + rect.width)/2);
		this.centerY = Math.round((rect.y + rect.height)/2);
		this.ratio = rect.width / rect.height;
	}
	
	public boolean checkNeighborGearTape(Tape tape2) {
		boolean yMatch = false;
		boolean xMatch = false;
		
		if((this.centerY + 10) > tape2.centerY && 
		   (this.centerY - 10) < tape2.centerY) {
			yMatch = true;
		}
		if(((this.rect.height * 1.6) + this.centerX - 30) < tape2.centerX && 
		   ((this.rect.height * 1.6) + this.centerX + 30) > tape2.centerX) {
			xMatch = true;
		}
		
		this.hasNeighbor = yMatch && xMatch;
//		if(this.hasNeighbor) {
//			tape2.calculateDistanceGear();
//			this.distanceNeighbor = tape2.distanceNeighbor;
//		}
		
		return this.hasNeighbor;
	}
	
	public void calculateDistanceGear() {
		this.distance = (int) Math.round( (5 * 240) / (2 * this.rect.height * Math.tan(50)) ) * -1;
	}
	
	static public boolean isValidGearTape(Rect newRect) {
		double testRatio = ((double)newRect.width / (double)newRect.height);
		//System.out.println(testRatio);
		if(testRatio > 0.3 && testRatio < 0.7) {
			return true;
		}
		else {
			return false;
		}
	}
}
