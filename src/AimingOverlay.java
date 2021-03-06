package org.usfirst.frc.team2077.vision.processors;

import java.util.Map;
import java.util.TreeMap;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team2077.vision.Main.FrameProcessor;
import org.usfirst.frc.team2077.vision.NTMain;

import edu.wpi.first.networktables.NetworkTableEntry;

public class AimingOverlay implements FrameProcessor {

    private Map<String,NetworkTableEntry> nte_ = new TreeMap<>();

    @Override
    public void processFrame( Mat frameMat, Mat overlayMat ) {
    
        Scalar red = new Scalar(0,0,255,255);
        Scalar green = new Scalar(0,255,0,255);
        Scalar white = new Scalar(255,255,255,255);
        Scalar black = new Scalar(0,0,0,255);
    
        int rows = overlayMat.rows();
        int cols = overlayMat.cols();
    
        Imgproc.line(overlayMat, new Point(cols/2, 0), new Point(cols/2, rows), green, 1);
        
        NetworkTableEntry nte;
        if ( (nte = getNTE("Crosshairs")) != null ) {
            double[] crosshairs = nte.getDoubleArray(new double[0]);
            if (crosshairs != null && crosshairs.length >= 4) {
              	// draw crosshairs
                double x = crosshairs[0];
                double y = crosshairs[1];
                double w = crosshairs[2];
                double h = crosshairs[3];
                x += w/2;
                y += h/2;
                // CHANGED 3/6 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                x = Math.round( Math.max(0, Math.min(cols-1, x)) );
                y = rows-y;
                y = Math.round( Math.max(0, y) );
                // CHANGED 3/6 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                Imgproc.line( overlayMat, new Point((int)x, 0), new Point((int)x, rows), white, 1);
                Imgproc.line( overlayMat, new Point(0, (int)y), new Point(cols, (int)y), white, 1);
            }
        }
        if ( (nte = getNTE("Position")) != null ) {
            double[] position = nte.getDoubleArray(new double[0]);
            if (position != null && position.length >= 3) {
                String location = "Location:" + (Math.round(position[0]*10.)/10.) + "N " + (Math.round(position[1]*10.)/10.) + "E";
                String heading = "Heading:" + (Math.round(position[2]*10.)/10.);
                drawText(overlayMat, location, 20, rows/2-80);
                drawText(overlayMat, heading, 20, rows/2-40);
            }
        }
        // CHANGED 3/6 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if ( (nte = getNTE("Target")) != null ) {
            double[] target = nte.getDoubleArray(new double[0]);
            if (target != null && target.length >= 2) {
                String range = "Direction:" + (Math.round(target[0]*10.)/10.);
                String angle = "Range:" + (Math.round(target[1]*10.)/10.);
                // CHANGED 3/6 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                drawText(overlayMat, range, 20, rows/2+0);
                drawText(overlayMat, angle, 20, rows/2+40);
            }
        }
    }

    private void drawText(Mat mat, String text, double x, double y) {
        Imgproc.putText( mat, text, new Point(x, y), Core.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0,0,0,255), 2 );
        Imgproc.putText( mat, text, new Point(x, y), Core.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255,255,255,255), 1 );
    }

    private NetworkTableEntry getNTE( String key ) {
        NetworkTableEntry nte;
        if ( NTMain.networkTable_ != null
          && ( (nte = nte_.get(key)) != null
            || ( (nte = NTMain.networkTable_.getEntry(key)) != null
              && nte_.put(key, nte) == null ) ) ) {
            return nte;
        }
        return null;
    }
}
