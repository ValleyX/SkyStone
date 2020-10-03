//package com.disnodeteam.dogecv.detectors.skystone;
package org.firstinspires.ftc.teamcode.dogecvDetectors;

import android.widget.HorizontalScrollView;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.HSVRangeFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class CameraRingDetectorTest extends DogeCVDetector {
    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Setting to decide to use MaxAreaScorer or PerfectAreaScorer

    //Create the default filters and scorers
    //public DogeCVColorFilter blackFilter = new GrayscaleFilter(0, 25);
    //public DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 70); //Default Yellow blackFilter
    //public DogeCVColorFilter orangeFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.RED, 70); //Default Yellow blackFilter
    private Scalar upper = new Scalar(25, 60, 90); // lighter orange
    private Scalar lower = new Scalar(35, 100, 110); // darker orange

    // target orange (30, 80, 100) (hue, saturation, value/brightness)
    public DogeCVColorFilter orangeFilter = new HSVRangeFilter(lower, upper);


    public RatioScorer ratioScorer = new RatioScorer(1.0, 3); // Used to find the short face of the stone
    public MaxAreaScorer maxAreaScorer = new MaxAreaScorer( 0.01);                    // Used to find largest objects
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(5000,0.05); // Used to find objects near a tuned area value
    //public int RequestedYLine     = 0;    // Countour y must be greater than this
    //public int RequestedXRightLine = 320;
    //public int RequestedLeftMid = 100;
    //public int RequestedRightMid = 200;
    //public int RequestedXLeftLine = 0;
    public int RequestedTopYLine = 160;
    public int RequestedMidYLine = 215;
    public int RequestedBottomYLine = 250;

    // Results of the detector
    private Point screenPosition = new Point(); // Screen position of the mineral
    private Rect foundRect = new Rect(); // Found rect

    private Mat rawImage = new Mat();
    private Mat workingMat = new Mat();
    private Mat displayMat = new Mat();
    private Mat orangeMask = new Mat();
    private Mat yellowMask = new Mat();
    private Mat hierarchy  = new Mat();

    public Point getScreenPosition() {
        return screenPosition;
    }

    public Rect foundRectangle() {
        return foundRect;
    }


    public CameraRingDetectorTest() {
        detectorName = "Skystone Detector";
    }

    @Override
    public Mat process(Mat input) {
        input.copyTo(rawImage);
        input.copyTo(workingMat);
        input.copyTo(displayMat);
        input.copyTo(orangeMask);

        // Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        //yellowFilter.process(workingMat.clone(), yellowMask);

        List<MatOfPoint> contoursYellow = new ArrayList<>();

        Imgproc.findContours(yellowMask, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursYellow,-1,new Scalar(255,30,30),2);


        // Current result
        Rect bestRect = foundRect;
        double bestDifference = Double.MAX_VALUE; // MAX_VALUE since less difference = better

        // Loop through the contours and score them, searching for the best result
        for(MatOfPoint cont : contoursYellow){
            double score = calculateScore(cont); // Get the difference score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0,0,255),2); // Draw rect

            // If the result is better then the previously tracked one, set this rect as the new best
            if(score < bestDifference){
                bestDifference = score;
                bestRect = rect;
            }
        }

        Imgproc.rectangle(orangeMask, bestRect.tl(), bestRect.br(), new Scalar(0,0,255), 1, Imgproc.LINE_4, 0);
        orangeFilter.process(workingMat.clone(), orangeMask);
        List<MatOfPoint> contoursBlack = new ArrayList<>();

        Imgproc.findContours(orangeMask, contoursBlack, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursBlack,-1,new Scalar(40,40,40),2);

        // top y line
        Point topLeft = new Point(0, RequestedTopYLine);
        Point topRight = new Point(600, RequestedTopYLine);
        Imgproc.line (displayMat, topLeft, topRight, new Scalar(0,0,255), 2);

        // mid y line
        Point midLeft = new Point(0, RequestedMidYLine);
        Point midRight = new Point(600, RequestedMidYLine);
        Imgproc.line (displayMat, midLeft, midRight, new Scalar(0,255,0), 2);

        // bottom y line
        Point bottomLeft = new Point(0, RequestedBottomYLine);
        Point bottomRight = new Point(600, RequestedBottomYLine);
        Imgproc.line (displayMat, bottomLeft, bottomRight, new Scalar(255,0,0), 2);



        for(MatOfPoint cont : contoursBlack){
            double score = calculateScore(cont); // Get the difference score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0,0,255),2); // Draw rect

            // If the result is better then the previously tracked one, set this rect as the new best
            //if(score < bestDifference){
            if ((score < bestDifference) && (rect.y >= RequestedTopYLine) && (rect.x <= RequestedBottomYLine))
            {
                bestDifference = score;
                bestRect = rect;
            }
        }
        if(bestRect != null) {
            // Show chosen result
            Imgproc.rectangle(displayMat, bestRect.tl(), bestRect.br(), new Scalar(255,0,0),4);
            Imgproc.putText(displayMat, "Chosen", bestRect.tl(),0,1,new Scalar(255,255,255));

            screenPosition = new Point(bestRect.x, bestRect.y);
            foundRect = bestRect;
            found = true;
        }
        else {
            found = false;
        }

        switch (stageToRenderToViewport) {
            case THRESHOLD: {
                Imgproc.cvtColor(orangeMask, orangeMask, Imgproc.COLOR_GRAY2BGR);

                return orangeMask;
            }
            case RAW_IMAGE: {
                return rawImage;
            }
            default: {
                return displayMat;
            }
        }
    }

    @Override
    public void useDefaults() {
        addScorer(ratioScorer);

        // Add diffrent scorers depending on the selected mode
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }
    }

    public void SetRequestedTopYLine(int y) { RequestedTopYLine = y;}
    public void SetRequestedBottomYLine (int y2) {RequestedBottomYLine = y2;}
    public void SetRequestedMidYLine (int y3) {RequestedMidYLine = y3;}
    //public void SetRequestedXLeftLine (int x) {RequestedXLeftLine = x;}
/*
    public void SetRequestedMidlinesRightLine (int leftmid, int rightmid)
    {
        RequestedLeftMid = leftmid;
        RequestedRightMid = rightmid;
    }

 */

}
