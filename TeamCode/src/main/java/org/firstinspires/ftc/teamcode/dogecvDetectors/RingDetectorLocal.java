//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package org.firstinspires.ftc.teamcode.dogecvDetectors;

import com.disnodeteam.dogecv.DogeCV.AreaScoringMethod;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.HSVColorFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter.ColorPreset;
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
import java.util.Iterator;
import java.util.List;

public class RingDetectorLocal extends DogeCVDetector {
    public AreaScoringMethod areaScoringMethod;
    public DogeCVColorFilter blackFilter;
    public DogeCVColorFilter yellowFilter;
    public RatioScorer ratioScorer;
    public MaxAreaScorer maxAreaScorer;
    public PerfectAreaScorer perfectAreaScorer;
    public int RequestedTopYLine = 160;
    public int RequestedMidYLine = 215;
    public int RequestedBottomYLine = 250;
    private Point screenPosition;
    private Rect foundRect;
    private Mat rawImage;
    private Mat workingMat;
    private Mat displayMat;
    private Mat blackMask;
    private Mat yellowMask;
    private Mat hierarchy;

    public Point getScreenPosition() {
        return this.screenPosition;
    }

    public Rect foundRectangle() {
        return this.foundRect;
    }

    public RingDetectorLocal() {
        this.areaScoringMethod = AreaScoringMethod.MAX_AREA;
        //this.blackFilter = new GrayscaleFilter(0, 25);
       // Scalar oranage = new Scalar(58.0, 52.0D, 38.0D);
        Scalar oranage = new Scalar(30, 80, 80);
        //30, 80, 100
        Scalar range = new Scalar(20.0D, 20.0D, 20.0D);
        this.blackFilter = new HSVColorFilter(oranage, range);

        this.yellowFilter = new LeviColorFilter(ColorPreset.YELLOW, 70.0D);
        this.ratioScorer = new RatioScorer(1.25D, 3.0D);
        this.maxAreaScorer = new MaxAreaScorer(0.01D);
        this.perfectAreaScorer = new PerfectAreaScorer(5000.0D, 0.05D);
        this.screenPosition = new Point();
        this.foundRect = new Rect();
        this.rawImage = new Mat();
        this.workingMat = new Mat();
        this.displayMat = new Mat();
        this.blackMask = new Mat();
        this.yellowMask = new Mat();
        this.hierarchy = new Mat();
        this.detectorName = "Skystone Detector";
    }

    public Mat process(Mat input) {
        input.copyTo(this.rawImage);
        input.copyTo(this.workingMat);
        input.copyTo(this.displayMat);
        input.copyTo(this.blackMask);
        this.yellowFilter.process(this.workingMat.clone(), this.yellowMask);
        List<MatOfPoint> contoursYellow = new ArrayList();
        Imgproc.findContours(this.yellowMask, contoursYellow, this.hierarchy, 3, 2);
        Imgproc.drawContours(this.displayMat, contoursYellow, -1, new Scalar(255.0D, 30.0D, 30.0D), 2);
        Rect bestRect = this.foundRect;
        double bestDifference = 1.7976931348623157E308D;
        Iterator var6 = contoursYellow.iterator();
/*
        while(var6.hasNext()) {
            MatOfPoint cont = (MatOfPoint)var6.next();
            double score = this.calculateScore(cont);
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(this.displayMat, rect.tl(), rect.br(), new Scalar(0.0D, 0.0D, 255.0D), 2);
            if (score < bestDifference) {
                bestDifference = score;
                bestRect = rect;
            }
        }
*/
        Imgproc.rectangle(this.blackMask, bestRect.tl(), bestRect.br(), new Scalar(255.0D, 165.0, 0.0), 1, 4, 0);
        this.blackFilter.process(this.workingMat.clone(), this.blackMask);
        List<MatOfPoint> contoursBlack = new ArrayList();
        Imgproc.findContours(this.blackMask, contoursBlack, this.hierarchy, 3, 2);
        Imgproc.drawContours(this.displayMat, contoursBlack, -1, new Scalar(40.0D, 40.0D, 40.0D), 2);
        Iterator var13 = contoursBlack.iterator();

        // top y line
        Point topLeft = new Point(0, this.RequestedTopYLine);
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



        while(var13.hasNext()) {
            MatOfPoint cont = (MatOfPoint)var13.next();
            double score = this.calculateScore(cont);
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(this.displayMat, rect.tl(), rect.br(), new Scalar(0.0D, 0.0D, 255.0D), 2);
            if (score < bestDifference) {
                bestDifference = score;
                bestRect = rect;
            }
        }

        if (bestRect != null) {
            Imgproc.rectangle(this.displayMat, bestRect.tl(), bestRect.br(), new Scalar(255.0D, 0.0D, 0.0D), 4);
            Imgproc.putText(this.displayMat, "Chosen", bestRect.tl(), 0, 1.0D, new Scalar(255.0D, 255.0D, 255.0D));
            this.screenPosition = new Point((double)bestRect.x, (double)bestRect.y);
            this.foundRect = bestRect;
            this.found = true;
        } else {
            this.found = false;
        }

        switch(this.stageToRenderToViewport) {
            case THRESHOLD:
                Imgproc.cvtColor(this.blackMask, this.blackMask, 8);
                return this.blackMask;
            case RAW_IMAGE:
                return this.rawImage;
            default:
                return this.displayMat;
        }
    }

    public void useDefaults() {
        this.addScorer(this.ratioScorer);
        if (this.areaScoringMethod == AreaScoringMethod.MAX_AREA) {
            this.addScorer(this.maxAreaScorer);
        }

        if (this.areaScoringMethod == AreaScoringMethod.PERFECT_AREA) {
            this.addScorer(this.perfectAreaScorer);
        }

    }

    public void SetRequestedTopYLine(int y) { RequestedTopYLine = y;}
    public void SetRequestedBottomYLine (int y2) {RequestedBottomYLine = y2;}
    public void SetRequestedMidYLine (int y3) {RequestedMidYLine = y3;}
}
