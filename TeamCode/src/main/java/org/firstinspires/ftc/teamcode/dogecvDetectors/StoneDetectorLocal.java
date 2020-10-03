//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package org.firstinspires.ftc.teamcode.dogecvDetectors;

import com.disnodeteam.dogecv.DogeCV.AreaScoringMethod;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.HSVColorFilter;
import com.disnodeteam.dogecv.filters.HSVRangeFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter.ColorPreset;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class StoneDetectorLocal extends DogeCVDetector {
    public AreaScoringMethod areaScoringMethod;
    public DogeCVColorFilter filter;
    public int stonesToFind;
    public RatioScorer ratioScorerForShortFace;
    public RatioScorer ratioScorerForLongFace;
    public MaxAreaScorer maxAreaScorer;
    public PerfectAreaScorer perfectAreaScorer;
    private ArrayList<Point> screenPositions;
    private ArrayList<Rect> foundRects;
    private Mat rawImage;
    private Mat workingMat;
    private Mat displayMat;
    private Mat yellowMask;
    private Mat hierarchy;

    public List<Point> foundScreenPositions() {
        return this.screenPositions;
    }

    public List<Rect> foundRectangles() {
        return this.foundRects;
    }

    public StoneDetectorLocal() {
        this.areaScoringMethod = AreaScoringMethod.MAX_AREA;
        //this.filter = new LeviColorFilter(ColorPreset.YELLOW, 10.0D);

        //Scalar upper = new Scalar(25, 60, 90); // lighter orange
        //Scalar lower = new Scalar(35, 100, 110); // darker orange

        Scalar upper = new Scalar(20, 60, 80); // lighter orange
        Scalar lower = new Scalar(40, 100, 120); // darker orange

        this.filter  = new HSVRangeFilter(lower, upper);
        //this.filter  = new HSVRangeFilter(upper, lower);

        this.stonesToFind = 1;
        //this.ratioScorerForShortFace = new RatioScorer(1.0D, 3.0D);
        //this.ratioScorerForLongFace = new RatioScorer(1.0D, 3.0D);
        this.maxAreaScorer = new MaxAreaScorer(5.0D);
        this.perfectAreaScorer = new PerfectAreaScorer(5000.0D, 0.05D);
        this.screenPositions = new ArrayList();
        this.foundRects = new ArrayList();
        this.rawImage = new Mat();
        this.workingMat = new Mat();
        this.displayMat = new Mat();
        this.yellowMask = new Mat();
        this.hierarchy = new Mat();
        this.detectorName = "Stone Detector";
    }

    public Mat process(Mat input) {
        this.screenPositions.clear();
        this.foundRects.clear();
        input.copyTo(this.rawImage);
        input.copyTo(this.workingMat);
        input.copyTo(this.displayMat);
        input.copyTo(this.yellowMask);
        this.filter.process(this.workingMat.clone(), this.yellowMask);
        List<MatOfPoint> contoursYellow = new ArrayList();
        Imgproc.findContours(this.yellowMask, contoursYellow, this.hierarchy, 3, 2);
        //Imgproc.findContours(this.yellowMask, contoursYellow, this.hierarchy,);

        Imgproc.drawContours(this.displayMat, contoursYellow, -1, new Scalar(230.0D, 70.0D, 70.0D), 2);
        new ArrayList();
        double bestDifference = 1.7976931348623157E308D;
        Collections.sort(contoursYellow, new Comparator<MatOfPoint>() {
            public int compare(MatOfPoint matOfPoint, MatOfPoint t1) {
                return StoneDetectorLocal.this.calculateScore(matOfPoint) > StoneDetectorLocal.this.calculateScore(t1) ? 1 : 0;
            }
        });
        List<MatOfPoint> subList = contoursYellow;
        if (contoursYellow.size() > this.stonesToFind) {
            subList = contoursYellow.subList(0, this.stonesToFind);
        }

        Iterator var7 = ((List)subList).iterator();

        while(var7.hasNext()) {
            MatOfPoint contour = (MatOfPoint)var7.next();
            Rect rect = Imgproc.boundingRect(contour);
            Imgproc.rectangle(this.displayMat, rect.tl(), rect.br(), new Scalar(255.0D, 0.0D, 0.0D), 4);
            Imgproc.putText(this.displayMat, "Chosen", rect.tl(), 0, 1.0D, new Scalar(255.0D, 255.0D, 255.0D));
            this.screenPositions.add(new Point((double)rect.x, (double)rect.y));
            this.foundRects.add(rect);
        }

        if (this.foundRects.size() > 0) {
            this.found = true;
        } else {
            this.found = false;
        }

        switch(this.stageToRenderToViewport) {
            case THRESHOLD:
                Imgproc.cvtColor(this.yellowMask, this.yellowMask, 8);
                return this.yellowMask;
            case RAW_IMAGE:
                return this.rawImage;
            default:
                return this.displayMat;
        }
    }

    public void useDefaults() {
        //this.addScorer(this.ratioScorerForShortFace);
        //this.addScorer(this.ratioScorerForLongFace);
        if (this.areaScoringMethod == AreaScoringMethod.MAX_AREA) {
            this.addScorer(this.maxAreaScorer);
        }

        if (this.areaScoringMethod == AreaScoringMethod.PERFECT_AREA) {
            this.addScorer(this.perfectAreaScorer);
        }

    }
}
