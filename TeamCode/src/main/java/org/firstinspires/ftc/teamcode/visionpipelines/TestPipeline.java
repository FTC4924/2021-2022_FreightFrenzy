package org.firstinspires.ftc.teamcode.visionpipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Constants.BarcodePos;
import static org.firstinspires.ftc.teamcode.Constants.GREEN;
import static org.firstinspires.ftc.teamcode.Constants.RESOLUTION_HEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.ROI;
import static org.firstinspires.ftc.teamcode.Constants.SHIPPING_ELEMENT_WIDTH;
import static org.firstinspires.ftc.teamcode.Constants.YellowLowerBound;
import static org.firstinspires.ftc.teamcode.Constants.YellowUpperBound;

public class TestPipeline extends OpenCvPipeline
{
    public static double boxArea;
    public static Rect largestRect = new Rect(0,0,0,0);

    public static int leftMean;
    public static int centerMean;
    public static int rightMean;
    private static Scalar maskedChannels;

    Mat cropped = new Mat();
    Mat hsv = new Mat();
    Mat thresh = new Mat();
    Mat open = new Mat();
    Mat dilate = new Mat();
    Mat edges = new Mat();
    Mat output = new Mat();

    private static volatile BarcodePos barcodePos = null;

    void findBarcodePosition(Mat input)
    {
        cropped = input.submat(ROI);
        output = cropped;
        Imgproc.cvtColor(cropped, hsv, Imgproc.COLOR_RGB2HSV);

        //Mat centerMat = new Mat();
        Core.inRange(hsv, YellowLowerBound, YellowUpperBound, thresh);
        Imgproc.morphologyEx(thresh, open, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10,10)));
        Imgproc.morphologyEx(open, dilate, Imgproc.MORPH_DILATE, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(30,30)));
        Imgproc.Canny(dilate, edges, 150, 250);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        largestRect = new Rect(0,0,0,0);
        if (boundRect.length > 0) {
            for (Rect rect : boundRect) {
                if(rect.area() > largestRect.area()) {
                    largestRect = rect;
                }
            }
            if(largestRect.area() > SHIPPING_ELEMENT_WIDTH) {
                if(largestRect.x < RESOLUTION_HEIGHT / 2 - largestRect.width / 2) {
                    barcodePos = BarcodePos.CENTER;
                } else {
                    barcodePos = BarcodePos.RIGHT;
                }
            } else {
                barcodePos = BarcodePos.LEFT;
            }
            Imgproc.rectangle(output, largestRect, GREEN, 5);
        } else {
            barcodePos = BarcodePos.LEFT;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        findBarcodePosition(input);

        return edges;
    }

    public static BarcodePos getBarcodePos() {
        return barcodePos;
    }

    public static Rect getLargestRect() {
        return largestRect;
    }
}