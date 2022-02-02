package org.firstinspires.ftc.teamcode.visionpipelines;

import static org.firstinspires.ftc.teamcode.Constants.BarcodePos;
import static org.firstinspires.ftc.teamcode.Constants.COLOR_CHANNEL;
import static org.firstinspires.ftc.teamcode.Constants.YellowLowerBound;
import static org.firstinspires.ftc.teamcode.Constants.YellowUpperBound;

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

public class TestPipeline2 extends OpenCvPipeline
{

    public static int leftMean;
    public static int centerMean;
    public static int rightMean;
    private static Scalar maskedChannels;

    Mat hsv = new Mat();
    Mat thresh = new Mat();
    Mat edges = new Mat();
    Mat output = new Mat();

    private static volatile BarcodePos barcodePos = null;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        //Mat centerMat = new Mat();
        Core.inRange(hsv, YellowLowerBound, YellowUpperBound, thresh);
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(20,20)));
        Imgproc.Canny(thresh, edges, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        /*MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        for (Rect rect : boundRect) {
            Imgproc.rectangle(output, rect, new Scalar(0.5, 76.9, 89.8));
        }*/
    }

    private void maxAverage() {
        int min = leftMean;
        barcodePos = BarcodePos.LEFT;
        if (centerMean < min) {
            min = centerMean;
            barcodePos = BarcodePos.CENTER;
        }
        if (rightMean < min) {
            barcodePos = BarcodePos.RIGHT;
        }
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);
        if (COLOR_CHANNEL == 0) {
            maskedChannels = new Scalar(0, 1, 1);
        } else if (COLOR_CHANNEL == 1) {
            maskedChannels = new Scalar(1, 1, 0);
        } else {
            maskedChannels = new Scalar(1, 0, 1);
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no skystone

        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSV = new Scalar(20, 100, 100); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(30, 255, 255); // higher bound HSV for yellow
        Mat thresh = new Mat();

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, thresh);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);

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

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
        for (int i = 0; i != boundRect.length; i++) {

            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
            Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
        }

        // if there is no yellow regions on a side
        // that side should be a Skystone

        return mat; // return the mat with rectangles drawn
    }

    public static BarcodePos getBarcodePos()
    {
        return barcodePos;
    }
}