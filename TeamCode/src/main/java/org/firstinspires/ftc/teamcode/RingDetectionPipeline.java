package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Constants.*;

class RingDetectionPipeline extends OpenCvPipeline
{
    static final int REGION_WIDTH = 60;
    static final int REGION_HEIGHT = 60;

    static final Point REGION1_TOP_LEFT = new Point(10,245);
    static final Point REGION1_BOTTOM_RIGHT = new Point(
            REGION1_TOP_LEFT.x + REGION_WIDTH,
            REGION1_TOP_LEFT.y + REGION_HEIGHT);

    static final Point REGION2_TOP_LEFT = new Point(60,245);
    static final Point REGION2_BOTTOM_RIGHT = new Point(
            REGION1_TOP_LEFT.x + REGION_WIDTH,
            REGION1_TOP_LEFT.y + REGION_HEIGHT);

    static final Point REGION3_TOP_LEFT = new Point(110,245);
    static final Point REGION3_BOTTOM_RIGHT = new Point(
            REGION1_TOP_LEFT.x + REGION_WIDTH,
            REGION1_TOP_LEFT.y + REGION_HEIGHT);

    Mat region1_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    protected int avg1;

    private volatile RingNumber ringNumber = null;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame)
    {
        inputToCb(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];

        if(avg1 > 120) {
            ringNumber = RingNumber.NONE;
        } else if (avg1 > 110) {
            ringNumber = RingNumber.ONE;
        } else {
            ringNumber = RingNumber.FOUR;
        }

        Imgproc.rectangle(
                input, // Buffer to draw on
                REGION1_TOP_LEFT, // First point which defines the rectangle
                REGION1_BOTTOM_RIGHT, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        return input;
    }

    public RingNumber getRingNumber()
    {
        return ringNumber;
    }
}