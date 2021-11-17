package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Constants.*;

class DuckDetectionPipeline extends OpenCvPipeline
{

    private int leftMean;
    private int centerMean;
    private int rightMean;

    Mat YCrCb = new Mat();
    Mat Cb = new Mat();

    private volatile BarcodePos barcodePos = null;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Mat leftMat = new Mat();
        Mat centerMat = new Mat();
        Mat rightMat = new Mat();

        Core.extractChannel(YCrCb.submat(REGION_A), leftMat, 2);
        Core.extractChannel(YCrCb.submat(REGION_B), leftMat, 2);
        Core.extractChannel(YCrCb.submat(REGION_C), leftMat, 2);

        leftMean = (int) Core.mean(leftMat).val[0];
        centerMean = (int) Core.mean(centerMat).val[0];
        rightMean = (int) Core.mean(rightMat).val[0];
    }

    private void maxAverage() {
        int max = leftMean;
        barcodePos = BarcodePos.LEFT;
        if (centerMean > max) {
            max = centerMean;
            barcodePos = BarcodePos.CENTER;
        }
        if (rightMean > max) {
            barcodePos = BarcodePos.RIGHT;
        }
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

        maxAverage();

        Imgproc.rectangle(input, REGION_A, GREEN, 2);
        Imgproc.rectangle(input, REGION_B, GREEN, 2);
        Imgproc.rectangle(input, REGION_C, GREEN, 2);

        return input;
    }

    public BarcodePos getBarcodePos()
    {
        return barcodePos;
    }
}