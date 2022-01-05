package org.firstinspires.ftc.teamcode.visionpipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class DuckDetectionPipeline extends OpenCvPipeline
{

    public static int leftMean;
    public static int centerMean;
    public static int rightMean;

    Mat YCrCb = new Mat();

    private static volatile BarcodePos barcodePos = null;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        //Mat centerMat = new Mat();
        Mat rightMat = new Mat();

        //Core.extractChannel(YCrCb.submat(REGION_A), centerMat, COLOR_CHANNEL);
        Core.extractChannel(YCrCb.submat(REGION_B), rightMat, COLOR_CHANNEL);

        leftMean = 100;
        centerMean = (int) Core.mean(YCrCb.submat(REGION_A)).val[COLOR_CHANNEL];
        rightMean = (int) Core.mean(rightMat).val[0];
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
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);

        maxAverage();

        Mat outputFrame = new Mat();
        Core.extractChannel(YCrCb, outputFrame, COLOR_CHANNEL);

        Imgproc.rectangle(outputFrame, REGION_A, GREEN, 2);
        Imgproc.rectangle(outputFrame, REGION_B, GREEN, 2);

        return outputFrame;
    }

    public static BarcodePos getBarcodePos()
    {
        return barcodePos;
    }
}