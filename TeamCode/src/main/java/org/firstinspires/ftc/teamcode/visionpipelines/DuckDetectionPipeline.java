package org.firstinspires.ftc.teamcode.visionpipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class DuckDetectionPipeline extends OpenCvPipeline
{

    public static int leftMean;
    public static int centerMean;
    public static int rightMean;
    private static Scalar maskedChannels;

    Mat YCrCb = new Mat();

    private static volatile BarcodePos barcodePos = null;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        //Mat centerMat = new Mat();
        Mat rightMat = new Mat();

        //Core.extractChannel(YCrCb.submat(REGION_A), centerMat, COLOR_CHANNEL);
        Core.extractChannel(YCrCb.submat(REGION_B), rightMat, COLOR_CHANNEL);

        leftMean = 118;
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
        if (COLOR_CHANNEL == 0) {
            maskedChannels = new Scalar(1, 0, 0);
        } else if (COLOR_CHANNEL == 1) {
            maskedChannels = new Scalar(0, 1, 0);
        } else {
            maskedChannels = new Scalar(0, 0, 1);
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);

        maxAverage();

        Mat outputFrame = new Mat();
        Core.multiply(YCrCb, maskedChannels, outputFrame);

        Imgproc.cvtColor(outputFrame, outputFrame, Imgproc.COLOR_YCrCb2RGB);

        Imgproc.rectangle(outputFrame, REGION_A, GREEN, 2);
        Imgproc.rectangle(outputFrame, REGION_B, GREEN, 2);

        return outputFrame;
    }

    public static BarcodePos getBarcodePos()
    {
        return barcodePos;
    }
}