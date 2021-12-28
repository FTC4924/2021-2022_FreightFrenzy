package org.firstinspires.ftc.teamcode.visionpipelines;

import android.util.Log;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.PipelineRecordingParameters;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.WEBCAM_RECORDING_FILE;

public class CameraRecordingPipeline extends OpenCvPipeline {

    public static OpenCvCamera webcam;

    @Override
    public void init(Mat mat) {
        try {
            PipelineRecordingParameters parameters = new PipelineRecordingParameters.Builder()
                    .setBitrate(4, PipelineRecordingParameters.BitrateUnits.Mbps)
                    .setEncoder(PipelineRecordingParameters.Encoder.H264)
                    .setOutputFormat(PipelineRecordingParameters.OutputFormat.MPEG_4)
                    .setFrameRate(30)
                    .setPath(WEBCAM_RECORDING_FILE)
                    .build();
            //webcam.startRecordingPipeline(parameters);
            webcam.showFpsMeterOnViewport(true);

        } catch (OpenCvCameraException e) {
            Log.e("Camera Pipeline", Arrays.toString(e.getStackTrace()));
            throw new OpenCvCameraException(Arrays.toString(e.getStackTrace()) + ":  " + e.getMessage());
        }
    }

    public Mat processFrame(Mat input) {
        return input;
    }
}
