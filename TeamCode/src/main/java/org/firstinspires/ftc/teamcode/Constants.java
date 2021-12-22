package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import androidx.annotation.FloatRange;

/**
 * Contains constants for all of the programs in one file for easy access.
 */

public class Constants {
    protected static final double TICKS_PER_FOOT = 758.51;
    protected static final double HALF_FIELD_DISTANCE = 1828.8;

    protected static final double CONTROLLER_TOLERANCE = 0.05;
    protected static final double ANGLE_ERROR_TOLERANCE = 0.05;
    protected static final double ENCODER_POSITION_TOLERANCE = 20.0;

    @FloatRange(from=0.0, to=1.0)
    protected static final double BRISTLES_POWER = 1;
    protected static final double DUCK_SPEED = -0.12;
    protected static final double AUTO_DUCK_SPEED = -.05;

    public enum BarcodePos {

        LEFT,
        CENTER,
        RIGHT

    }

    public enum AllianceColor {

        BLUE(1, 0),
        RED(-1, 0);

        public final int direction;
        protected final double angleOffset;

        AllianceColor(int direction, double angleOffset) {
            this.direction = direction;
            this.angleOffset = angleOffset;
        }

    }

    public enum CommandType {
        NONE,
        MOVE,
        TURN,
        WAIT,
        DETECT_IMAGE,
        RED_BLUE,
        DUCKS
    }

    protected static final String VUFORIA_KEY = "AaeF/Hb/////AAABmXyUA/dvl08Hn6O8IUco1axEjiRtYCVASe" +
            "XGzCnFiMaizR1b3cvD+SXpU1UHHbSpnyem0dMfGb6wce32IWKttH90xMTnLjY4aXBEYscpQbX/FzUi6uf5M+sXD" +
            "VNMtaVxLDGOb1phJ8tg9/Udb1cxIUCifI+AHmcwj3eknyY1ZapF81n/R0mVSmuyApS2oGQLnETWaWK+kxkx8cGn" +
            "Q0Nj7a79gStXqm97obOdzptw7PdDNqOfSLVcyKCegEO0zbGoInhRMDm0MPPTxwnBihZsjDuz+I5kDEZJZfBWZ9O" +
            "1PZMeFmhe6O8oFwE07nFVoclw7j2P6qHbsKTabg3w9w4ZdeTSZI4sV2t9OhbF13e0MWeV";

    protected static final float CAMERA_FORWARD_DISPLACEMENT = 57;
    protected static final float CAMERA_VERTICAL_DISPLACEMENT = 0;
    protected static final float CAMERA_LEFT_DISPLACEMENT = 184;
    protected static final float CAMERA_X_ROTATE = 0;
    protected static final float CAMERA_Y_ROTATE = 0;
    protected static final float CAMERA_Z_ROTATE = 0;

    protected static final int IMAGE_DETECTION_COUNT = 20;
    protected static final double DEFAULT_DISTANCE_FROM_IMAGE = 208.0;

    public static final Scalar GREEN = new Scalar(0, 255, 0);
    public static final int COLOR_CHANNEL = 2;
    public static final Size REGION_SIZE = new Size(175, 175);
    public static final Rect REGION_A = new Rect(new Point(65,695), REGION_SIZE);
    public static final Rect REGION_B = new Rect(new Point(725,695), REGION_SIZE);
    public static final int RESOLUTION_WIDTH = 1280;
    public static final int RESOLUTION_HEIGHT = 960;
    public static final String WEBCAM_RECORDING_FILE = "/Movies/match_recording.mp4";

    protected static final double TURNING_ANGLE_POSITION_SCALAR = 2.0;
    protected static final double TURNING_ENCODER_POSITION_SCALAR = 20.0;
    protected static final double TURNING_POWER_SCALAR = 4;

}
