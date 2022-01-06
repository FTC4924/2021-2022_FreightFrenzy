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

    @FloatRange(from=0.0, to=1.0)
    protected static final double CONTROLLER_TOLERANCE = 0.05;
    @FloatRange(from=0.0, to=1.0)
    protected static final double ANGLE_ERROR_TOLERANCE = 0.05;
    protected static final double ENCODER_POSITION_TOLERANCE = 20.0;

    protected static final double TURNING_ENCODER_POSITION_SCALAR = 20.0;
    protected static final double TURNING_POWER_SCALAR = 1;

    @FloatRange(from=0.0, to=1.0)
    protected static final double BRISTLES_POWER_IN = 1;
    @FloatRange(from=0.0, to=1.0)
    protected static final double BRISTLES_POWER_OUT = .75;
    @FloatRange(from=0.0, to=1.0)
    protected static final double DUCK_SPEED = 0.22;
    @FloatRange(from=0.0, to=1.0)
    protected static final double AUTO_DUCK_SPEED = .1;
    public static final double MAX_ARM_EXTENSION = -7500;
    public static final double MAX_ARM_ROTATION = -7500;

    public enum BarcodePos {

        LEFT,
        CENTER,
        RIGHT

    }

    public enum AllianceColor {

        BLUE(1, 0,.6),
        RED(-1, 0,.45);

        public final int direction;
        protected final double angleOffset;
        public final double distanceToDucks;

        AllianceColor(int direction, double angleOffset, double distanceToDucks) {
            this.direction = direction;
            this.angleOffset = angleOffset;
            this.distanceToDucks = distanceToDucks;
        }

    }

    public static final Scalar GREEN = new Scalar(0, 255, 0);
    public static final int COLOR_CHANNEL = 2;
    public static final Size REGION_SIZE = new Size(175, 175);
    public static final Rect REGION_A = new Rect(new Point(65,695), REGION_SIZE);
    public static final Rect REGION_B = new Rect(new Point(725,695), REGION_SIZE);
    public static final int RESOLUTION_WIDTH = 1280;
    public static final int RESOLUTION_HEIGHT = 960;
    public static final String WEBCAM_RECORDING_FILE = "/Movies/match_recording.mp4";

}
