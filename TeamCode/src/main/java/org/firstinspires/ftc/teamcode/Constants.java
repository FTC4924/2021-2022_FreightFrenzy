package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import androidx.annotation.FloatRange;
import androidx.annotation.IntRange;

/**
 * Contains constants for all of the programs in one file for easy access.
 */

public class Constants {
    /**
     * Number of motor encoder ticks per foot.
     */
    protected static final double TICKS_PER_FOOT = 758.51;

    /**
     * Defines the dead zone for controller input.
     */
    @FloatRange(from=0.0, to=1.0)
    protected static final double CONTROLLER_TOLERANCE = 0.05;
    /**
     * Defines the tolerance for the angle error.
     */
    @FloatRange(from=0.0, to=1.0)
    protected static final double ANGLE_ERROR_TOLERANCE = 0.05;
    /**
     * How close the encoder needs to get to the target position for autonomous to move to the next command.
     */
    protected static final double ENCODER_POSITION_TOLERANCE = 100.0;

    protected static final double TURNING_ENCODER_POSITION_SCALAR = 20.0;
    protected static final double TURNING_POWER_SCALAR = 1;

    /**
     * The percent speed to run the bristles when pulling in.
     */
    @FloatRange(from=0.0, to=1.0)
    protected static final double BRISTLES_POWER_IN = 1;
    /**
     * The percent speed to run the bristles when expelling.
     */
    @FloatRange(from=0.0, to=1.0)
    protected static final double BRISTLES_POWER_OUT = .75;
    /**
     * The percent speed to run the duck wheel during TeleOp.
     */
    @FloatRange(from=0.0, to=1.0)
    protected static final double DUCK_SPEED = 0.22;
    /**
     * The percent speed to extend and retract the arm.
     */
    @FloatRange(from=0.0, to=1.0)
    protected static final double ARM_SPEED = 0.75;
    /**
     * The percent speed to run the duck wheel during autonomous.
     */
    @FloatRange(from=0.0, to=1.0)
    protected static final double AUTO_DUCK_SPEED = .14;
    /**
     * The maximum encoder position that the arm can extend to.
     */
    public static final double MAX_ARM_EXTENSION = -10500;
    /**
     * The maximum arm encoder position that the arm can rotate to in autonomous.
     */
    public static final int MAX_ARM_ROTATION = -4650;
    public static final int STARTING_ARM_POSITION = -1650;

    /**
     * An enum meaning whether the team shipping element is on the left, center, or right barcode position.
     */
    public enum BarcodePos {

        LEFT,
        CENTER,
        RIGHT

    }

    /**
     * An enum for the type of freight its corresponding weight.
     */
    public enum FreightType {
        NONE(-3.57e5),
        DUCK(-3.63e5),
        BALL(-3.70e5),
        LIGHT(-3.80e5),
        MEDIUM(-4.00e5),
        HEAVY(-4.18e5);

        /**
         * The weight of the type of freight.
         */
        double weight;

        /**
         * Construct a new type of freight.
         * @param weight The weight of this type of freight. Technically an int, but needs to be a double to work with java's scientific notation.
         */
        FreightType(double weight) {
            this.weight = weight;
        }
    }

    /**
     * An enum representing the color of the robot's current alliance and values that change based on the alliance.
     */
    public enum AllianceColor {

        BLUE(1, -90,.625, RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE),
        RED(-1, 90,.45, RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);

        /**
         * Used to reflect autonomous programs onto the other side of the field.
         */
        @IntRange(from=-1, to=1)
        public final int direction;
        /**
         * The offset of the starting angle to the field angle.
         */
        protected final double angleOffset;
        /**
         * The horizontal distance to the position to detect the barcode position.
         */
        public final double distanceToDucks;
        /**
         * The pattern for the led strip while idling.
         */
        protected final RevBlinkinLedDriver.BlinkinPattern pattern;

        /**
         * Construct an alliance.
         * @param direction See {@linkplain AllianceColor#direction}.
         * @param angleOffset See {@linkplain AllianceColor#angleOffset}.
         * @param distanceToDucks See {@linkplain AllianceColor#distanceToDucks}.
         * @param pattern See {@linkplain AllianceColor#pattern}.
         */
        AllianceColor(@IntRange(from=-1, to=1) int direction,
                      double angleOffset,
                      double distanceToDucks,
                      RevBlinkinLedDriver.BlinkinPattern pattern) {
            this.direction = direction;
            this.angleOffset = Math.toRadians(angleOffset);
            this.distanceToDucks = distanceToDucks;
            this.pattern = pattern;
        }

    }

    /*
    All the constants below are part of image processing.
     */
    public static final Scalar GREEN = new Scalar(0, 255, 0);
    public static final Scalar YellowLowerBound = new Scalar(20, 100, 100);
    public static final Scalar YellowUpperBound = new Scalar(30, 255, 255);
    public static final int COLOR_CHANNEL = 2;
    public static final Size REGION_SIZE = new Size(335, 175);
    public static final Rect REGION_A = new Rect(new Point(0,665), REGION_SIZE);
    public static final Rect REGION_B = new Rect(new Point(625,665), REGION_SIZE);
    public static final int RESOLUTION_WIDTH = 1280;
    public static final int RESOLUTION_HEIGHT = 960;
    public static final int SHIPPING_ELEMENT_WIDTH = 15000;
    public static final String WEBCAM_RECORDING_FILE = "/Movies/match_recording.mp4";

}
