package org.firstinspires.ftc.teamcode;

/**
 * Contains constants for all of the programs in one file for easy access.
 */

public class Constants {
    protected static final double TICKS_PER_FOOT = 759.12;
    protected static final double HALF_FIELD_DISTANCE = 1828.8;

    protected static final double CONTROLLER_TOLERANCE = 0.05;
    protected static final double ANGLE_ERROR_TOLERANCE = 0.05;
    protected static final double ENCODER_POSITION_TOLERANCE = 20.0;

    protected enum AllianceColor {

        BLUE(1, -1 * Math.PI/2, 914.0),
        RED(-1, Math.PI/2, 300.0);

        protected final int direction;
        protected final double angleOffset;
        protected final double highGoalX;

        AllianceColor(int direction, double angleOffset, double highGoalX) {
            this.direction = direction;
            this.angleOffset = angleOffset;
            this.highGoalX = highGoalX;
        }

    }

    protected enum CommandType {

        MOVE,
        TURN,
        ELEVATOR,
        WAIT,
        SHOOTER_REV,
        DETECT_IMAGE,
        KICKER,
        DETECT_RING_NUMBER

    }

    public enum RingNumber {

        NONE,
        ONE,
        FOUR

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

    protected static double REGION_WIDTH = 60;
    protected static double REGION_HEIGHT = 60;
    protected static int RESOLUTION_WIDTH = 1280;

    protected static final double TURNING_ANGLE_POSITION_SCALAR = 2.0;
    protected static final double TURNING_POWER_SCALAR = 2.0;

    protected static final double FUNNEL_DOWN = 0.0;
    protected static final double FUNNEL_UP = 0.0;

    protected static final double BRISTLES_DEFAULT_POWER = 0.6;

    protected enum ElevatorPositions {
        /* For a go-BILDA Torque servo, the values are about 0.95, 0.65, 0.555, 0.5425, 0.5, and 0.48 respectively
        go-BILDA servos accept PWM range of 500us-2500us with increasing PWM going clockwise
        This corresponds to a range in the software of 0-1
        Changed to a SAVOX super torque servo with a PWM range of 800-2200 and increasing PWM goes counter-clockwise
        This corresponds to a range in the software of 0.15-0.85, so don't try to send it outside that range
        So through MATH our values for SAVOX should be 0.15 for 0 degrees, 0.544 for 90 degrees, 1 for 160 degrees
        On 6/26/21, we configured as 0.19 = down; 0.45 middle; 0.56 ring 1; 0.60 ring 2; 0.63 ring 3; 0.68 UNJAM
        */
        DOWN(0.15),  // 0 degrees = 0.15, 2.2 degrees per .01
        MIDDLE(0.45),  //45 degrees and some
        RING_ONE(0.54),
        RING_TWO(0.575),
        RING_THREE(0.62),
        UNJAM(0.66); //Requested by Serena, added by Coach Ethan 1/19/2021. Corresponds to full up, 90 degrees
        // Arnit changed the RING_THREE From 0.595 to 62 and made UNJAM from 0.62 to 0.66 2/23/2021 5:04PM
        protected final double positionValue;

        ElevatorPositions(double positionValue) { this.positionValue = positionValue; }

    }
    protected static final double ELEVATOR_MOVE_TIME = 2.0;

    protected static final double KICKER_POSITION_SCALAR = 0.3;
    protected static final double KICKER_KICK_POSITION = 0.8;

    protected static final double SHOOTER_LIFTER_MAX_POSITION = 0.8;
    protected static final double SHOOTER_LIFTER_MIN_POSITION = 0.2;
    protected static final double SHOOTER_LIFTER_DEFAULT_POSITION = 0.42;
    protected static final double SHOOTER_LIFTER_REDUCTION = .005;

    protected static final double SHOOTER_POWER = .70;

}
