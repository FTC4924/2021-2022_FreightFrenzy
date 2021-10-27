package org.firstinspires.ftc.teamcode;

/**
 * Contains constants for all of the programs in one file for easy access.
 */

public class Constants {
    protected static final double TICKS_PER_FOOT = 6446.96;
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
        WAIT,
        DETECT_IMAGE
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
    protected static final double TURNING_ENCODER_POSITION_SCALAR = 20.0;
    protected static final double TURNING_POWER_SCALAR = 2.0;

}
