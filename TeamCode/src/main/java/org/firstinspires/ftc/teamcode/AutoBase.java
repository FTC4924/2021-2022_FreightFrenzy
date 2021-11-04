package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.Constants.ANGLE_ERROR_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Constants.AllianceColor;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_FORWARD_DISPLACEMENT;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_LEFT_DISPLACEMENT;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_VERTICAL_DISPLACEMENT;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_X_ROTATE;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_Y_ROTATE;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_Z_ROTATE;
import static org.firstinspires.ftc.teamcode.Constants.DEFAULT_DISTANCE_FROM_IMAGE;
import static org.firstinspires.ftc.teamcode.Constants.ENCODER_POSITION_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Constants.HALF_FIELD_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.IMAGE_DETECTION_COUNT;
import static org.firstinspires.ftc.teamcode.Constants.TICKS_PER_FOOT;
import static org.firstinspires.ftc.teamcode.Constants.TURNING_POWER_SCALAR;
import static org.firstinspires.ftc.teamcode.Constants.TURNING_ENCODER_POSITION_SCALAR;
import static org.firstinspires.ftc.teamcode.Constants.VUFORIA_KEY;


/**
 * The base that all of the autonomous programs run off of.
 */

public abstract class AutoBase extends OpMode {

    protected static AllianceColor allianceColor;
    private ArrayList<Command> currentCommands;
    private ArrayList<Command> newCommands;
    private ArrayList<ArrayList<Command>> upstreamCommands;
    private Command currentCommand;
    private int currentCommandIndex;
    private ArrayList<Integer> upstreamCommandIndexes;
    private boolean commandFirstLoop;
    private int count;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private double targetPosition;
    private double leftFrontTargetPosition;
    private double leftBackTargetPosition;
    private double rightFrontTargetPosition;
    private double rightBackTargetPosition;

    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;

    private BNO055IMU imu;
    private Orientation angles;

    private double currentRobotAngle;
    private double targetAngle;
    private double angleError;

    private OpenGLMatrix robotFromCamera;
    private OpenGLMatrix robotLocationTransform;
    private OpenGLMatrix lastLocation;

    private int cameraMonitorViewId;

    private WebcamName webcam1;
    private VuforiaLocalizer.Parameters vuforiaParameters;
    private VuforiaLocalizer vuforia;

    private VuforiaTrackables targetsUltimateGoal;
    private List<VuforiaTrackable> allTrackables;

    private boolean targetVisible;
    private double distanceFromImage;

    public void init() {
        allianceColor = getAllianceColor();
        currentCommands = getCommands();
        newCommands = null;
        upstreamCommands = new ArrayList<>();
        currentCommand = currentCommands.get(0);
        currentCommandIndex = 0;
        upstreamCommandIndexes = new ArrayList<>();
        commandFirstLoop = true;
        count = 0;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontPower = 0;
        leftBackPower = 0;
        rightFrontPower = 0;
        rightBackPower = 0;

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);
        angles = null;

        currentRobotAngle = 0.0;
        targetAngle = 0.0;
        angleError = 0.0;

        /*robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, RADIANS, CAMERA_X_ROTATE, CAMERA_Y_ROTATE, CAMERA_Z_ROTATE));
        lastLocation = null;
        robotLocationTransform = null;

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforiaParameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforiaParameters.cameraName = webcam1;
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);

        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        allTrackables = new ArrayList<>();
        allTrackables.addAll(targetsUltimateGoal);

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(robotFromCamera, vuforiaParameters.cameraDirection);
        }

        targetsUltimateGoal.activate();

        targetVisible = false;
        distanceFromImage = 0.0;*/
    }

    public void start() {
        resetStartTime();
    }

    public void loop() {
        getAngleError();

        switch (currentCommand.commandType) {
            case MOVE:
                holonomicDrive();
                break;

            case TURN:
                turn();
                break;

            case DETECT_IMAGE:
                detectImage();
                break;

            case WAIT:
                pause();
                break;
        }

        gyroCorrection();

        setWheelPowersAndPositions();
    }

    /**
     * Calculates the holonomic drive motor target encoder positions and sets the motor speeds.
     */
    public void holonomicDrive() {
        /*Determines what encoder position each wheel should be based on the angle we get from the input
        plus the current robot angle so that the controls are independent of what direction the
        robot is facing*/
        /*if (commandFirstLoop) {

            targetPosition = currentCommand.distance * TICKS_PER_FOOT;

            double leftTargetPositionCalculation = targetPosition * Math.cos(currentCommand.angle + (Math.PI / 4) - currentRobotAngle);
            double rightTargetPositionCalculation = targetPosition * Math.sin(currentCommand.angle + (Math.PI / 4) - currentRobotAngle);

            leftFrontTargetPosition += (leftTargetPositionCalculation * -1);
            leftBackTargetPosition += (rightTargetPositionCalculation * -1);
            rightFrontTargetPosition += (rightTargetPositionCalculation);
            rightBackTargetPosition += (leftTargetPositionCalculation);

            commandFirstLoop = false;
        }*/


        /*Determines what power each wheel should get based on the angle we get from the stick
        plus the current robot angle so that the controls are independent of what direction the
        robot is facing*/
        leftFrontPower = Math.cos(currentCommand.angle + (Math.PI/4) - currentRobotAngle)*-1;
        leftBackPower = Math.sin(currentCommand.angle + (Math.PI/4) - currentRobotAngle)*-1;
        rightFrontPower = Math.sin(currentCommand.angle + (Math.PI/4) - currentRobotAngle);
        rightBackPower = Math.cos(currentCommand.angle + (Math.PI/4) - currentRobotAngle);

        // Adjusts motor speed.
        leftFrontPower *= currentCommand.power;
        leftBackPower *= currentCommand.power;
        rightFrontPower *= currentCommand.power;
        rightBackPower *= currentCommand.power;

        if (time > currentCommand.distance) {

            startNextCommand();
        }
    }

    /**
     * Turns using the encoder positions.
      */
    private void turn() {
        if(commandFirstLoop) {
            commandFirstLoop = false;
            targetAngle = currentCommand.angle;
            angleError = targetAngle - currentRobotAngle;
            angleError = ((((angleError - Math.PI) % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI)) - Math.PI;
        }
        if(Math.abs(angleError) < ANGLE_ERROR_TOLERANCE) {
            startNextCommand();
        }
    }

    /**
     * Aims the robot at the target using a navigation image.
     */
    private void detectImage() {
        if(count < IMAGE_DETECTION_COUNT) {
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    targetVisible = true;

                    robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }

                    break;
                }
            }
            if (targetVisible) {
                distanceFromImage += (lastLocation.getTranslation().get(2));
            } else {
                distanceFromImage += DEFAULT_DISTANCE_FROM_IMAGE;
            }
            count ++;
        } else {
            distanceFromImage /= IMAGE_DETECTION_COUNT;

            startNextCommand();
        }
    }

    /**
     * Waits for a time. Could not use wait() because it is used in the base Java language.
     */
    private void pause() {
        if(time > currentCommand.duration) {
            startNextCommand();
        }
    }

    /**
     * Corrects the target positions and powers of the wheels based on the angle error.
     */
    private void gyroCorrection() {
        leftFrontTargetPosition += angleError * TURNING_ENCODER_POSITION_SCALAR;
        leftBackTargetPosition += angleError * TURNING_ENCODER_POSITION_SCALAR;
        rightFrontTargetPosition += angleError * TURNING_ENCODER_POSITION_SCALAR;
        rightBackTargetPosition += angleError * TURNING_ENCODER_POSITION_SCALAR;

        leftFrontPower += angleError * TURNING_POWER_SCALAR;
        leftBackPower += angleError * TURNING_POWER_SCALAR;
        rightFrontPower += angleError * TURNING_POWER_SCALAR;
        rightBackPower += angleError * TURNING_POWER_SCALAR;
    }

    /**
     * Sets the power and position of each wheel
     */
    private void setWheelPowersAndPositions() {
        leftFront.setTargetPosition((int) Math.round(leftFrontTargetPosition));
        leftBack.setTargetPosition((int) Math.round(leftBackTargetPosition));
        rightFront.setTargetPosition((int) Math.round(rightFrontTargetPosition));
        rightBack.setTargetPosition((int) Math.round(rightBackTargetPosition));

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    /**
     * Returns the error between the angle the gyroscope sensor reads, and the target angle.
     */
    private void getAngleError() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, RADIANS);
        //currentRobotAngle = angles.firstAngle;
        currentRobotAngle = angles.firstAngle + allianceColor.angleOffset;
        angleError = targetAngle - currentRobotAngle;
        angleError = ((((angleError - Math.PI) % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI)) - Math.PI;
    }

    /**
     * If there are new commands, saves the current commands, and replaces them with the new ones.
     */
    private void newCommands() {
        if(newCommands != null) {
            upstreamCommands.add(0, currentCommands);
            upstreamCommandIndexes.add(0, currentCommandIndex);
            currentCommands = newCommands;
            currentCommandIndex = 0;
            newCommands = null;
        }
    }

    /**
     * Starts the next command in the sequence. If there are no commands left, checks if there are
     * saved commands and goes through them in a first in last out sequence.
     */
    private void startNextCommand() {
        currentCommandIndex++;
        newCommands();
        if (currentCommandIndex < currentCommands.size()) {
            currentCommand = currentCommands.get(currentCommandIndex);
            leftFrontPower = 0;
            leftBackPower = 0;
            rightFrontPower = 0;
            rightBackPower = 0;
            commandFirstLoop = true;
            resetStartTime();
        } else if (upstreamCommands.size() != 0) {
            currentCommands = upstreamCommands.get(0);
            upstreamCommands.remove(0);
            currentCommandIndex = upstreamCommandIndexes.get(0);
            upstreamCommandIndexes.remove(0);

            if (currentCommandIndex < currentCommands.size()) {
                currentCommand = currentCommands.get(currentCommandIndex);
                leftFrontPower = 0;
                leftBackPower = 0;
                rightFrontPower = 0;
                rightBackPower = 0;
                commandFirstLoop = true;
                resetStartTime();
            }
        }
    }

    protected abstract AllianceColor getAllianceColor();
    protected abstract ArrayList<Command> getCommands();

}