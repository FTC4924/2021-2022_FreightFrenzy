package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.visionpipelines.DuckDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Constants.*;


/**
 * The base that all of the autonomous programs run off of.
 */

public abstract class AutoBase extends OpMode {

    protected static AllianceColor allianceColor;

    private ArrayList<Command> currentCommands;
    private ArrayList<ArrayList<Command>> upstreamCommands;
    private Command currentCommand;
    private boolean commandFirstLoop;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private Servo duckServo;

    private double targetPosition;
    private double leftFrontTargetPosition;
    private double leftBackTargetPosition;
    private double rightFrontTargetPosition;
    private double rightBackTargetPosition;

    private double leftFrontPower;
    private double leftBackPower;
    private double rightFrontPower;
    private double rightBackPower;

    private DcMotor armRotator;
    private DcMotor armExtender;

    private BNO055IMU imu;
    private Orientation angles;

    private double currentRobotAngle;
    private double targetAngle;
    private double angleError;

    private OpenCvWebcam webcam;

    private boolean ducksOn;

    private BarcodePos barcodePos;

    public void init() {
        allianceColor = getAllianceColor();
        currentCommands = getCommands();
        upstreamCommands = new ArrayList<>();
        currentCommand = currentCommands.get(0);
        commandFirstLoop = true;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setTargetPosition(0);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setTargetPosition(0);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setTargetPosition(0);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setTargetPosition(0);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armRotator = hardwareMap.get(DcMotor.class, "armRotator");
        armExtender = hardwareMap.get(DcMotor.class, "armExtender");
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        duckServo = hardwareMap.get(Servo.class, "duckServo");

        targetPosition = 0.0;
        leftFrontTargetPosition = 0.0;
        leftBackTargetPosition = 0.0;
        rightFrontTargetPosition = 0.0;
        rightBackTargetPosition = 0.0;

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new DuckDetectionPipeline());
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(RESOLUTION_WIDTH, RESOLUTION_HEIGHT, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                telemetry.addData("Webcam", "Setup Finished");
            }
           
            public void onError(int errorCode)
            {
                telemetry.speak("The web cam wasn't initialised correctly! Error code: " + errorCode);
                telemetry.addData("Webcam", "Setup Failed! Error code: " + errorCode);
            }
        });
    }

    public void start() {
        resetStartTime();
    }

    public void loop() {
        switch (currentCommand.getClass().getName()) {
            case "Move":
                holonomicDrive();
                break;

            case "Turn":
                turn();
                break;

            case "BlueRed":
                blueOrRed();
                break;

            case "Wait":
                pause();
                break;

            case "ArmExtend":
                armExtend();
                break;

            case "ArmRotate":
                armRotate();
                break;

            case "Ducks":
                ducks();
                break;

            case "DetectDuckPosition":
                detectDuckPosition();
                break;

            case "LoadDuckCommands":
                loadDuckCommands();
                break;
        }

        gyroCorrection();

        setWheelPowersAndPositions();

        telemetry.addData("centerColor", DuckDetectionPipeline.centerMean);
        telemetry.addData("rightColor", DuckDetectionPipeline.rightMean);
        telemetry.addData("barcodePos", DuckDetectionPipeline.getBarcodePos());

        if(commandFirstLoop)
            commandFirstLoop = false;
    }

    private void armRotate() {
        armRotator.setTargetPosition(currentCommand.position);
    }

    private void armExtend() {
        armExtender.setTargetPosition(currentCommand.position);
    }

    private void loadDuckCommands() {
        switch (barcodePos) {
            case LEFT:
                newCommands(currentCommand.leftCommands);
                break;

            case CENTER:
                newCommands(currentCommand.centerCommands);
                break;

            case RIGHT:
                newCommands(currentCommand.rightCommands);
                break;
        }
    }

    private void blueOrRed() {
        switch (allianceColor) {
            case BLUE:
                newCommands(currentCommand.blueCommands);
                break;
            case RED:
                newCommands(currentCommand.redCommands);
                break;
        }
        startNextCommand();
    }

    /**
     * Controls the duck wheel.
     */
    public void ducks() {
        ducksOn = !ducksOn;
        if (ducksOn) {
            duckServo.setPosition((0.5 - AUTO_DUCK_SPEED * allianceColor.direction) / 2);
        } else {
            duckServo.setPosition(.5);
        }
        startNextCommand();
    }

    /**
     * Calculates the holonomic drive motor target encoder positions and sets the motor speeds.
     */
    public void holonomicDrive() {
        if (commandFirstLoop) {

            targetPosition = currentCommand.distance * TICKS_PER_FOOT;

            double leftTargetPositionCalculation = targetPosition * Math.cos(currentCommand.angle * allianceColor.direction + (Math.PI / 4) - currentRobotAngle);
            double rightTargetPositionCalculation = targetPosition * Math.sin(currentCommand.angle * allianceColor.direction + (Math.PI / 4) - currentRobotAngle);

            leftFrontTargetPosition += (leftTargetPositionCalculation * -1);
            leftBackTargetPosition += (rightTargetPositionCalculation * -1);
            rightFrontTargetPosition += (rightTargetPositionCalculation);
            rightBackTargetPosition += (leftTargetPositionCalculation);

            commandFirstLoop = false;
        }

        /*Determines what power each wheel should get based on the angle we get from the stick
        plus the current robot angle so that the controls are independent of what direction the
        robot is facing*/
        leftFrontPower = Math.cos(currentCommand.angle * allianceColor.direction + (Math.PI/4) - currentRobotAngle)*-1;
        leftBackPower = Math.sin(currentCommand.angle * allianceColor.direction + (Math.PI/4) - currentRobotAngle)*-1;
        rightFrontPower = Math.sin(currentCommand.angle * allianceColor.direction + (Math.PI/4) - currentRobotAngle);
        rightBackPower = Math.cos(currentCommand.angle * allianceColor.direction + (Math.PI/4) - currentRobotAngle);

        // Adjusts motor speed.
        leftFrontPower *= currentCommand.power;
        leftBackPower *= currentCommand.power;
        rightFrontPower *= currentCommand.power;
        rightBackPower *= currentCommand.power;

        if (Math.abs(leftFront.getCurrentPosition() - leftFrontTargetPosition) <= ENCODER_POSITION_TOLERANCE &&
            Math.abs(leftBack.getCurrentPosition() - leftBackTargetPosition) <= ENCODER_POSITION_TOLERANCE &&
            Math.abs(rightFront.getCurrentPosition() - rightFrontTargetPosition) <= ENCODER_POSITION_TOLERANCE &&
            Math.abs(rightBack.getCurrentPosition() - rightBackTargetPosition) <= ENCODER_POSITION_TOLERANCE) {
            startNextCommand();
        }
    }

    /**
     * Turns using the encoder positions.
      */
    private void turn() {
        if(commandFirstLoop) {
            targetAngle = currentCommand.angle * allianceColor.direction;
            getAngleError();
        }
        if(Math.abs(angleError) < ANGLE_ERROR_TOLERANCE) {
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

    private void detectDuckPosition() {
        barcodePos = DuckDetectionPipeline.getBarcodePos();
        startNextCommand();
    }

    /**
     * Corrects the target positions and powers of the wheels based on the angle error.
     */
    private void gyroCorrection() {
        getAngleError();

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

        leftFrontPower = 0;
        leftBackPower = 0;
        rightFrontPower = 0;
        rightBackPower = 0;
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
        telemetry.addData("Angle Error", angleError);
    }

    /**
     * If there are new commands, saves the current commands, and replaces them with the new ones.
     */
    private void newCommands(ArrayList<Command> newCommands) {
        if(!newCommands.isEmpty()) {
            if(!currentCommands.isEmpty())
                upstreamCommands.add(0, currentCommands);
            currentCommands = newCommands;
        }
    }

    /**
     * Starts the next command in the sequence. If there are no commands left, checks if there are
     * saved commands and goes through them in a first in last out sequence.
     */
    private void startNextCommand() {
        if (!currentCommands.isEmpty()) {
            currentCommand = currentCommands.get(0);
            currentCommands.remove(0);
            commandFirstLoop = true;
            resetStartTime();
        } else if (!upstreamCommands.isEmpty()) {
            currentCommands = upstreamCommands.get(0);
            upstreamCommands.remove(0);
            currentCommand = currentCommands.get(0);
            currentCommands.remove(0);
            commandFirstLoop = true;
            resetStartTime();
        } else {
            requestOpModeStop();
        }
    }

    protected abstract AllianceColor getAllianceColor();
    protected abstract ArrayList<Command> getCommands();
}