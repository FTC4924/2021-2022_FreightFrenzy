package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.Constants.BRISTLES_POWER;


public abstract class XDrive extends OpMode {

    protected static AllianceColor allianceColor;

    private FtcDashboard dashboard;
    private Telemetry dashboardTelemetry;

    //Motors
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor armMotor;

    //Servos
    private Servo duckServo;
    private Servo bristleServo;

    //Creating the variables for the gyro sensor
    private BNO055IMU imu;

    private Orientation angles;
    private double angleOffset;
    private double currentRobotAngle;
    private double targetAngle;
    private double robotAngleError;

    private double duckAccelerate;

    private boolean xPressed;
    private boolean yPressed;
    private boolean aPressed;
    private boolean bPressed;
    private boolean duckIn;
    private boolean duckOut;
    private boolean bristlesIn;
    private boolean bristlesOut;

    public void init() {

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        /*Instantiating the motor and servo objects as their appropriate motor/servo in the
        configuration on the robot*/
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        duckServo = hardwareMap.get(Servo.class, "duckServo");
        bristleServo = hardwareMap.get(Servo.class, "bristleServo");

        //Initializing the Revhub IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = null;
        currentRobotAngle = 0.0;
        targetAngle = 0.0;
        angleOffset = 0.0;
        duckAccelerate = 0.1;
    }

    public void loop() {

        getAngle();

        recalibrateGyro();

        holonomicDrive();

        ducks();

        arm();

    }

    /**
     * Gets the angle of the robot from the rev imu and subtracts the angle offset.
     */
    private void getAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, RADIANS);
        currentRobotAngle = angles.firstAngle - angleOffset;
    }

    /**
     * Sets the gyro angle offset based off of the current angle when the b button is pressed.
     */
    private void recalibrateGyro() {
        if(gamepad1.b) {
            angleOffset = angles.firstAngle;
            targetAngle = 0.0;
        }
    }

    /**
     * Holonomic controls according to what direction the robot is facing when we start the
     * program or when we recalibrate the gyro.
     * Uses the left stick to control movement, the triggers to control turning using exponential
     * controls, and the right stick up and down for speed.
     */
    private void holonomicDrive() {

        double gamepad1LeftStickX = gamepad1.left_stick_x;
        double gamepad1LeftStickY = gamepad1.left_stick_y;
        double gamepad1RightStickX = gamepad1.right_stick_x;
        double gamepad1RightStickY = gamepad1.right_stick_y;
        double gamepad1LeftTrigger = gamepad1.left_trigger;
        double gamepad1RightTrigger = gamepad1.right_trigger;
        robotAngleError = 0;

        double leftFrontPower;
        double leftBackPower ;
        double rightFrontPower;
        double rightBackPower;

        if (Math.abs(gamepad1LeftStickX) >= CONTROLLER_TOLERANCE || Math.abs(gamepad1LeftStickY) >= CONTROLLER_TOLERANCE) {

            //Uses atan2 to convert the x and y values of the controller to an angle
            double gamepad1LeftStickAngle = Math.atan2(gamepad1LeftStickY, gamepad1LeftStickX);

            /*Determines what power each wheel should get based on the angle we get from the stick
            plus the current robot angle so that the controls are independent of what direction the
            robot is facing*/
            leftFrontPower = Math.cos(gamepad1LeftStickAngle + Math.PI / 4 + currentRobotAngle) * -1;
            leftBackPower = Math.sin(gamepad1LeftStickAngle + Math.PI / 4 + currentRobotAngle);
            rightFrontPower = Math.sin(gamepad1LeftStickAngle + Math.PI / 4 + currentRobotAngle) * -1;
            rightBackPower = Math.cos(gamepad1LeftStickAngle + Math.PI / 4 + currentRobotAngle);

            /*Uses the Y of the right stick to determine the speed of the robot's movement with 0
            being 0.5 power*/
            double rightYPower = Math.sqrt(Math.pow(gamepad1LeftStickX, 2) + Math.pow(gamepad1LeftStickY, 2));
            leftFrontPower *= rightYPower;
            leftBackPower *= rightYPower;
            rightFrontPower *= rightYPower;
            rightBackPower *= rightYPower;

        } else {

            leftFrontPower = 0.0;
            leftBackPower = 0.0;
            rightFrontPower = 0.0;
            rightBackPower = 0.0;

        }

        if (gamepad1LeftTrigger >= CONTROLLER_TOLERANCE) {
            //targetAngle += Math.toRadians(gamepad1LeftTrigger * TURNING_ANGLE_POSITION_SCALAR);

            robotAngleError += gamepad1LeftTrigger;
        }
        if (gamepad1RightTrigger >= CONTROLLER_TOLERANCE) {
            //targetAngle -= Math.toRadians(gamepad1RightTrigger * TURNING_ANGLE_POSITION_SCALAR);
            robotAngleError -= gamepad1RightTrigger;
        }

        telemetry.addData("target angle", Math.toDegrees(targetAngle));
        dashboardTelemetry.addData("robotAngleError",robotAngleError);
        dashboardTelemetry.update();

        //robotAngleError = targetAngle - currentRobotAngle;
        //robotAngleError = ((((robotAngleError - Math.PI) % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI)) - Math.PI;

        leftFrontPower += robotAngleError * TURNING_POWER_SCALAR;
        leftBackPower += robotAngleError * TURNING_POWER_SCALAR;
        rightFrontPower += robotAngleError * TURNING_POWER_SCALAR;
        rightBackPower += robotAngleError * TURNING_POWER_SCALAR;

        //Sets the wheel powers
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

    }

    /**
     * Controls for the duck wheel.
     */
    private void ducks() {
        if (gamepad2.x) {
            if (!xPressed) {
                xPressed = true;
                duckIn = !duckIn;
            }
        } else {
            xPressed = false;
        }

        if (gamepad2.y) {
            if (!yPressed) {
                yPressed = true;
                duckOut = !duckOut;
            }
        } else {
            yPressed = false;
        }

        if (duckIn) {
            duckServo.setPosition(0.5 + DUCK_SPEED * allianceColor.direction);
        } else {
            duckServo.setPosition(0.5);
        }
        telemetry.addData("Duck Accelerate", duckAccelerate);
    }

    /**
     * Controls the arm.
     */
    private void arm() {
        if (Math.abs(gamepad2.left_stick_y) >= CONTROLLER_TOLERANCE) {
            armMotor.setPower(gamepad2.left_stick_y);
        } else {
            armMotor.setPower(0.0);
        }

        //Double toggle for the bristles
        if (gamepad2.a) {
            if (!aPressed) {
                aPressed = true;
                bristlesIn = !bristlesIn;
                if (bristlesIn) {
                    bristlesOut = false;
                }
            }
        } else {
            aPressed = false;
        }
        if (gamepad2.b) {
            if (!bPressed) {
                bPressed = true;
                bristlesOut = !bristlesOut;
                if (bristlesOut) {
                    bristlesIn = false;
                }
            }
        } else {
            bPressed = false;
        }

        //Setting the bristles power
        if (bristlesIn) {
            bristleServo.setPosition(BRISTLES_POWER);
        } else if (bristlesOut) {
            bristleServo.setPosition(1 - BRISTLES_POWER);
        } else {
            bristleServo.setPosition(0.5);
        }
    }

    protected abstract AllianceColor getAllianceColor();
}