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

@TeleOp(name="XDrive")
public class XDrive extends OpMode {

    private FtcDashboard dashboard;
    private Telemetry dashboardTelemetry;

    //Motors
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    //Servos
    private Servo duckServo;

    //Creating the variables for the gyro sensor
    private BNO055IMU imu;

    private Orientation angles;
    private double angleOffset;
    private double currentRobotAngle;
    private double targetAngle;
    private double robotAngleError;

    private boolean xPressed;
    private boolean duckOn;

    public void init() {

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        /*Instantiating the motor and servo objects as their appropriate motor/servo in the
        configuration on the robot*/
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        duckServo = hardwareMap.get(Servo.class, "duckServo");

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
    }

    public void loop() {

        getAngle();

        recalibrateGyro();

        holonomicDrive();

        ducks();

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
            targetAngle += Math.toRadians(gamepad1LeftTrigger * TURNING_ANGLE_POSITION_SCALAR);
        }
        if (gamepad1RightTrigger >= CONTROLLER_TOLERANCE) {
            targetAngle -= Math.toRadians(gamepad1RightTrigger * TURNING_ANGLE_POSITION_SCALAR);
        }

        telemetry.addData("target angle", Math.toDegrees(targetAngle));
        dashboardTelemetry.addData("robotAngleError",robotAngleError);
        dashboardTelemetry.update();

        robotAngleError = targetAngle - currentRobotAngle;
        robotAngleError = ((((robotAngleError - Math.PI) % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI)) - Math.PI;

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

    private void ducks() {
        if (gamepad1.x) {
            if (!xPressed) {
                xPressed = true;
                duckOn = !duckOn;
            }
        } else {
            xPressed = false;
        }

        if (duckOn) {
            duckServo.setPosition(1.0);
        } else {
            duckServo.setPosition(0.5);
        }
    }
}