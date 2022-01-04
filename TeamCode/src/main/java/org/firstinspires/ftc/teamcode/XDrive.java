package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.Constants.BRISTLES_POWER_IN;


public abstract class XDrive extends OpMode {

    protected static AllianceColor allianceColor;

    //Motors
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor armRotator;
    private DcMotor armExtender;

    //Servos
    private Servo duckWheel;
    private Servo bristleServo;

    DigitalChannel digitalTouch;

    //Creating the variables for the gyro sensor
    private BNO055IMU imu;

    private Orientation angles;
    private double angleOffset;
    private double currentRobotAngle;

    private boolean xPressed;
    private boolean yPressed;
    private boolean aPressed;
    private boolean bPressed;
    private boolean duckIn;
    private boolean duckOut;
    private boolean bristlesIn;
    private boolean bristlesOut;
    private boolean limitSwitchPressed;

    public void init() {
        allianceColor = getAllianceColor();

        /*Instantiating the motor and servo objects as their appropriate motor/servo in the
        configuration on the robot*/
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        armRotator = hardwareMap.get(DcMotor.class, "armRotator");
        armExtender = hardwareMap.get(DcMotor.class, "armExtender");

        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        duckWheel = hardwareMap.get(Servo.class, "duckWheel");
        bristleServo = hardwareMap.get(Servo.class, "bristleServo");

        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        //Initializing the RevHub IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = null;
        currentRobotAngle = 0.0;
        angleOffset = 0.0;
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
        double gamepad1LeftTrigger = gamepad1.left_trigger;
        double gamepad1RightTrigger = gamepad1.right_trigger;

        double angleError = 0.0;

        double leftFrontPower;
        double leftBackPower ;
        double rightFrontPower;
        double rightBackPower;

        if (Math.abs(gamepad1LeftStickX) >= CONTROLLER_TOLERANCE || Math.abs(gamepad1LeftStickY) >= CONTROLLER_TOLERANCE) {

            //Uses atan2 to convert the x and y values of the controller to an angle
            double gamepad1LeftStickAngle = Math.atan2(gamepad1LeftStickY, gamepad1LeftStickX);

            /*Subtracts the robot's current angle from the command angle so that it travels globally
            rather than relative to the robot, then rotates it 45 degrees so that the angle's components
            align with the wheels*/
            double holonomicAngle = gamepad1LeftStickAngle + currentRobotAngle + Math.PI / 4;

            //overall power based on how far the stick is from the center
            double power = Math.sqrt(Math.pow(gamepad1LeftStickX, 2) + Math.pow(gamepad1LeftStickY, 2));

            //the main diagonal is the diagonal from top left to bottom right
            double mainDiagonalPercent = Math.cos(holonomicAngle);
            //the anti-diagonal is the diagonal from topRight to bottomLeft
            double antiDiagonalPercent = Math.sin(holonomicAngle);

            leftFrontPower = mainDiagonalPercent * -power;
            rightBackPower = mainDiagonalPercent * power;
            rightFrontPower = antiDiagonalPercent * -power;
            leftBackPower = antiDiagonalPercent * power;

        } else {

            leftFrontPower = 0.0;
            leftBackPower = 0.0;
            rightFrontPower = 0.0;
            rightBackPower = 0.0;

        }

        if (gamepad1LeftTrigger >= CONTROLLER_TOLERANCE) {
            angleError += Math.pow(gamepad1LeftTrigger, 2);
        }
        if (gamepad1RightTrigger >= CONTROLLER_TOLERANCE) {
            angleError -= Math.pow(gamepad1RightTrigger, 2);
        }

        leftFrontPower += angleError * TURNING_POWER_SCALAR;
        leftBackPower += angleError * TURNING_POWER_SCALAR;
        rightFrontPower += angleError * TURNING_POWER_SCALAR;
        rightBackPower += angleError * TURNING_POWER_SCALAR;

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
            duckWheel.setPosition(0.5 + DUCK_SPEED * allianceColor.direction);
        } else {
            duckWheel.setPosition(0.5);
        }
    }

    /**
     * Controls the arm.
     */
    private void arm() {
        if (Math.abs(gamepad2.left_stick_y) >= CONTROLLER_TOLERANCE) {
            armRotator.setPower(gamepad2.left_stick_y);
        } else {
            armRotator.setPower(0.0);
        }

        //Double toggle for the bristles
        if (gamepad2.b) {
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
        if (gamepad2.a) {
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
            bristleServo.setPosition(.5 + BRISTLES_POWER_IN / 2);
        } else if (bristlesOut) {
            bristleServo.setPosition(.5 - BRISTLES_POWER_OUT / 2);
        } else {
            bristleServo.setPosition(0.5);
        }

        // Controls the arm extender
        if (digitalTouch.getState()) {
            telemetry.addData("Digital Touch", "Pressed");
        } else {
            telemetry.addData("Digital Touch", "Not Pressed");
        }

        if (!digitalTouch.getState() && gamepad2.right_stick_y >= CONTROLLER_TOLERANCE) {
            armExtender.setPower(gamepad2.right_stick_y / 2);
        } else if (armExtender.getCurrentPosition() > MAX_ARM_EXTENSION && gamepad2.right_stick_y <= -CONTROLLER_TOLERANCE) {
            armExtender.setPower(gamepad2.right_stick_y / 2);
        } else {
            armExtender.setPower(0.0);
        }

        if (digitalTouch.getState()) {
            if (!limitSwitchPressed) {
                limitSwitchPressed = true;
                armExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }
        } else {
            if (limitSwitchPressed) {
                armExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            limitSwitchPressed = false;
        }
        telemetry.addData("Arm Extender", armExtender.getCurrentPosition());
        telemetry.addData("Arm Rotator", armRotator.getCurrentPosition());

    }

    protected abstract AllianceColor getAllianceColor();
}