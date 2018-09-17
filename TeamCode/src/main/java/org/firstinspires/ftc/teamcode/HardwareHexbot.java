
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


public class HardwareHexbot {
    /* Public OpMode members. */
    public DcMotor leftDriveFront = null;
    public DcMotor leftDriveBack = null;
    public DcMotor rightDriveFront = null;
    public DcMotor rightDriveBack = null;
    public DcMotor linearSlide = null;
    public Servo bottomLeftClaw = null;
    public Servo bottomRightClaw = null;
    public Servo topLeftClaw = null;
    public Servo topRightClaw = null;
    public Servo jewelServo = null;
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;
    BNO055IMU imu;
    Orientation angles;
    Telemetry localtelemetry;
    ElapsedTime runtime = new ElapsedTime(); //what are we doing here

    public final int ANDYMARK_ENCODER_COUNT_PER_REV = 1120;
    //public final int TETRIX_REVOLUTION = 1440;
    public final double WHEEL_DIAMETER = 3.8;
    public final double COUNTS_PER_INCH = ANDYMARK_ENCODER_COUNT_PER_REV / (WHEEL_DIAMETER * Math.PI);
    //CPI is about 4827.35 per the above math
    public static final double LEFT_CLAW_BOTTOM_INITIAL_POS = 0.65 ;
    public static final double RIGHT_CLAW_BOTTOM_INITIAL_POS = 0.15;
    public static final double LEFT_CLAW_TOP_INITIAL_POS = 0.8;
    public static final double RIGHT_CLAW_TOP_INITIAL_POS = 0.1;

    public static final double SLIDER_UP_POWER = 0.5;
    public static final double SLIDER_DOWN_POWER = -0.5;
    public static final double JEWEL_INITIAL_POS = 0;

    /* local OpMode members. */
    HardwareMap hwMap = null;


    /* Constructor */
    public HardwareHexbot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telemetry) {

        localtelemetry = telemetry;

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize drive Motors
        leftDriveFront = hwMap.get(DcMotor.class, "left_driveFront");
        leftDriveBack = hwMap.get(DcMotor.class, "left_driveBack");
        rightDriveFront = hwMap.get(DcMotor.class, "right_driveFront");
        rightDriveBack = hwMap.get(DcMotor.class, "right_driveBack");

        // Define and Initialize slide Motor

        linearSlide = hwMap.get(DcMotor.class, "linear_slide");

        // Define and initialize ALL installed servos.
        bottomLeftClaw = hwMap.get(Servo.class, "left_claw_bottom");
        bottomRightClaw = hwMap.get(Servo.class, "right_claw_bottom");
        topLeftClaw = hwMap.get(Servo.class, "left_claw_top");
        topRightClaw = hwMap.get(Servo.class, "right_claw_top");
        jewelServo = hwMap.get(Servo.class, "jewel_servo");

        //adding color and distance sensor
        colorSensor = hwMap.colorSensor.get("sensor_color_distance");
        distanceSensor = hwMap.get(DistanceSensor.class, "sensor_color_distance");
        //adding rev imu (gyro,accelerometer,etc.)
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        runtime.reset();
        resetMotorsAndEncoders();

    }


    //----------------------------------------------------------------------------------------------
    // Angle Measurement Formatting
    //----------------------------------------------------------------------------------------------

    public double getCurrentAngle() {
        double anglesNorm;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        anglesNorm = NormalizeDegrees(angles.angleUnit, angles.firstAngle);
        return anglesNorm;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public double NormalizeDegrees(AngleUnit angleUnit, double angle) {
        double degrees;
        degrees = AngleUnit.DEGREES.fromUnit(angleUnit, angle);
        degrees = AngleUnit.DEGREES.normalize(degrees);
        return degrees;
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    //----------------------------------------------------------------------------------------------
    // Methods to Reset Motors and Encoders
    //----------------------------------------------------------------------------------------------

    public void resetMotorsAndEncoders() {
        resetMotors();
        resetEncoderValues();
    }


    public void resetMotors() {
        leftDriveFront.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveFront.setPower(0);
        rightDriveBack.setPower(0);
        linearSlide.setPower(0);
    }

    public void resetEncoderValues() {
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void setEncoderMode(DcMotor.RunMode mode) {
        leftDriveFront.setMode(mode);
        leftDriveBack.setMode(mode);
        rightDriveFront.setMode(mode);
        rightDriveBack.setMode(mode);
        linearSlide.setMode(mode);
    }


    public void reset() {
        resetMotorsAndEncoders();
        bottomLeftClaw.setPosition(0);
        bottomRightClaw.setPosition(0);
        topLeftClaw.setPosition(.9);
        topRightClaw.setPosition(.4);
        jewelServo.setPosition(.6);
    }
    //----------------------------------------------------------------------------------------------
    // Methods for Drive Motors
    //----------------------------------------------------------------------------------------------

    public void driveBackwardInches(double inches, double power, double timeout) {
        resetMotorsAndEncoders();
        int tolerance = 20;
        int counts = (int) Math.round(COUNTS_PER_INCH * inches);
//must set direction first
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);

        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
//then set position
        leftDriveFront.setTargetPosition(-counts);
        leftDriveBack.setTargetPosition(-counts);
        rightDriveFront.setTargetPosition(-counts);
        rightDriveBack.setTargetPosition(-counts);
//then set the mode
        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//then set the desired power
        runtime.reset();
        leftDriveFront.setPower(-power);
        leftDriveBack.setPower(-power);
        rightDriveFront.setPower(power);
        rightDriveBack.setPower(power);


        while (Math.abs(leftDriveFront.getTargetPosition() - leftDriveFront.getCurrentPosition())
                > tolerance || Math.abs(rightDriveFront.getTargetPosition() - rightDriveFront.getCurrentPosition())
                > tolerance) {
            if (runtime.seconds() > timeout) {
                break;
            }

            localtelemetry.addData("Current LeftDriveFront Counts:", (leftDriveFront.getCurrentPosition()));
            localtelemetry.addData("Current LeftDriveBack Counts:", (leftDriveBack.getCurrentPosition()));
            localtelemetry.addData("Current RightDriveFront Counts:", (rightDriveFront.getCurrentPosition()));
            localtelemetry.addData("Current RightDriveBack Counts:", (rightDriveBack.getCurrentPosition()));
            localtelemetry.addData("LeftDriveFront Power:", leftDriveFront.getPower());
            localtelemetry.addData("LeftDriveBack Power:", leftDriveBack.getPower());
            localtelemetry.addData("RightDriveFront Power:", rightDriveFront.getPower());
            localtelemetry.addData("RightDriveBack Power:", rightDriveBack.getPower());
            localtelemetry.addData("LeftDriveFront Target Pos:", leftDriveFront.getTargetPosition());
            localtelemetry.addData("RightDriveFront Target Pos:", rightDriveFront.getTargetPosition());
            localtelemetry.update();
        }
        resetMotorsAndEncoders();
    }


    public void driveForwardInches(double inches, double power, double timeout) {
        resetMotorsAndEncoders();
        int tolerance = 20;
        int counts = (int) Math.round(COUNTS_PER_INCH * inches);
//must set direction first
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);

        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
//then set position
        leftDriveFront.setTargetPosition(-counts);
        leftDriveBack.setTargetPosition(-counts);
        rightDriveFront.setTargetPosition(-counts);
        rightDriveBack.setTargetPosition(-counts);
//then set the mode
        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//then set the desired power
        runtime.reset();
        leftDriveFront.setPower(power);
        leftDriveBack.setPower(power);
        rightDriveFront.setPower(-power);
        rightDriveBack.setPower(-power);


        while (Math.abs(leftDriveFront.getTargetPosition() - leftDriveFront.getCurrentPosition())
                > tolerance || Math.abs(rightDriveFront.getTargetPosition() - rightDriveFront.getCurrentPosition())
                > tolerance) {
            if (runtime.seconds() > timeout) {
                break;
            }

            localtelemetry.addData("Current LeftDriveFront Counts:", (leftDriveFront.getCurrentPosition()));
            localtelemetry.addData("Current LeftDriveBack Counts:", (leftDriveBack.getCurrentPosition()));
            localtelemetry.addData("Current RightDriveFront Counts:", (rightDriveFront.getCurrentPosition()));
            localtelemetry.addData("Current RightDriveBack Counts:", (rightDriveBack.getCurrentPosition()));
            localtelemetry.addData("LeftDriveFront Power:", leftDriveFront.getPower());
            localtelemetry.addData("LeftDriveBack Power:", leftDriveBack.getPower());
            localtelemetry.addData("RightDriveFront Power:", rightDriveFront.getPower());
            localtelemetry.addData("RightDriveBack Power:", rightDriveBack.getPower());
            localtelemetry.addData("LeftDriveFront Target Pos:", leftDriveFront.getTargetPosition());
            localtelemetry.addData("RightDriveFront Target Pos:", rightDriveFront.getTargetPosition());
            localtelemetry.update();
        }
        resetMotorsAndEncoders();
    }

    public void setTargetPosition(int left, int right) {

        leftDriveFront.setTargetPosition(left);
        leftDriveBack.setTargetPosition(left);
        rightDriveFront.setTargetPosition(right);
        rightDriveBack.setTargetPosition(right);
    }


    public void setDrivePower(double left, double right) {

        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);

        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        leftDriveFront.setPower(-left);
        leftDriveBack.setPower(-left);
        rightDriveFront.setPower(right);
        rightDriveBack.setPower(right);
    }


    public void driveStraight(double power, double time) {
        resetMotorsAndEncoders();
        double startingAngle = getCurrentAngle();
        double tolerance = 5;
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();
        while (runtime.seconds() < time) {
            double localAngle = getCurrentAngle();
            localtelemetry.addData("Runtime:", runtime.seconds());
            localtelemetry.addData("Heading:", localAngle);
            localtelemetry.addData("Left Front Motor Power:", leftDriveFront.getPower());
            localtelemetry.addData("Right Front Motor Power:", leftDriveFront.getPower());
            localtelemetry.update();
            if (Math.abs(localAngle - startingAngle) <= tolerance) {
                setDrivePower(power, power);
            } else {
                if ((localAngle - startingAngle) > 0) {
                    setDrivePower(power, power * 0.5);
                } else {
                    setDrivePower(power * 0.5, power);
                }
            }
        }
        resetMotorsAndEncoders();

    }

    public void turnToDegrees(float degrees) {
        resetMotorsAndEncoders();

        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);

        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);


        double tolerance = 5;
        //wrap final Angle to +/- 180
        if (degrees > 180)
            degrees -= 360;
        if (degrees <= -180)
            degrees += 360;

        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (degrees > 0) {
            setDrivePower(-0.2, 0.2);
        } else {
            setDrivePower(0.2, -0.2);
        }
        while (Math.abs(degrees - getCurrentAngle()) > tolerance) {
            localtelemetry.addData("Heading:", getCurrentAngle());
            localtelemetry.addData("Target Angle", degrees);
            localtelemetry.update();
        }
        resetMotorsAndEncoders();

    }

    public void turnDegrees(float degrees) {
        resetMotorsAndEncoders();

        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);

        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        double startingAngle = getCurrentAngle();
        double finalAngle = startingAngle + degrees;
        double tolerance = 3;
        //wrap final Angle to +/- 180
        if (finalAngle > 180)
            finalAngle -= 360;
        if (finalAngle <= -180)
            finalAngle += 360;

        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (finalAngle > startingAngle) {
            setDrivePower(-0.2, 0.2);
        } else {
            setDrivePower(0.2, -0.2);
        }
        while (Math.abs(finalAngle - getCurrentAngle()) > tolerance) {
            localtelemetry.addData("Heading:", getCurrentAngle());
            localtelemetry.addData("start angle:", startingAngle);
            localtelemetry.addData("final angle:", finalAngle);
            localtelemetry.update();
        }
    }

    public void openClaw(){
        topLeftClaw.setPosition(LEFT_CLAW_TOP_INITIAL_POS);
        topRightClaw.setPosition(RIGHT_CLAW_TOP_INITIAL_POS);
        bottomLeftClaw.setPosition(LEFT_CLAW_BOTTOM_INITIAL_POS);
        bottomRightClaw.setPosition(RIGHT_CLAW_BOTTOM_INITIAL_POS);
    }
    public void closeClaw(){

        bottomLeftClaw.setPosition(0.25);
        bottomRightClaw.setPosition(0.55);
        topLeftClaw.setPosition(0.35);
        topRightClaw.setPosition(0.55);
    }


    public void slideup(double power) {

        int tolerance = 20;
        int targetPosition = 5000;
        localtelemetry.addData("Linearslide Current Position:", (linearSlide.getCurrentPosition()));
        localtelemetry.update();
        if (linearSlide.getCurrentPosition() < targetPosition) {
//must set direction first
            linearSlide.setDirection(DcMotor.Direction.FORWARD);


//then set position
            linearSlide.setTargetPosition(targetPosition);

//then set the mode
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//then set the desired power

            linearSlide.setPower(power);

            while (Math.abs(targetPosition - linearSlide.getCurrentPosition())
                    > tolerance) {
            }

            localtelemetry.addData("Linearslide Counts:", (linearSlide.getCurrentPosition()));

            localtelemetry.addData("LinearSlide Power:", leftDriveBack.getPower());

            localtelemetry.addData("Linear Slide Target Pos:", leftDriveFront.getTargetPosition());

            localtelemetry.update();
        }
        linearSlide.setPower(0);
    }


    public void slidedown(double power) {

        int tolerance = 20;
        int targetPosition = 0;
        power = -power;

        localtelemetry.addData("Linearslide Current Position:", (linearSlide.getCurrentPosition()));
        localtelemetry.update();
        if (linearSlide.getCurrentPosition() > targetPosition) {
//must set direction first
            linearSlide.setDirection(DcMotor.Direction.REVERSE);


//then set position
            linearSlide.setTargetPosition(targetPosition);

//then set the mode
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//then set the desired power

            linearSlide.setPower(power);

            while (Math.abs( linearSlide.getCurrentPosition() - targetPosition)
                    > tolerance) {
            }

            localtelemetry.addData("Linearslide Counts:", (linearSlide.getCurrentPosition()));

            localtelemetry.addData("LinearSlide Power:", leftDriveBack.getPower());

            localtelemetry.addData("Linear Slide Target Pos:", leftDriveFront.getTargetPosition());

            localtelemetry.update();
        }
        linearSlide.setPower(0);
    }

    public void slideup(double power, int position) {

        int tolerance = 20;
        int targetPosition = position;
        localtelemetry.addData("Linearslide Current Position:", (linearSlide.getCurrentPosition()));
        localtelemetry.update();
        if (linearSlide.getCurrentPosition() < targetPosition) {
//must set direction first
            linearSlide.setDirection(DcMotor.Direction.FORWARD);


//then set position
            linearSlide.setTargetPosition(targetPosition);

//then set the mode
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//then set the desired power

            linearSlide.setPower(power);

            while (Math.abs(targetPosition - linearSlide.getCurrentPosition())
                    > tolerance) {
            }

            localtelemetry.addData("Linearslide Counts:", (linearSlide.getCurrentPosition()));

            localtelemetry.addData("LinearSlide Power:", leftDriveBack.getPower());

            localtelemetry.addData("Linear Slide Target Pos:", leftDriveFront.getTargetPosition());

            localtelemetry.update();
        }
        linearSlide.setPower(0);
    }

}
