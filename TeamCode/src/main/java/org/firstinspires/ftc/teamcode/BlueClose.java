package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
import com.qualcomm.robotcore.util.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="BlueClose",group = "Red Autonomus")
//@Disabled
public class BlueClose extends LinearOpMode {

    //SET DEBUGWAIT TO ZERO FOR NORMAL RUN
    static int debugWait = 0;
    RelicRecoveryVuMark currentVuMark = RelicRecoveryVuMark.UNKNOWN;
    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    Orientation angles;

    //instance of VumarkDetection
    HexbotVuMarkDetection vuDetector = new HexbotVuMarkDetection();
    HardwareHexbot robot = new HardwareHexbot();
    @Override
    public void runOpMode() throws InterruptedException {
        try {

                /*
                 * Initialize the drive system variables.
                 * The init() method of the hardware class does all the work here
                 */
            robot.init(hardwareMap, telemetry);

            // hsvValues is an array that will hold the hue, saturation, and value information.
            float hsvValues[] = {0F, 0F, 0F};

            // values is a reference to the hsvValues array.

            // sometimes it helps to multiply the raw RGB values with a scale factor
            // to amplify/attentuate the measured values.
            final double SCALE_FACTOR = 255;

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Ready to run");
            telemetry.update();

            vuDetector.init(hardwareMap, telemetry);

            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Wait for the game to start (driver presses PLAY)
            // robot grabs the glyph
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            waitForStart();
            robot.reset();
            robot.closeClaw();
            sleep(1000);
            robot.slideup(0.5,1500);
            sleep(700);

            // Robot id decting cryptokry
            telemetry.addData("Status", "Detecting Crypto-Key");
            telemetry.update();
            //crypto key code, move init outside waitforstart later
            vuDetector.RunDetection();
            currentVuMark = vuDetector.getCryptoKey();
            //ADD CODE TO TURN OFF THE CAMERA
            //  robot.driveForwardInches(2, .1,1.5);

            telemetry.addData("Status", "Dropping Color Sensor arm");
            telemetry.addData("Status", "Move to Crypto-Box Position-%s", currentVuMark);
            telemetry.update();
            robot.jewelServo.setPosition(0.93);
            sleep(2000);

            telemetry.addData("Status", "Detecting Jewel Color");
            Color.RGBToHSV((int) (robot.colorSensor.red() * SCALE_FACTOR),
                    (int) (robot.colorSensor.green() * SCALE_FACTOR),
                    (int) (robot.colorSensor.blue() * SCALE_FACTOR),
                    hsvValues);

            telemetry.addData("Status", "Displacing Jewel");
            if (robot.colorSensor.blue() > robot.colorSensor.red()) {
                telemetry.addData("Color:", "Red");
                telemetry.update();
                robot.driveForwardInches(2,0.4,1);
                sleep(500);
                robot.jewelServo.setPosition(0);
                robot.driveBackwardInches(2,0.4,1);
                // robot.turnDegrees(10);



                sleep(200);
                //robot.turnDegrees(-10);
            }
            else {
                telemetry.addData("Color:", "Blue");
                telemetry.update();
                //robot.turnDegrees(-10);
                robot.driveBackwardInches(2,0.4,1);
                sleep(500);

                robot.jewelServo.setPosition(0);
                robot.driveForwardInches(2,0.4,1);
                sleep(200);
                //robot.turnDegrees(10);
            }

            telemetry.update();


            sleep(debugWait);


            telemetry.update();
            sleep(debugWait);
            telemetry.addData("Status", "Placing glyph in %s position", currentVuMark);



            switch (currentVuMark) {
                case UNKNOWN:
                    //if unknown, assume center and continue
                    robot.driveBackwardInches(47,0.4,6);
                    robot.turnDegrees(-49);
                    robot.driveForwardInches(15,0.4,2);
                    break;
                case CENTER:
                    robot.driveBackwardInches(47,0.4,6);
                    robot.turnDegrees(-49);
                    robot.driveForwardInches(15,0.4,2);
                    break;
                case LEFT:
                    robot.driveBackwardInches(44,0.4,6);
                    robot.turnDegrees(-56);
                    robot.driveForwardInches(9,0.4,2);
                    break;
                case RIGHT:
                    robot.driveBackwardInches(28,0.4,6);
                    robot.turnDegrees(-126);
                    robot.driveForwardInches(14,0.4,2);
                    break;
            }
            //do attachment stuff here
            telemetry.addData("Heading:", robot.getCurrentAngle());
            telemetry.update();
            sleep(debugWait);

            sleep(500);
            robot.openClaw();
            telemetry.addData("Status", "Parking in Safe Zone");
            robot.driveBackwardInches(3,0.4,1);
            robot.linearSlide.setPower(-.3);
            sleep(700);
            robot.slideup(0.5,0);
            robot.reset();

            //robot.driveBackwardInches(5,0.2,2);

            telemetry.update();
            sleep(debugWait);
            //robot.resetMotorsAndEncoders();
        }
        catch(InterruptedException e){

        }
        finally{

            robot.reset();
        }
    }

}
