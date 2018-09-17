package org.firstinspires.ftc.teamcode; //package org.firstinspires.ftc.teamcode is just stating the program

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Movement", group = "Linear Opmode")
//@Disabled
public class Movement extends LinearOpMode {

    HardwareHexbot robot = new HardwareHexbot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        // We are displaying details on Driver Station
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.resetMotorsAndEncoders();

        robot.openClaw();

        telemetry.addData("Linear Slide", robot.linearSlide.getCurrentPosition());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            float throttle = gamepad1.right_stick_y;
            float turn = -gamepad1.left_stick_x;

            float leftSpeed = -(throttle + turn);
            float rightSpeed = throttle - turn;


            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > .6) {
                leftSpeed /= max;
                rightSpeed /= max;
            }
            robot.leftDriveFront.setPower(leftSpeed);
            robot.leftDriveBack.setPower(leftSpeed);
            robot.rightDriveFront.setPower(rightSpeed);
            robot.rightDriveBack.setPower(rightSpeed);

            telemetry.addData("Right JS", -gamepad1.right_stick_y);
            telemetry.addData("Left JS", gamepad1.left_stick_x);
            telemetry.addData("Left Speed", leftSpeed);
            telemetry.addData("Right Speed", rightSpeed);

            telemetry.update();

            // inward position
            if (gamepad2.left_bumper) {
                robot.closeClaw();
            }

            // outward position
            if (gamepad2.right_bumper) {
                robot.openClaw();
            }

            // Slide has a range. This helps the string not come out of position.
            if (gamepad2.dpad_up) {
                robot.slideup(0.9);
            }
            if (gamepad2.dpad_down) {
                robot.slidedown(0.9);
            }
        }
    }
}
