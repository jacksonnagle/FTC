package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DriveCode extends LinearOpMode {
    // Declare motors and servo
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor sweepMotor;
    private DcMotor linearMotor;
    private Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors and servo
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        sweepMotor = hardwareMap.dcMotor.get("sweepMotor");
        linearMotor = hardwareMap.dcMotor.get("linearMotor");
        servo = hardwareMap.servo.get("servo");

        // Reverse right motors if necessary
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize the linear motor correctly
        linearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder
        linearMotor.setTargetPosition(0); // Set the initial target position
        linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set to RUN_TO_POSITION
        linearMotor.setPower(1.0); // Apply power to hold position

        // Set initial position for the servo
        servo.setPosition(0.5); // Neutral middle position

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Control the robot's driving
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;

            double frontLeftPower = drive + strafe + rotate;
            double backLeftPower = drive - strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backRightPower = drive + strafe - rotate;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // D-pad control for sweepMotor
            if (gamepad1.dpad_up) {
                sweepMotor.setPower(-0.4); // Spin forward
            } else if (gamepad1.dpad_down) {
                sweepMotor.setPower(0.4); // Spin backward
            } else {
                sweepMotor.setPower(0); // Stop motor
            }

            // Linear motor control with triggers
            if (gamepad1.right_trigger > 0) {
                // Move up
                int newTargetPosition = linearMotor.getCurrentPosition() + 100;
                linearMotor.setTargetPosition(newTargetPosition);
                linearMotor.setPower(1.0);
            } else if (gamepad1.left_trigger > 0) {
                // Move down
                int newTargetPosition = linearMotor.getCurrentPosition() - 100;
                linearMotor.setTargetPosition(newTargetPosition);
                linearMotor.setPower(1.0);
            } else {
                // Keep the linear motor holding its position
                linearMotor.setTargetPosition(linearMotor.getTargetPosition());
                linearMotor.setPower(1.0);
            }

            // Servo control with X and Y buttons
            double servoPosition = servo.getPosition();
            if (gamepad1.x) {
                // Move servo forward faster
                servoPosition += 0.01; // Increased increment
            } else if (gamepad1.y) {
                // Move servo backward faster
                servoPosition -= 0.01; // Increased decrement
            }

            // Clamp the servo position to its valid range
            servoPosition = Math.max(0, Math.min(1, servoPosition));
            servo.setPosition(servoPosition);

            // Telemetry for debugging
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.addData("Linear Motor Position", linearMotor.getCurrentPosition());
            telemetry.addData("Linear Motor Target", linearMotor.getTargetPosition());
            telemetry.addData("Linear Motor Busy", linearMotor.isBusy());
            telemetry.update();
        }
    }
}