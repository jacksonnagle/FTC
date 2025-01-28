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
        linearMotor.setPower(1.0); // Apply power to maintain position

        // Set initial position for the servo
        servo.setPosition(0.42); // Neutral middle position

        int linearTargetPosition = 0; // Store the current target position of the linear motor

        waitForStart();

        if (isStopRequested()) return;

        boolean isAutonomousRunning = false;

        while (opModeIsActive()) {
            if (!isAutonomousRunning) {
                // Driving controls on gamepad1 (proportional control)
                double drive = -gamepad1.left_stick_y; // Forward/backward
                double strafe = gamepad1.left_stick_x; // Strafing
                double rotate = -gamepad1.right_stick_x; // Rotation

                double frontLeftPower = drive + strafe + rotate;
                double backLeftPower = drive - strafe + rotate;
                double frontRightPower = drive - strafe - rotate;
                double backRightPower = drive + strafe - rotate;

                // Normalize powers to ensure no motor is set above 1.0
                double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                        Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));
                if (maxPower > 1.0) {
                    frontLeftPower /= maxPower;
                    backLeftPower /= maxPower;
                    frontRightPower /= maxPower;
                    backRightPower /= maxPower;
                }

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);

                // Linear motor control on gamepad2
                double linearInput = -gamepad2.right_stick_y; // Up is negative, down is positive
                if (Math.abs(linearInput) > 0.1) { // Deadzone to avoid accidental movements
                    linearTargetPosition += (int) (linearInput * 10); // Adjust target position based on input
                    linearMotor.setTargetPosition(linearTargetPosition); // Update target position
                    linearMotor.setPower(1.0); // Ensure full power for movement
                } else {
                    // Keep the motors holding its position
                    linearMotor.setTargetPosition(linearTargetPosition);
                    linearMotor.setPower(1.0);
                }

                // Sweep motor control on gamepad2 triggers
                if (gamepad2.right_trigger > 0) {
                    sweepMotor.setPower(0.4); // Spin forward
                } else if (gamepad2.left_trigger > 0) {
                    sweepMotor.setPower(-0.4); // Spin backward
                } else {
                    sweepMotor.setPower(0); // Stop motor
                }

                // Servo control presets on gamepad2
                if (gamepad2.x) {
                    // Neutral position
                    servo.setPosition(0.42);
                } else if (gamepad2.y) {
                    // Tilt back halfway
                    servo.setPosition(0.3);
                } else if (gamepad2.b) {
                    // Dump fully forward and reset
                    servo.setPosition(1.0);
                    sleep(500); // Wait half a second for dumping
                    servo.setPosition(0.42); // Return to neutral position
                }
            }

            // Autonomous mode triggered by left bumper
            if (gamepad2.left_bumper && !isAutonomousRunning) {
                isAutonomousRunning = true;
                AutonomousMode.runAutonomous(this, frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, sweepMotor, linearMotor, servo);
                isAutonomousRunning = false;
            }

            // Telemetry for debugging
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.addData("Linear Motor Target", linearTargetPosition);
            telemetry.addData("Linear Motor Current", linearMotor.getCurrentPosition());
            telemetry.addData("Sweep Motor Power", sweepMotor.getPower());
            telemetry.update();
        }
    }
}