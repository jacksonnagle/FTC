package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutonomousMode {
    // Updated constants for autonomous (we no longer use rotations for linear motor target)
    private static final double LINEAR_MOTOR_POWER = 0.3; // Slow power for raising the linear motor
    // Removed LINEAR_MOTOR_ROTATIONS and TICKS_PER_ROTATION since we are using a fixed target.
    private static final double SERVO_TILT_POSITION = 0.3;    // Servo tilted position
    private static final double SERVO_DROP_POSITION = 1.0;    // Servo drop position
    private static final double SERVO_NEUTRAL_POSITION = 0.42; // Servo neutral position

    public static void runAutonomous(
            LinearOpMode opMode,
            DcMotor frontLeftMotor,
            DcMotor backLeftMotor,
            DcMotor frontRightMotor,
            DcMotor backRightMotor,
            DcMotor sweepMotor,
            DcMotor linearMotor,
            DcMotor armMotor,  // New ArmMotor parameter
            Servo servo
    ) {
        // Instead of using rotations, we now directly set the linear motor target position to 3000.
        int targetPosition = 0;
        linearMotor.setTargetPosition(targetPosition);
        linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearMotor.setPower(LINEAR_MOTOR_POWER);

        // Tilt the servo after starting the linear motor
        servo.setPosition(SERVO_TILT_POSITION);

        // Initialize variables for pausing and resuming
        boolean isPaused = false;
        boolean wasPaused = false;
        ElapsedTime timer = new ElapsedTime();

        // Step 1: Move 4 inches to the right
        moveByTime(opMode, frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor,
                0.4, -0.4, -0.4, 0.4, 0.5);

        // Step 2: Move forward 25 inches
        moveByTime(opMode, frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor,
                0.5, 0.5, 0.5, 0.5, 2.5);

        // Step 3: Turn 90 degrees counterclockwise
        moveByTime(opMode, frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor,
                -0.4, -0.4, 0.4, 0.4, 1.0);

        // Step 4: Move 10 inches to the right
        moveByTime(opMode, frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor,
                0.4, -0.4, -0.4, 0.4, 1.0);

        // Step 5: Move forward 5 inches
        moveByTime(opMode, frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor,
                0.5, 0.5, 0.5, 0.5, 0.5);

        // Step 6: Wait for the linear motor to finish raising (i.e., reaching position 3000)
        while (opMode.opModeIsActive() && linearMotor.isBusy() && !isPaused) {
            if (opMode.gamepad2.right_bumper) {
                if (!wasPaused) {
                    isPaused = true;
                    stopAllMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, linearMotor);
                    opMode.telemetry.addData("Autonomous", "Paused");
                    opMode.telemetry.update();
                    wasPaused = true;
                } else {
                    isPaused = false;
                    linearMotor.setPower(LINEAR_MOTOR_POWER);
                    opMode.telemetry.addData("Autonomous", "Resumed");
                    opMode.telemetry.update();
                    wasPaused = false;
                }
                while (opMode.gamepad2.right_bumper) {
                    // Wait until the bumper is released to avoid rapid toggling
                }
            }
            opMode.telemetry.addData("Linear Motor Position", linearMotor.getCurrentPosition());
            opMode.telemetry.update();
        }

        // Step 7: Drop the payload
        servo.setPosition(SERVO_DROP_POSITION);
        opMode.sleep(500); // Wait for the servo to drop
        servo.setPosition(SERVO_NEUTRAL_POSITION); // Reset the servo

        // Stop all motors
        stopAllMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, linearMotor);
        // Stop the arm motor as well
        armMotor.setPower(0);

        opMode.telemetry.addData("Autonomous", "Completed");
        opMode.telemetry.update();
    }

    private static void moveByTime(LinearOpMode opMode, DcMotor fl, DcMotor bl, DcMotor fr, DcMotor br,
                                   double flPower, double blPower, double frPower, double brPower, double timeSec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        fl.setPower(flPower);
        bl.setPower(blPower);
        fr.setPower(frPower);
        br.setPower(brPower);

        while (opMode.opModeIsActive() && timer.seconds() < timeSec) {
            opMode.telemetry.addData("Autonomous", "Driving...");
            opMode.telemetry.update();
        }

        stopAllMotors(fl, bl, fr, br, null);
    }

    private static void stopAllMotors(DcMotor fl, DcMotor bl, DcMotor fr, DcMotor br, DcMotor linearMotor) {
        if (fl != null) fl.setPower(0);
        if (bl != null) bl.setPower(0);
        if (fr != null) fr.setPower(0);
        if (br != null) br.setPower(0);
        if (linearMotor != null) linearMotor.setPower(0);
    }
}