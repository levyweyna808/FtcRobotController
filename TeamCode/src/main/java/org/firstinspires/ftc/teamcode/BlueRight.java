
/* * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:*/

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//commit stuff
@Autonomous(name="BlueRight", group="Robot")
public class BlueRight extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor linearSlide = null;
    private Servo grabberServo = null;
    private DcMotor armDrive =null;
    private ElapsedTime runtime = new ElapsedTime();
    //for the drive train
    static final double COUNTS_PER_MOTOR_REV = 28; // General encoders for all rev motors
    static final double DRIVE_GEAR_REDUCTION = 12.0;
    static final double WHEEL_DIAMETER_INCHES = 3.77953;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.7;
    //for sweeper motor
    static final double SWEEPER_GEARING = 9.0;
    static final double SWEEPER_DIAMETER_INCHES = 3;
    static final double COUNTS_PER_SWEEPER_INCH = (COUNTS_PER_MOTOR_REV * SWEEPER_GEARING) / (SWEEPER_DIAMETER_INCHES * 3.1415);
    static final double SPEED_OF_SWEEPER = 1.0;
    //for linear slide
    static final double COUNTS_PER_MOTOR_BILDA = 28;
    static final double LINEAR_SLIDE_GEARING = 	19.20;
    static final double PULLEY_WHEEL_DIAMETER_INCHES = 1.45;
    static final double COUNTS_PER_LS_INCH = (COUNTS_PER_MOTOR_BILDA * LINEAR_SLIDE_GEARING) / (PULLEY_WHEEL_DIAMETER_INCHES * 3.1415);
    static final double LINEAR_SLIDE_SPEED = 1.0;
    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armDrive = hardwareMap.get(DcMotor.class,"Arm_drive" );
        linearSlide = hardwareMap.get(DcMotor.class,"linear_slide");
        grabberServo = hardwareMap.get(Servo.class, "box_tilt_servo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        //servo
        grabberServo.setDirection(Servo.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d", leftBackDrive.getCurrentPosition(), leftFrontDrive.getCurrentPosition(),
        rightBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), armDrive.getCurrentPosition(), linearSlide.getCurrentPosition());
        telemetry.update();

        waitForStart();
        encoderDrive(DRIVE_SPEED,  42,  40, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   -60, 60, 5.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -60, -60, 5.0);
        //SweeperDrive(SPEED_OF_SWEEPER, 0.3, 3.0);
        //LinearSlideDrive(LINEAR_SLIDE_SPEED, 0.4, 3.0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS){
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);
            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Running to",  " %7d :%7d", leftFrontDrive,  rightFrontDrive, leftBackDrive, rightBackDrive);
//                telemetry.addData("Currently at",  " at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(),
//                        leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            // Turn off RUN_TO_POSITION
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }

    public void SweeperDrive(double sweeperSpeed, double sweeperRotationInches, double SweeperTmeoutS){
        int sweeperTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            sweeperTarget = armDrive.getCurrentPosition() + (int)(sweeperRotationInches * COUNTS_PER_SWEEPER_INCH);
            armDrive.setTargetPosition(sweeperTarget);
            // Turn On RUN_TO_POSITION
            armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            armDrive.setPower(Math.abs(sweeperSpeed));
            while (opModeIsActive() && (runtime.seconds() < SweeperTmeoutS) && (armDrive.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Running to",  " %7d :%7d", leftFrontDrive,  rightFrontDrive, leftBackDrive, rightBackDrive);
//                telemetry.addData("Currently at",  " at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(),
//                        leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            armDrive.setPower(0);
            // Turn off RUN_TO_POSITION
            armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }

    public void LinearSlideDrive(double slideSpeed, double slideRotationInches, double slideTmeoutS){
        int slideTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            slideTarget = linearSlide.getCurrentPosition() + (int)(slideRotationInches * COUNTS_PER_LS_INCH);
            linearSlide.setTargetPosition(slideTarget);
            // Turn On RUN_TO_POSITION
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            linearSlide.setPower(Math.abs(slideSpeed));
            while (opModeIsActive() && (runtime.seconds() < slideTmeoutS) && (linearSlide.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Running to",  " %7d :%7d", leftFrontDrive,  rightFrontDrive, leftBackDrive, rightBackDrive);
//                telemetry.addData("Currently at",  " at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(),
//                        leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            linearSlide.setPower(0);
            // Turn off RUN_TO_POSITION
            linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }
}
