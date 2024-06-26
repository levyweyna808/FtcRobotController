/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//hello
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="2 player drive mode", group="Linear OpMode")
// @Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armDrive = null;
    private DcMotor armDrive2 = null;
    private DcMotor linearSlide = null;
    private DcMotor otherSlide = null;
    private  Servo grabberServo = null;
    private Servo door_opener_servo = null;
    boolean imGoingToKillMyself = false;
    //good luck
    private DcMotor planeYeeter = null;
    //do been yeeten
    //private Servo rightClaw = null;
    //private Servo leftClaw = null;
    private Servo rightServo = null;
    private Servo leftServo = null;

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double WHEEL_DIAMETER_MM = 75;
    //mucho grande
    static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM * 0.0393701;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * 1/9) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private DigitalChannel digitalTouch=null;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armDrive = hardwareMap.get(DcMotor.class,"Arm_drive" );
        armDrive2 = hardwareMap.get(DcMotor.class,"linear_slide");
        //linearSlide = hardwareMap.get(DcMotor.class,"insertnamehere");
        grabberServo = hardwareMap.get(Servo.class, "box_tilt_servo");
        door_opener_servo = hardwareMap.get(Servo.class, "door_servo");
        planeYeeter = hardwareMap.get(DcMotor.class, "PlaneYeeter");
        otherSlide = hardwareMap.get(DcMotor.class,"Slide2");
        rightServo = hardwareMap.get(Servo.class,"rightServo");
        leftServo = hardwareMap.get(Servo.class,"leftServo");
        //rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        //leftClaw = hardwareMap.get(Servo.class, "leftClaw");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        otherSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        planeYeeter.setDirection(DcMotor.Direction.REVERSE);

        //arm drives:
        armDrive.setDirection(DcMotor.Direction.FORWARD);
        armDrive2.setDirection(DcMotor.Direction.FORWARD);
        otherSlide.setDirection(DcMotor.Direction.REVERSE);
        //linearSlide.setDirection(DcMotor.Direction.REVERSE);
        //servo for the grabber
        grabberServo.setDirection(Servo.Direction.FORWARD);
        //For the the claw thingy ma bober
        //rightClaw.setDirection(Servo.Direction.FORWARD);
        //leftClaw.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.FORWARD);
        leftServo.setDirection(Servo.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            if (gamepad2.right_bumper) {
                armDrive.setPower(-0.85);
            } else if (gamepad2.left_bumper) {
                armDrive.setPower(0);
            }
            if (gamepad2.dpad_down) {
                door_opener_servo.setPosition(-0.648);
            } else if (gamepad2.dpad_up) {
                door_opener_servo.setPosition(0.648);
            }
            if (gamepad2.b) {
                rightServo.setPosition(0.47); leftServo.setPosition(0.47);//flat. lower number= more back tilt
            }
            else if (gamepad2.y) {//tilt
                rightServo.setPosition(0.12);
                leftServo.setPosition(0.12);
            }
            else if (gamepad2.a) {
                rightServo.setPosition(0.44);
                leftServo.setPosition(0.44);
            }
            else if (gamepad2.x) {//right most button
                rightServo.setPosition(0.12);
                leftServo.setPosition(0.12);
                sleep(500);
                door_opener_servo.setPosition(0.648);
                sleep(500);
                rightServo.setPosition(0.47);
                leftServo.setPosition(0.47);
            }
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x; // should make mecanum do good( added an "-")
            double yaw = gamepad1.right_stick_x;
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            double slidePower = gamepad2.left_trigger - gamepad2.right_trigger;
            slidePower = Range.clip(slidePower, -1.0, 1.0);
            double slidePower2 = gamepad2.left_trigger - gamepad2.right_trigger;
            slidePower2 = Range.clip(slidePower2, -1.0, 1.0);
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            armDrive2.setPower(slidePower);
            otherSlide.setPower(slidePower2);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("encoder", armDrive.getCurrentPosition());
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower, );
            telemetry.update();
        }
    }}