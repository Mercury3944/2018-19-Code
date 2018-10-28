/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.ColorSensor;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DriveTest", group="Linear Opmode")
public class DriveTest extends LinearOpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor liftMotor = null;
    boolean resetMode = false;
    boolean xPressed = false;
    boolean upMode = false;
    boolean aPressed = false;
    boolean downMode = false;
    boolean yPressed = false;
    boolean shuffleRight = false;
    boolean rightPressed = false;
    int indexMove = 0;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // get a reference to our ColorSensor object.

        // Set the LED in the beginning

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;


//            if (gamepad1.b && pointTurnTrigger) {
//                pointTurn = !pointTurn;
//                pointTurnTrigger = false;
//                telemetry.addData("button", pointTurn);
//            } else {
//                pointTurnTrigger = true;
//            }

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.left_stick_y;
            double turn =  gamepad1.right_stick_x;
            rightPower = Range.clip(drive + turn, -1.0, 1.0) ;
            leftPower = Range.clip(drive - turn, -1.0, 1.0) ;

            if (gamepad1.right_bumper) {
                leftPower /= 5.0;
                rightPower /= 5.0;
            }
            liftMotor.setPower(0);
            if (gamepad1.x && !xPressed) {
                xPressed = true;
                resetMode = !resetMode;
                if (!resetMode) {
                    liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            } else if (!gamepad1.x) {
                xPressed = false;
            }

            if (gamepad1.dpad_right && !rightPressed) {
                rightPressed = true;
                rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shuffleRight = true;

            } else {
                rightPressed = false;
            }

            if (indexMove == 0 && rightDrive.getCurrentPosition() >= -100 && shuffleRight) {
                leftPower = 0.2;
            } else {
                leftPower = 0;
                shuffleRight = false;
            }
            telemetry.addData("right", "value (%d)", rightDrive.getCurrentPosition());
            if (gamepad1.a && !aPressed) {
                aPressed = true;
                upMode = false;
                downMode = !downMode;
            } else {
                aPressed = false;
            }

            if (gamepad1.y && !yPressed) {
                yPressed = true;
                downMode = false;
                upMode = !upMode;
            } else {
                yPressed = false;
            }

            if ((upMode || gamepad1.right_trigger > 0)) {
                liftMotor.setPower(1.0);
            } else if ((downMode || gamepad1.left_trigger > 0)) {
                liftMotor.setPower(-1.0);
            }
            if (liftMotor.getCurrentPosition() <= -4784 || liftMotor.getCurrentPosition() >= 0) {
                upMode = false;
                downMode = false;
            }

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            // Show the elapsed game time and wheel power. -5849, 645
            telemetry.addData("Mode", "resetMode " + shuffleRight);
            if (upMode) {
                telemetry.addData("Lift", "up");
            } else if (downMode) {
                telemetry.addData("Lift", "down");
            } else {
                telemetry.addData("Lift", "none");
            }
            telemetry.addData("Lift", "value (%d)", liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
