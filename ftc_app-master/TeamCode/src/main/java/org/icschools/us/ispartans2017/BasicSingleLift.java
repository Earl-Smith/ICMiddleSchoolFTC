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

package org.icschools.us.ispartans2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Basic draft", group="Linear Opmode")
@Disabled
public class BasicSingleLift extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private DcMotor gripperMotor = null;
    private Servo   leftServo = null;
    private Servo   rightServo = null;
    private boolean isFirst = false;

    @Override
    public void runOpMode() {
        RunNormal();
    }
    private void RunNormal(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor  = hardwareMap.get(DcMotor.class, "arm_drive");
        gripperMotor = hardwareMap.get(DcMotor.class, "gripper_drive");
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  .5 * gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1, 1) ;
            rightPower   = Range.clip(drive - turn, -1, 1) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            /*double armPower;
            double gripperPower;
            double armLift = gamepad2.left_stick_x;
            double gripperLift = gamepad2.right_stick_x;
            armPower = Range.clip(armLift, -1, 1);
            gripperPower = Range.clip(gripperLift, -1, 1);

            armMotor.setPower(armPower);
            gripperMotor.setPower(gripperPower);
*/

            if(gamepad2.a)
            {
                ArmLift();
            }
            else {
                StopArm();
            }
            if(gamepad2.b){
                ArmFall();
            }
            else
            {
                StopArm();
            }
            if(gamepad2.x)
            {
                GripperUp();
            }
            else{
                StopGripper();
            }
            if(gamepad2.y)
            {
                GripperDown();
            }
            else{
                StopGripper();
            }
            if(gamepad2.right_bumper)
            {
                /*boolean isMotorInactive = true;
                telemetry.addData("Bumper","Pressed");
                telemetry.update();

                isMotorInactive = (testServo.getPosition()==.5 || testServo.getPosition()==0);

                if(isFirst && isMotorInactive) {
                    testServo.setPosition(.5);
                    isFirst = !isFirst;
                }
                else if(isMotorInactive)
                {
                    testServo.setPosition(0);
                    isFirst = !isFirst;
                }*/
                //testServo.setPosition(1);
                leftServo.setPosition(1);
                rightServo.setPosition(.01);

            }
            else
            {
                leftServo.setPosition(.5);
                rightServo.setPosition(.5);
            }
            if(gamepad2.left_bumper)
            {
                leftServo.setPosition(.01);
                rightServo.setPosition(1);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
    private void StopGripper(){
        gripperMotor.setPower(0);
    }
    private void GripperUp(){
        gripperMotor.setDirection(DcMotor.Direction.FORWARD);
        gripperMotor.setPower(.45);

    }
    private void GripperDown(){
        gripperMotor.setDirection(DcMotor.Direction.REVERSE);
        gripperMotor.setPower(.45);
    }
    private void ArmLift(){
        //armMotor.setTargetPosition(630);
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor.setPower(.5);

        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setPower(.75);
    }
    private void ArmFall()
    {
        //armMotor.setTargetPosition(0);
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor.setPower(.5);

        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setPower(.75);
    }
    private void StopArm(){
        armMotor.setPower(0);
    }
}
