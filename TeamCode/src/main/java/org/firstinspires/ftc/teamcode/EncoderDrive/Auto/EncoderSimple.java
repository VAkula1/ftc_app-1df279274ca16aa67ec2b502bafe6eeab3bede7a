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

package org.firstinspires.ftc.teamcode.EncoderDrive.Auto;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.EncoderDrive.EncoderLib;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name="EncoderSimple", group="DogeCV")
public class EncoderSimple extends LinearOpMode {
    // EN = Encoder
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    EncoderLib aut = new EncoderLib();
    private void UpDateTM(){

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("getDeltaSM 0: ",aut.getDeltaSM(0));
        telemetry.addData("getDeltaSM 1: ",aut.getDeltaSM(1));
        telemetry.addData("getDeltaSM 2: ",aut.getDeltaSM(2));
        telemetry.addData("getDeltaSM 3: ",aut.getDeltaSM(3));
        telemetry.update();
    }
    private void UpDateEN(){
        aut.tick[0] = aut.left_front.getCurrentPosition();
        aut.tick[1] = aut.right_front.getCurrentPosition();
        aut.tick[2] = aut.left_rear.getCurrentPosition();
        aut.tick[3] = aut.right_rear.getCurrentPosition();}
    private void ResetEN(){
        aut.right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aut.left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aut.right_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aut.left_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        aut.right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aut.left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aut.right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aut.left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        UpDateEN();
    }
    private void setAllTZero(){
        aut.MecanumDriveCartesian(0, 0, 0);
    }

    private void MoveToFront(double inSM){
        while ((aut.getDeltaSM(0)<inSM)&&(opModeIsActive())){
            aut.MecanumDriveCartesian(1, 0, 0);
            UpDateEN();
            UpDateTM();
        }
        setAllTZero();
        ResetEN();
    }
    private void MoveToBack(double inSM){
        while ((aut.getDeltaSM(0)<inSM)&&(opModeIsActive())){
            aut.MecanumDriveCartesian(-1, 0, 0);
            UpDateEN();
            UpDateTM();
        }
        setAllTZero();
        ResetEN();
    }
    private void MoveBoch(double inSM, boolean right){
        if (right) {
            while ((aut.getDeltaSM(0) < inSM*1.3) && (opModeIsActive())) {
                aut.MecanumDriveCartesian(0, 1, 0);
                UpDateEN();
                UpDateTM();
            }
        }
        else {
            while ((aut.getDeltaSM(0) < inSM*1.3) && (opModeIsActive())) {
                aut.MecanumDriveCartesian(0, -1, 0);
                UpDateEN();
                UpDateTM();
            }
        }
        setAllTZero();
        ResetEN();
    }
    private void Turn (double inGrad, boolean right){
        if (right) {
            while ((aut.getDeltaGrad(0) < inGrad) && (opModeIsActive())) {
                aut.MecanumDriveCartesian(0, 0, -1);
                UpDateEN();
                UpDateTM();
            }
        }
        else {
            while ((aut.getDeltaGrad(0) < inGrad) && (opModeIsActive())) {
                aut.MecanumDriveCartesian(0, 0, 1);
                UpDateEN();
                UpDateTM();
            }
        }
        setAllTZero();
        ResetEN();
    }
    @Override
    public void runOpMode() {
        aut.init(hardwareMap);
        aut.tick[0] = aut.left_front.getCurrentPosition();
        aut.tick[1] = aut.right_front.getCurrentPosition();
        aut.tick[2] = aut.left_rear.getCurrentPosition();
        aut.tick[3] = aut.right_rear.getCurrentPosition();

        runtime.reset();
        waitForStart();


        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            UpDateTM();
            Turn(90,true);
            Turn(180,true);
            Turn(270,true);
            Turn(360,false);
        }
    }
}

