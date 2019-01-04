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

import android.os.SystemClock;

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
 * It includes all t he skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="FR_TIME_Blizko", group="DogeCV")
public class FR_TIME_Blizko extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor lift = null;
    private GoldAlignDetector detector;
    double test = 0;
    int costl = 0;
    double delta;
    private DcMotor sosat = null;
    private DcMotor pleDrive = null;
    private DcMotor vdvig = null;


    int tick;
    final int TickM = 5242;
    int AgileCon = 90;
    double TimeCon = 2000;
    double Timing = 2000;
    double TimeBoch = 4;

    private DcMotor left_rear = null;
    private DcMotor right_rear = null;
    private DcMotor left_front = null;
    private DcMotor right_front = null;





    public void mecanumDrive_Cartesian(double x, double y, double rotation) {
        double wheelSpeeds[] = new double[4];

        wheelSpeeds[0] = x + y + rotation;
        wheelSpeeds[1] = -x + y - rotation;
        wheelSpeeds[2] = -x + y + rotation;
        wheelSpeeds[3] = x + y - rotation;

        normalize(wheelSpeeds);

        left_front.setPower(wheelSpeeds[0]);
        right_front.setPower(wheelSpeeds[1]);
        left_rear.setPower(wheelSpeeds[2]);
        right_rear.setPower(wheelSpeeds[3]);
    }


    private void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);

        for (int i = 1; i < wheelSpeeds.length; i++) {
            double magnitude = Math.abs(wheelSpeeds[i]);

            if (magnitude > maxMagnitude) {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }



    private void move(double m) {
        double x;
        if(m>0) {
                mecanumDrive_Cartesian(0, -1, 0);
                x = m*(Timing);
                sleep(Math.round(x));
                mecanumDrive_Cartesian(0, 0, 0);
        }
        else {
            m = m*(-1);
            mecanumDrive_Cartesian(0, 1, 0);
            x = m*(Timing);
            sleep(Math.round(x));
            mecanumDrive_Cartesian(0, 0, 0);
            }
        sleep(1000);
    }



    private  void bochkom (double m){
        double x;
        if(m>0) {
                mecanumDrive_Cartesian(1, 0, 0);
                x = m*(TimeBoch);
                sleep(Math.round(x));
                mecanumDrive_Cartesian(0, 0, 0);
        }
        else {
            m = m*(-1);
            mecanumDrive_Cartesian(-1, 0, 0);
            x = m*(TimeBoch);
            sleep(Math.round(x));
            mecanumDrive_Cartesian(0, 0, 0);
            }
        sleep(1000);
    }



    private void turn(double agile){
        if(agile>0) {
            mecanumDrive_Cartesian(0, 0, 0.5);
            sleep(Math.round(agile) / AgileCon * Math.round(TimeCon));
            mecanumDrive_Cartesian(0, 0, 0);
        }
        else {
            agile = agile*-1;
            mecanumDrive_Cartesian(0, 0, -0.5);
            sleep(Math.round(agile) / AgileCon * Math.round(TimeCon));
            mecanumDrive_Cartesian(0, 0, 0);
        }
        sleep(1000);
    }



    private void totem(){
        sosat.setPower(-1);
        sleep(700);
        sosat.setPower(0);
    }




    @Override
    public void runOpMode() {
        left_rear = hardwareMap.get(DcMotor.class, "left_rear");
        right_rear = hardwareMap.get(DcMotor.class, "right_rear");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");

        left_rear.setDirection(DcMotor.Direction.FORWARD);
        right_rear.setDirection(DcMotor.Direction.REVERSE);
        left_front.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);

        lift  = hardwareMap.get(DcMotor.class, "lift");

        sosat  = hardwareMap.get(DcMotor.class, "sosat");
        pleDrive  = hardwareMap.get(DcMotor.class, "ple_drive");

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; //диапазон, в котором будет выровнен золотой объект. (Представлено зелеными полосами в предварительном просмотре)
        detector.alignPosOffset = 0; // расстояние от центральной рамки до смещения этой зоны выравнивания.
        detector.downscale = 0.4; // насколько уменьшить входные кадры

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        runtime.reset();
        waitForStart();


        // run until the end of the match (driver presses STOP)


            // Setup a variable for each drive wheel to save power level for telemetry
            telemetry.addData("IsAligned", detector.getAligned());
            telemetry.addData("X Pos", detector.getXPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            boolean pos = detector.getAligned();
            double x = detector.getXPosition();
            double liftPower;
            telemetry.update();
            //спуск
        lift.setPower(1);
        sleep(2800);
        lift.setPower(0);
        move(-0.1);
        turn(-90);
        move(0.45);
        turn(90);
        //bochkom(0.4);
        //подъезд к правому
        move(-0.27);

        //Скан
        delta = getRuntime();
        while (opModeIsActive() && !pos){
            mecanumDrive_Cartesian(0, -0.5, 0);
            pos = detector.getAligned();
        }
        delta = (getRuntime() - delta)*Timing;
        turn(-70);
        move(0.2);
        move(-0.2);
        turn(70);
        mecanumDrive_Cartesian(0, 1, 0);
        sleep(Math.round(delta));

        move(0.37);
        turn(-45);
        move(0.6);

        //totem();

        move(-1.9);
        detector.disable();
    }
}

