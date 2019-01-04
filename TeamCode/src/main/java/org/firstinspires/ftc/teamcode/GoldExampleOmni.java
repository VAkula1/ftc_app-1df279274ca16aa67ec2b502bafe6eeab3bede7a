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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="GoldExampleOmni", group="DogeCV")
public class GoldExampleOmni extends OpMode {
    private GoldAlignDetector detector;
    double test = 0;
    int costl = 0;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor left_rear = null;
    private DcMotor right_rear = null;
    private DcMotor left_front = null;
    private DcMotor right_front = null;


    @Override
    public void init() {
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");


        left_rear = hardwareMap.get(DcMotor.class, "left_rear");
        right_rear = hardwareMap.get(DcMotor.class, "right_rear");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");

        left_rear.setDirection(DcMotor.Direction.FORWARD);
        right_rear.setDirection(DcMotor.Direction.REVERSE);
        left_front.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);


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


    }

    /*private void kick() {
        mecanumDrive_Cartesian(0, 0, 1);
        SystemClock.sleep(2000);
        mecanumDrive_Cartesian(0, 1, 0);
        SystemClock.sleep(2000);
    }
    public void vali() {

    }*/


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


    @Override
    public void init_loop() {
    }


    @Override
    public void start() { runtime.reset(); }

    //из-за беды с координатами на дисплее и в коде, они отличаются
    @Override
    public void loop() {
     //   boolean pos;
            telemetry.addData("IsAligned", detector.getAligned());
            telemetry.addData("X Pos", detector.getXPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            boolean pos = detector.getAligned();
            double x = detector.getXPosition();
            telemetry.update();

        if (test==0) {
            if (x < 250) {
                mecanumDrive_Cartesian(0, 0.5, 0);
                costl = 800;
            } else if (x > 360) {
                mecanumDrive_Cartesian(0, -0.5, 0);

            } else {
                mecanumDrive_Cartesian(0, 0, 0);
                costl = 400;
                test++;
            }
        }



            /*mecanumDrive_Cartesian(0,0, -1);
            SystemClock.sleep(2000);}*/

        if (test == 1) {
            /*telemetry.addData("pos", pos);
            telemetry.update();*/
            //detector.disable();
            mecanumDrive_Cartesian(0, 0, -0.9);
            SystemClock.sleep(900);
            mecanumDrive_Cartesian(0, 0.3, 0);
            SystemClock.sleep(2000);
            mecanumDrive_Cartesian(0, -0.3, 0);
            SystemClock.sleep(700);
            test++;

        }

        if (test == 2) {
            mecanumDrive_Cartesian(0, 0, 0.9);
            SystemClock.sleep(900);
            mecanumDrive_Cartesian(0, 0.3, 0);
            SystemClock.sleep(300);

            mecanumDrive_Cartesian(0, 0, 0.9);
            SystemClock.sleep(400);
            mecanumDrive_Cartesian(0, 0.3, 0);
            SystemClock.sleep(5000);
            /*
            mecanumDrive_Cartesian(0, 0, -0.9);
            SystemClock.sleep(900);
            mecanumDrive_Cartesian(0, 0.3, 0);
            SystemClock.sleep(2000);
            mecanumDrive_Cartesian(0, -0.3, 0);
            SystemClock.sleep(700);*/
            test++;

        }

        if (test==3){
            mecanumDrive_Cartesian(0,0,0);

        }







}

        @Override
        public void stop() {
        }
    }
