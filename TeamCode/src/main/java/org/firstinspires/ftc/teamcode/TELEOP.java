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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TELEOP", group="Iterative Opmode")
public class TELEOP extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_rear = null;
    private DcMotor right_rear = null;
    private DcMotor left_front = null;
    private DcMotor right_front = null;

    private DcMotor pleDrive = null;
    private DcMotor vdvig = null;

    private DcMotor lift = null;

    private DcMotor sosat = null;

    //Servo servo;
    //double   position = 0.5;
    Servo   servoLeft;
    Servo   servoRight;

    double     leftpos = 0;
    double     rightpos = 0;

    @Override
    public void init() {








        telemetry.addData("Status", "Initialized");

        left_rear  = hardwareMap.get(DcMotor.class, "left_rear");
        right_rear = hardwareMap.get(DcMotor.class, "right_rear");
        left_front  = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");

        pleDrive  = hardwareMap.get(DcMotor.class, "ple_drive");
        vdvig = hardwareMap.get(DcMotor.class, "vdvig_drive");

        lift  = hardwareMap.get(DcMotor.class, "lift");

        //servo = hardwareMap.get(Servo.class, "servo");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        sosat  = hardwareMap.get(DcMotor.class, "sosat");



        left_rear.setDirection(DcMotor.Direction.FORWARD);
        right_rear.setDirection(DcMotor.Direction.REVERSE);
        left_front.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);

//        right_front.setPower(0);
//        left_front.setPower(0);
//        right_rear.setPower(0);
//        left_rear.setPower(0);
//        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        right_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void mecanumDrive_Cartesian(double x, double y, double rotation)
    {
        double wheelSpeeds[] = new double[4];

        wheelSpeeds[0] = x + y + rotation;
        wheelSpeeds[1] = -x + y - rotation;
        wheelSpeeds[2] = -x + y + rotation;
        wheelSpeeds[3] = x + y - rotation;


        normalize(wheelSpeeds);

        left_front.setPower(wheelSpeeds[0]*0.88);//
        right_front.setPower(wheelSpeeds[1]*0.85);//
        left_rear.setPower(wheelSpeeds[2]);
        right_rear.setPower(wheelSpeeds[3]*0.9);//
    }

    private void normalize(double[] wheelSpeeds)
    {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);

        for (int i = 1; i < wheelSpeeds.length; i++)
        {
            double magnitude = Math.abs(wheelSpeeds[i]);

            if (magnitude > maxMagnitude)
            {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0)
        {            for (int i = 0; i < wheelSpeeds.length; i++)
            {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }



    ////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void loop() {
//        telemetry.addData("Encoder",  Math.round(right_front.getCurrentPosition()));



        mecanumDrive_Cartesian(-gamepad1.right_stick_x,gamepad1.right_stick_y,-gamepad1.left_stick_x);

        double plePower = -gamepad2.right_stick_x;
        double vdvigPower = gamepad2.right_stick_y;
        double liftPower = 0;

        if (gamepad1.right_bumper) {
             liftPower = 1;
        }
        else if (gamepad1.left_bumper){
             liftPower = -1;
        }
        else {
            liftPower=0;
        }

        double sosatPower = 0;

        if(gamepad2.a){
             sosatPower=1;
        }
        else if(gamepad2.b){
             sosatPower=-1;
        }
        else {
            sosatPower = 0;
        }

        if (gamepad2.dpad_up){
            leftpos +=0.01;
            rightpos -=0.01;
        }
        else if (gamepad2.dpad_down) {
            leftpos -=0.01 ;
            rightpos +=0.01 ;
        }
        /*if (gamepad2.a){
            position +=0.01;
        }
        else if (gamepad2.b) {
            position -= 0.01;
        }
        servo.setPosition(position);*/
        servoLeft.setPosition(leftpos);
        servoRight.setPosition(rightpos);

        pleDrive.setPower(plePower/**+0.25upPower+0.25*horPower*/);
        vdvig.setPower(vdvigPower/**+0.25upPower+0.5*horPower*/);

        lift.setPower(liftPower);
        sosat.setPower(sosatPower);

//      telemetry.addData("Encoder", - Math.round(left_front.getCurrentPosition()));
//        telemetry.addData("Encoder2",  Math.round(right_rear.getCurrentPosition()));
//        telemetry.addData("Encoder3",  Math.round(left_rear.getCurrentPosition()));
        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "left (%.2f), right (%.2f)");
//        telemetry.addData("lf",left_front.getPower());
//        telemetry.addData("rf",right_front.getPower());
//        telemetry.addData("lr",left_rear.getPower());
//        telemetry.addData("rr",right_rear.getPower());
    }
    @Override
    public void stop() {
    }
}
