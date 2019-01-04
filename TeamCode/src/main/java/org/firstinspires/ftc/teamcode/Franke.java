package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class Franke {
    HardwareMap hardwareMap =  null;

    private ElapsedTime runtime = new ElapsedTime();
    DcMotor left_rear = null;
    DcMotor right_rear = null;
    DcMotor left_front = null;
    DcMotor right_front = null;

    DcMotor pleDrive = null;
    DcMotor vdvig = null;

    DcMotor lift = null;

    DcMotor sosat = null;

//    DigitalChannel TouchLift;
//    DigitalChannel TouchVdvig;

    Servo servoLeft;

    double leftpos = 0;
    int costlV = 0;
    int costlVi = 0;
    double test=0;

    //библиотека функций
    public void init(HardwareMap hardwareMap) {
//        TouchLift = hardwareMap.get(DigitalChannel.class, "TouchLift");
//        TouchVdvig = hardwareMap.get(DigitalChannel.class, "TouchVdvig");

//        TouchLift.setMode(DigitalChannel.Mode.INPUT);
//        TouchVdvig.setMode(DigitalChannel.Mode.INPUT);


        left_rear = hardwareMap.get(DcMotor.class, "left_rear");
        right_rear = hardwareMap.get(DcMotor.class, "right_rear");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");

        pleDrive = hardwareMap.get(DcMotor.class, "ple_drive");
        vdvig = hardwareMap.get(DcMotor.class, "vdvig_drive");

        lift = hardwareMap.get(DcMotor.class, "lift");

        //servo = hardwareMap.get(Servo.class, "servo");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");

        sosat = hardwareMap.get(DcMotor.class, "sosat");

        left_rear.setDirection(DcMotor.Direction.FORWARD);
        right_rear.setDirection(DcMotor.Direction.REVERSE);
        left_front.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);
    }
    public void MecanumDrive_Cartesian(double x, double y, double rotation) {
        double wheelSpeeds[] = new double[4];

        wheelSpeeds[0] = x + y + rotation;
        wheelSpeeds[1] = -x + y - rotation;
        wheelSpeeds[2] = -x + y + rotation;
        wheelSpeeds[3] = x + y - rotation;

        normalize(wheelSpeeds);

        left_front.setPower(wheelSpeeds[0]*0.9);
        right_front.setPower(wheelSpeeds[1]*0.85);
        left_rear.setPower(wheelSpeeds[2]);
        right_rear.setPower(wheelSpeeds[3]*0.9);
    }
    private void normalize(double[] wheelSpeeds) {
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
        {
            for (int i = 0; i < wheelSpeeds.length; i++)
            {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }
    void LiftPow(boolean down,boolean up) {
        if (up) lift.setPower(1);
        else if (down)lift.setPower(-1);
        else lift.setPower(0);
//        if (up) {
//            lift.setPower(1);
//        } else if (down) {
//            lift.setPower(-1);
//        } else {
//            lift.setPower(0);
//        }
    }
    void Sosatel(boolean vSos,boolean viSos){
        if(vSos){
            costlV++;
            if(costlV%2==1) sosat.setPower(0.25);
            if(costlV%2==0) sosat.setPower(0);
            SystemClock.sleep(50);
//            sosat.setPower(0.5);
        }
        else if(viSos){
            costlVi++;
            if(costlVi%2==1) sosat.setPower(-0.25);
            if(costlVi%2==0) sosat.setPower(0);
            SystemClock.sleep(10);
//            sosat.setPower(-0.5);
        }
    }
    void Servak(boolean dpad_left,boolean dpad_right){
        if (dpad_left){
        servoLeft.setPosition(0.75);
//        servoRight.setPosition(rightpos-0.01);
    }
    else if (dpad_right) {
            servoLeft.setPosition(0);
//            servoRight.setPosition(rightpos+0.01);
    }}
    void Plecho (double plePower){
//        int a = 50;
//        double plePowerF = 0.6*plePower/50;
//        for(double plePowerL = 0.6*plePower/a;plePowerL<plePower;a--) {
//            test = plePowerL;

                pleDrive.setPower(plePower);


//        pleDrive.setPower(0.6*plePower);
    }
    void Vdvig (double vdvigPower){
        vdvig.setPower(vdvigPower);
    }}