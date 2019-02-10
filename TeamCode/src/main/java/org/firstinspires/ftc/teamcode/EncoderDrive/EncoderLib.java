package org.firstinspires.ftc.teamcode.EncoderDrive;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderLib {
    HardwareMap hardwareMap =  null;
    public int TickSM =21;
    public int TickGR =13;
    double CD[] = new double[4];
    double CD0;double CD1;double CD2;double CD3;
    double lastTime;double curTime;
    double global;

    public DcMotor left_rear = null;
    public DcMotor right_rear = null;
    public DcMotor left_front = null;
    public DcMotor right_front = null;

    public DcMotor leftSlider = null;
    public DcMotor rightSlider = null;
    public DcMotor giraffe = null;
    public DcMotor hook = null;

    public double[] tick = new double[4];
    public double[] lTick = new double[4];
    public double[] curSPD = new double[4];
    double inWheelPows[] = new double[4];
    double lasWheelPows[] = new double[4];
    double curWheelPows[] = new double[4];
    double norWheelPows[] = new double[4];

//    DigitalChannel TouchLift;
//    DigitalChannel TouchVdvig;

    public Servo servoLeft;
    public Servo servoRight;
    public Servo servoPick;
    public double leftpos;
    public double rightpos;
    public double pickpos;
    int costlV = 0;
    int costlVi = 0;
    //библиотека функций//////////////////////////////////////////////////////////////////////////////
    public void init(HardwareMap hardwareMap) {
        leftpos = 0;
        rightpos = 1;
        pickpos = 0;
        left_rear = hardwareMap.get(DcMotor.class, "left_rear");
        right_rear = hardwareMap.get(DcMotor.class, "right_rear");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");

        right_front.setPower(0);
        left_front.setPower(0);
        right_rear.setPower(0);
        left_rear.setPower(0);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlider = hardwareMap.get(DcMotor.class, "leftSlider");
        rightSlider = hardwareMap.get(DcMotor.class, "rightSlider");
        giraffe = hardwareMap.get(DcMotor.class, "giraffe");
        hook = hardwareMap.get(DcMotor.class, "hook");
        //servo = hardwareMap.get(Servo.class, "servo");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoPick = hardwareMap.get(Servo.class, "servoPick");

        left_rear.setDirection(DcMotor.Direction.FORWARD);
        right_rear.setDirection(DcMotor.Direction.REVERSE);
        left_front.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        lTick[0]=0;lTick[1]=0;lTick[2]=0;lTick[3]=0;
        curSPD[0]=0;curSPD[1]=0;curSPD[2]=0;curSPD[3]=0;
        CD[0] =0;CD[1] =0;CD[2] =0;CD[3] =0;
        CD0 = 0;CD1 = 0;CD2 = 0;CD3 = 0;
    }

    public void getSPD(){
        if (curTime-lastTime>0.1){
            for(int nomOfEncoder=0;nomOfEncoder<4;nomOfEncoder++){
                curSPD[nomOfEncoder]=Math.abs((tick[nomOfEncoder]-lTick[nomOfEncoder])/(curTime-lastTime));
                lTick[nomOfEncoder]=tick[nomOfEncoder];lasWheelPows[nomOfEncoder]=inWheelPows[nomOfEncoder];
            }
            lastTime=curTime;
            }
    }

    public int getDeltaSM(int nomOfEncoder){
        getSPD();
        return ConvertToSantiSM((int)Math.abs( (tick[nomOfEncoder]-lTick[nomOfEncoder])));
    }
    public int getDeltaGrad(int nomOfEncoder){
        getSPD();
        return ConvertToGrads((int)Math.abs( (tick[nomOfEncoder]-lTick[nomOfEncoder])));
    }
    public int ConvertToSantiSM(int ticks){
        int toRet;
        toRet=ticks/TickSM;
        return toRet;
    }
    public int ConvertToGrads(int ticks){
        int toRet;
        toRet=ticks/TickGR;
        return toRet;
    }

    public void SetDC(boolean mode,boolean x,boolean y,boolean a,boolean b){
        if (mode) {

            if (x) {CD0 =-0.25;CD1 =0.25;CD2 =0.25;CD3 =-0.25;}
            if (y) {CD0 =0.25;CD1 =0.25;CD2 =0.25;CD3 =0.25;}
            if (a) {CD0 =-0.25;CD1 =-0.25;CD2 =-0.25;CD3 =-0.25; }
            if (b) {CD0 =0.25;CD1 =-0.25;CD2 =-0.25;CD3 =0.25;}
        }
        else CD0 =0;CD1 =0;CD2 =0;CD3 =0;

    }

    public void MecanumDriveCartesian(double x, double y, double rotation) {
        double wheelSpeeds[] = new double[4];

        wheelSpeeds[0] =-y-x+rotation ;//-y-x ;
        wheelSpeeds[1] =y-x+rotation ;//y-x;
        wheelSpeeds[2] =y-x-rotation ;//y-x;
        wheelSpeeds[3] =-y-x-rotation ;//-y-x;

        normalize(wheelSpeeds);

        left_front.setPower(wheelSpeeds[0]);
        right_front.setPower(wheelSpeeds[1]);
        left_rear.setPower(wheelSpeeds[2]);
        right_rear.setPower(wheelSpeeds[3]);
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

    public void Hook(boolean up,boolean down) {
        if (down) hook.setPower(1);
        else if (up)hook.setPower(-1);
        else hook.setPower(0);
    }

    public void ServPick(boolean left,boolean right){
        if (left){pickpos =1;}
        if (right){pickpos =0;}
    }

    public void Pickpos(boolean up,boolean down){
        if (up){leftpos =0.6;rightpos =0.4;
            if(leftpos >0.6)leftpos=0.6;
            if(rightpos <0.4)rightpos =0.4;
        }
        if (down){leftpos =0;rightpos =1;
            if(leftpos <0)leftpos=0;
            if(rightpos >1)rightpos =1;
        }
    }
    public void Slider (double sliderPower){
        leftSlider.setPower(sliderPower);
        rightSlider.setPower(-sliderPower);

    }
    public void Vdvig (double vdvigPower){
        giraffe.setPower(-vdvigPower);
    }
  }


//    public void getSPD(int nomOfEncoder){
//        if (curTime-lastTime>0.1){
//            curSPD[nomOfEncodor]=Math.abs((tick[nomOfEncodor]-lTick[nomOfEncodor])/(curTime-lastTime));
//            lastTime=curTime;lTick[nomOfEncodor]=tick[nomOfEncodor];
//        }
//    }
//    public void MecanumDrive_Cartesian(double x, double y, double rotation) {
//
//        inWheelPows[0] = x + y + rotation;
//        inWheelPows[1] = -x + y - rotation;
//        inWheelPows[2] = -x + y + rotation;
//        inWheelPows[3] = x + y - rotation;
//
//        normalize(lasWheelPows);
//
//        left_front.setPower(curWheelPows[0]);
//        right_front.setPower(curWheelPows[1]);
//        left_rear.setPower(curWheelPows[2]);
//        right_rear.setPower(curWheelPows[3]);
//    }
//
//    private void normalize(double[] forNormal) {
//        global = curSPD[0];
//        for(int nomOfMotor=0;nomOfMotor<4;nomOfMotor++){
//            curWheelPows[nomOfMotor]=forNormal[nomOfMotor];
//            if(curSPD[nomOfMotor]>global){curWheelPows[nomOfMotor]=forNormal[nomOfMotor]*0.6;
//            }
//            else if(curSPD[nomOfMotor]<global){curWheelPows[nomOfMotor]=forNormal[nomOfMotor]*1.4;
//            }
//            lasWheelPows[nomOfMotor] = curWheelPows[nomOfMotor];
//        }
//    }

//    public void getSPD(){
//        if (curTime-lastTime>0.1){
//            curSPDA=Math.abs((tickA-lTickA)/(curTime-lastTime));
//            curSPDB=Math.abs((tickB-lTickB)/(curTime-lastTime));
//            curSPDC=Math.abs((tickC-lTickC)/(curTime-lastTime));
//            curSPDD=Math.abs((tickD-lTickD)/(curTime-lastTime));
//            lastTime=curTime;lTickA=tickA;lTickB=tickB;lTickC=tickC;lTickD=tickD;
//        }
//    }

//    public double NormlazeSPD(){
//        double
//        if
//    }