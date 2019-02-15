package org.firstinspires.ftc.teamcode.EncoderDrive.Auto;

import android.os.SystemClock;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.EncoderDrive.Auto.AutoLib;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="EncoderSimple", group="DogeCV")
public class EncoderSimple extends LinearOpMode {
    // EN = Encoder
    double dopDist = 45;
    boolean TuchKru =false;

    private ElapsedTime runtime = new ElapsedTime();
    AutoLib aut = new AutoLib();
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
    private void Kicking(){
        MoveToBack(dopDist);
        Turn(90,false);
        MoveToFront(10);
        MoveToBack(10);
        Turn(90,true);
        MoveToFront(dopDist);
    }
    private void Search(){
        //setuping dopDist
    }

    private void Lending(){

        while (TuchKru) aut.Hook(false,true);
        MoveBoch(5,true);
    }// Первая сладия
    private void ToTravel(){
        Search();
        MoveToFront(35);
        Turn(90,true);
        Kicking();
        MoveToFront(25);
    }//Вторая стадия
    private void TotemLoading(){
        Turn(45,false);
        MoveToFront(100);
        Turn(90,true);
        MoveToFront(120);
        aut.Pickpos(true,false);
        sleep(1100);
        aut.Pickpos(false,true);
    }//Третья стадия
    private void KratStoping(){
        MoveToBack(180);
        Turn(90,true);
        aut.Vdvig(0.5);
        sleep(500);
        aut.Vdvig(0);
    }//Четвёртая стадия
    @Override
    public void runOpMode() {
        aut.init(hardwareMap);
        aut.tick[0] = aut.left_front.getCurrentPosition();
        aut.tick[1] = aut.right_front.getCurrentPosition();
        aut.tick[2] = aut.left_rear.getCurrentPosition();
        aut.tick[3] = aut.right_rear.getCurrentPosition();

        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            UpDateTM();
            Lending();
            ToTravel();
            TotemLoading();
            KratStoping();
            break;
        }
    }
}

