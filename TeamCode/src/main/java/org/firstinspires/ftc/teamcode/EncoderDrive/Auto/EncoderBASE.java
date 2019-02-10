package org.firstinspires.ftc.teamcode.EncoderDrive.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.teamcode.EncoderDrive.EncoderLib;
import org.firstinspires.ftc.teamcode.WebCa.DetectionLib;


@Autonomous(name="EncoderBASE", group="DogeCV")
public class EncoderBASE extends LinearOpMode {

    // EN = Encoder
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = detector.VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        detector.vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        detector.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, detector.vuforia);
        detector.tfod.loadModelFromAsset(detector.TFOD_MODEL_ASSET, detector.LABEL_GOLD_MINERAL, detector.LABEL_SILVER_MINERAL);
    }

    double dopDist = 45;
    boolean TuchKru =false;
    private ElapsedTime runtime = new ElapsedTime();
    DetectionLib detector = new DetectionLib();
    EncoderLib aut = new EncoderLib();
    private void Cold(){
        sleep(1000);
    }
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
        Cold();
    }
    private void MoveToBack(double inSM){
        while ((aut.getDeltaSM(0)<inSM)&&(opModeIsActive())){
            aut.MecanumDriveCartesian(-1, 0, 0);
            UpDateEN();
            UpDateTM();
        }
        setAllTZero();
        ResetEN();
        Cold();
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
        Cold();
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
        Cold();
    }
    private void Kicking(){
        MoveToFront(dopDist);
        Turn(70,true);
        MoveToFront(10);
        MoveToBack(10);
        Turn(70,false);
        MoveToFront(90-dopDist);
    }
    private void Search(){
        Turn(20,true);
        detector.SetXPos();
        if(detector.goldHere){
            Turn(20,false);
            dopDist = 90;
        }
        else{
            Turn(40,false);
            detector.SetXPos();
            if(detector.goldHere){
                Turn(20,true);
            }
            else{
                dopDist = 0;
                Turn(20,true);
            }
        }
    }

    private void Lending(){
        //while (TuchKru) aut.Hook(false,true);
        MoveBoch(5,true);
        Search();
    }// Первая сладия
    private void ToTravel(){
        MoveToFront(15);
        Turn(90,true);
        MoveToBack(60);
        Kicking();
        MoveToFront(25);
    }//Вторая стадия
    private void TotemLoading(){
        Turn(90,false);
        MoveToFront(80);
        Turn(45,false);
        MoveToFront(45);
        aut.Pickpos(true,false);
        sleep(1100);
        aut.Pickpos(false,true);
        Turn(180,true);
    }//Третья стадия
    private void KratStoping(){
        MoveToFront(180);
//        aut.Vdvig(0.5);
//        sleep(500);
//        aut.Vdvig(0);
    }//Четвёртая стадия
    @Override
    public void runOpMode() {
        detector.IninDetector();
        initVuforia();
        initTfod();
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

