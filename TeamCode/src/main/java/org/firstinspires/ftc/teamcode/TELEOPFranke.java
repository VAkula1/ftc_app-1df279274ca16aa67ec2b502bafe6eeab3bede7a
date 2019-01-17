package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Franke;
@TeleOp(name="TELEOPFranke", group="Iterative Opmode")
public class TELEOPFranke extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Franke bot = new Franke();
    @Override
    public void init() { bot.init(hardwareMap); }
    @Override
    public void init_loop() { }
    @Override
    public void start() {
        runtime.reset();
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void loop() {

        telemetry.addData("tickA",bot.tickA);
        telemetry.addData("tickB",bot.tickB);
        telemetry.addData("tickC",bot.tickC);
        telemetry.addData("tickD",bot.tickD);
        telemetry.update();

        bot.tickA = bot.left_front.getCurrentPosition();
        bot.tickB = bot.right_front.getCurrentPosition();
        bot.tickC = bot.left_rear.getCurrentPosition();
        bot.tickD = bot.right_rear.getCurrentPosition();
        if (gamepad1.x){
            bot.tickA=0;
            bot.tickB=0;
            bot.tickC=0;
            bot.tickD=0;
        }

        bot.MecanumDrive_Cartesian(-gamepad1.right_stick_x,gamepad1.right_stick_y,-gamepad1.left_stick_x);

        double plePower =-0.5*gamepad2.right_stick_y;
        double vdvig = 0.5*gamepad2.left_stick_x;

        bot.LiftPow(gamepad1.dpad_up,gamepad1.dpad_down);
        bot.Sosatel(gamepad2.y,gamepad2.a);
        bot.Servak(gamepad2.x,gamepad2.b);

        bot.Plecho(plePower);
        bot.Vdvig(vdvig);

//        if (bot.TouchLift.getState() == true) {
//            bot.lift.setPower(0);
//            telemetry.addData("TouchLift", "Is Not Pressed"); }
//        else {
//            telemetry.addData("TouchLift", "Is Pressed");
//        }
//
//        if (bot.TouchVdvig.getState() == true) {
//            telemetry.addData("TouchVdvig", "Is Not Pressed");
//            bot.vdvig.setPower(-0.5);}
//        else {
//            telemetry.addData("TouchVdvig", "Is Pressed");
 //        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Сервак",bot.leftpos);
        telemetry.update();
    }
    @Override
    public void stop() {
    }
}