package org.firstinspires.ftc.teamcode.EncoderDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EncoderDrive.EncoderLib;

@TeleOp(name="TELEOPEncoder", group="Iterative Opmode")
public class TELEOPEncoder extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    EncoderLib bot = new EncoderLib();
    @Override
    public void init() { bot.init(hardwareMap);}
    @Override
    public void init_loop() { }
    @Override
    public void start() {
        runtime.reset();
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void loop() {

        double x;double y; double r;

        bot.tick[0] = bot.left_front.getCurrentPosition();
        bot.tick[1] = bot.right_front.getCurrentPosition();
        bot.tick[2] = bot.left_rear.getCurrentPosition();
        bot.tick[3] = bot.right_rear.getCurrentPosition();

        bot.curTime = runtime.time();
        bot.SetDC(false,gamepad1.x,gamepad1.y,gamepad1.a,gamepad1.b);

        x = -gamepad1.right_stick_x; y =gamepad1.left_stick_y; r =-gamepad1.left_stick_x;
//        for(int nomOfMotor=0;nomOfMotor<4;nomOfMotor++){
//            bot.getSPD(nomOfMotor);
//        }

        bot.getSPD();

//        telemetry.addData("deltaTick[0]",Math.abs(bot.tick[0]-bot.lTick[0]));
//        telemetry.addData("deltaTick[1]",Math.abs(bot.tick[1]-bot.lTick[1]));
//        telemetry.addData("deltaTick[2]",Math.abs(bot.tick[2]-bot.lTick[2]));
//        telemetry.addData("deltaTick[3]",Math.abs(bot.tick[3]-bot.lTick[3]));
//
//        telemetry.addData("tickA l",bot.lTick[0]);
//        telemetry.addData("tickB l",bot.lTick[1]);
//        telemetry.addData("tickC l",bot.lTick[2]);
//        telemetry.addData("tickD l",bot.lTick[3]);
//
//        telemetry.addData("SPDA: ",bot.curSPD[0]);
//        telemetry.addData("SPDB: ",bot.curSPD[1]);
//        telemetry.addData("SPDC: ",bot.curSPD[2]);
//        telemetry.addData("SPDD: ",bot.curSPD[3]);

        telemetry.addData("x: ",x);
        telemetry.addData("y: ",y);
        telemetry.addData("r: ",r);
        telemetry.addData("time: ",bot.curTime);
        telemetry.update();
        //bot.MechanumY(gamepad1.right_stick_y);

        bot.MecanumDriveCartesian(x,y,r);

        double plePower =-0.4*gamepad2.right_stick_y;
        double vdvig = gamepad2.left_stick_x;

        bot.Hook(gamepad1.dpad_up,gamepad1.dpad_down);
        bot.Pickpos(gamepad2.y,gamepad2.a);
        bot.Slider(plePower);
        bot.Vdvig(vdvig);

        bot.ServPick(gamepad2.dpad_left,gamepad2.dpad_right);
        bot.servoLeft.setPosition(bot.leftpos);
        bot.servoRight.setPosition(bot.rightpos);
        bot.servoPick.setPosition(bot.pickpos);

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

    }
    @Override
    public void stop() {
    }
}