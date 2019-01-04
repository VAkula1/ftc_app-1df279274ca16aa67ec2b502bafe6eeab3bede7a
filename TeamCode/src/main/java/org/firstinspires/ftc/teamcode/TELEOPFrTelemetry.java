package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TELEOPFrTelemetry", group="Iterative Opmode")
public class TELEOPFrTelemetry extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Franke bot = new Franke();
    @Override
    public void init() { }
    @Override
    public void init_loop() { }
    @Override
    public void start() {
        runtime.reset();
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void loop() {
            double plePower=1;
            double test;
            int a = 50;

            double plePowerF = 0.6*plePower/500;
            for(double plePowerL = 0.6*plePower/a;plePowerL<plePower;a--) {
                test = plePowerL;
                telemetry.addData("Test",test);
                telemetry.update();
                //          pleDrive.setPower(plePowerL);
            }
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
    @Override
    public void stop() {
    }
}