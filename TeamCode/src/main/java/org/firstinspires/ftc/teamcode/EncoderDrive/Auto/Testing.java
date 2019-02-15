
package org.firstinspires.ftc.teamcode.EncoderDrive.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Testing", group="Linear Opmode")
public class Testing extends LinearOpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_front = null;
    double left_frontPower;
    double encoderValue;
    private void TelemetryUpdate(){
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motor", "left (%.2f)", left_frontPower);
        telemetry.update();
    }

    @Override
    public void runOpMode() {

        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_front.setDirection(DcMotor.Direction.FORWARD);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();
        //5/10/15

        while (opModeIsActive()) {
            TelemetryUpdate();

            left_frontPower=0.05;
            left_front.setPower(left_frontPower);
            TelemetryUpdate();
            sleep(5000);

            left_frontPower=0.10;
            left_front.setPower(left_frontPower);
            TelemetryUpdate();
            sleep(5000);

            left_frontPower=0.15;
            left_front.setPower(left_frontPower);
            TelemetryUpdate();
            sleep(5000);
        }
    }
}
