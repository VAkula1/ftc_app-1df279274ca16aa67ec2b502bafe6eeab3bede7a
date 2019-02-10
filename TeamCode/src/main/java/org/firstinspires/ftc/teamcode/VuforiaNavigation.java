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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Vuforia Navigation", group ="Vuforia")
//@Disabled
public class VuforiaNavigation extends LinearOpMode {

    public static final String TAG = "Vuforia Navigation";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AbLXb/7/////AAABmZPq0MvFDk4BkdDZPDy4vpEw4iXdotDA9rdzZPut5Y" +
                "NUWndrIRaL7DQXhD2F4ut0N739aGN6kDO9QpFzhkhyJDJ4MktcKeoy7Qgc0tSYofkBHUowzOrzS8hW1hs" +
                "Aq2S0oyCb6LZEMnbEjI8mUnYV2joSRqm5f2RTSDC1TzP8Hc7npykxT8eMQucj+VkBGsFSuFy/SnksJpVp" +
                "7Zuq8bNPhpTVIEHQ9UHU9km8RjCpoUqFxUKUMZcTPf/jZ2uV6bQxGmoo0CncW6Fa7vV2HWl/lbgL8NlVJ" +
                "92ph9g7B7DF1V+mQhgAtVEfQe20QQEliPg0KAlJpLHDBg8muHokO3KMc1f6Qsuks+qZMqX9nzsnrZv0";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;


        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables RoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");

        VuforiaTrackable FrontPerimeter = RoverRuckus.get(0);
        FrontPerimeter.setName("FrontPerimeter");

        VuforiaTrackable BackPerimeter = RoverRuckus.get(1);
        BackPerimeter.setName("BackPerimeter");

        VuforiaTrackable BluePerimeter = RoverRuckus.get(2);
        BluePerimeter.setName("BluePerimeter");

        VuforiaTrackable RedPerimeter = RoverRuckus.get(3);
        RedPerimeter.setName("RedPerimeter");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(RoverRuckus);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;

        OpenGLMatrix FrontPerimeterLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        FrontPerimeter.setLocation(FrontPerimeterLocationOnField);
        RobotLog.ii(TAG, "FrontPerimeter=%s", format(FrontPerimeterLocationOnField));

        OpenGLMatrix BackPerimeterLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        BackPerimeter.setLocation(BackPerimeterLocationOnField);
        RobotLog.ii(TAG, "BackPerimeter=%s", format(BackPerimeterLocationOnField));

        OpenGLMatrix BluePerimeterLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        BluePerimeter.setLocation(BluePerimeterLocationOnField);
        RobotLog.ii(TAG, "BluePerimeter=%s", format(BluePerimeterLocationOnField));

        OpenGLMatrix RedPerimeterLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        RedPerimeter.setLocation(RedPerimeterLocationOnField);
        RobotLog.ii(TAG, "RedPerimeter=%s", format(RedPerimeterLocationOnField));



        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)FrontPerimeter.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        RoverRuckus.activate();

        while (opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", format(lastLocation));
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
