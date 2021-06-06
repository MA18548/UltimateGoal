//package org.ma.ftc.lib.control; // MA FTC 18548
///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.ma.ftc.lib.command.HardwareMapEx;
//
//public class SpeedControl {
//    /* Declare OpMode members. */
//    private HardwareMapEx robot   = HardwareMapEx.getInstance();   // Use a Pushbot's hardware
//    private ElapsedTime     runtime = new ElapsedTime();
//
//    private double counts_per_revolution;
//    private double internal_gearbox_ratio;
//    private double external_gearbox_ratio;
//
//    private double lastLeftTime = runtime.seconds();
//    private double lastRightTime = runtime.seconds();
//
//    private double lastLeftTicks = 0;
//    private double lastRightTicks = 0;
//
//    public SpeedControl()
//    {
//        this.counts_per_revolution = 1;
//        this.internal_gearbox_ratio = 1;
//        this.external_gearbox_ratio = 1;
//    }
//
//    public SpeedControl(double counts_per_revolution)
//    {
//        this.counts_per_revolution = counts_per_revolution;
//
//        this.internal_gearbox_ratio = 1;
//        this.external_gearbox_ratio = 1;
//    }
//
//    public SpeedControl(double counts_per_revolution, double internal_gearbox_ratio)
//    {
//        this.counts_per_revolution = counts_per_revolution;
//        this.internal_gearbox_ratio = internal_gearbox_ratio;
//
//        this.external_gearbox_ratio = 1;
//    }
//
//    public SpeedControl(double counts_per_revolution, double internal_gearbox_ratio,
//                        double external_gearbox_ratio)
//    {
//        this.counts_per_revolution = counts_per_revolution;
//        this.internal_gearbox_ratio = internal_gearbox_ratio;
//        this.external_gearbox_ratio = external_gearbox_ratio;
//    }
//
//
//    public double getSpeed()
//    {
//        return ( getLeftSpeed() + getRightSpeed() ) / 2;
//    }
//
//    public double getLeftSpeed()
//    {
//
//        double currentLeftTicks = robot.leftDrive.getCurrentPosition();
//        double currentTime = runtime.seconds();
//
//        double speed =
//                ( currentLeftTicks - lastLeftTicks ) / ( currentTime - lastLeftTime );
//
//
//        lastLeftTicks = currentLeftTicks;
//        lastLeftTime = currentTime;
//
//        return speed;
//    }
//
//    public double getRightSpeed()
//    {
//
//        double currentRightTicks = robot.rightDrive.getCurrentPosition();
//        double currentTime = runtime.seconds();
//
//        double speed =
//                ( currentRightTicks - lastRightTicks ) / ( currentTime - lastRightTime );
//
//
//        lastRightTicks = currentRightTicks;
//        lastRightTime = currentTime;
//
//        return speed;
//    }
//
//    public double getRPM()
//    {
//        return (getLeftRPM() + getLeftRPM()) / 2;
//    }
//
//    public double getLeftRPM()
//    {
//        return (getLeftSpeed() / this.counts_per_revolution) * 60;
//    }
//
//    public double getRightRPM()
//    {
//        return (getRightSpeed() / this.counts_per_revolution) * 60;
//    }
//
//
//    public double getMotorRPM() { return (getLeftMotorRPM() + getRightMotorRPM()) / 2; }
//
//    public double getLeftMotorRPM() { return getLeftRPM() * internal_gearbox_ratio; }
//
//    public double getRightMotorRPM() { return getLeftRPM() * internal_gearbox_ratio; }
//
//
//    public double getExternalRPM() { return (getLeftExternalRPM() + getRightExternalRPM()) / 2; }
//
//    public double getLeftExternalRPM() { return getLeftMotorRPM() * external_gearbox_ratio; }
//
//    public double getRightExternalRPM() { return getRightMotorRPM() * external_gearbox_ratio;}
//
//    public void resetTimer()
//    {
//        robot.period.reset();
//    }
//}