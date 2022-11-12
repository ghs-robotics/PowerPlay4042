/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.Core.mean;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.filter2D;
import static org.opencv.imgproc.Imgproc.rectangle;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ColorDetectionPipeline extends OpenCvPipeline {
    private Telemetry telemetry;

    private ArrayList<Integer> colorDetections;
    final int hueThreshold = 20;

    //RGB
//    final int[] lime = {182, 255, 0};
//    final int[] magenta = {255, 0, 198};
//    final int[] cyan = {0, 207, 255};

    //OpenCV HSV
    final int[] lime = {38, 255, 255};
    final int[] magenta = {156, 255, 255};
    final int[] cyan = {95, 255, 255};
    final int[] hues = {38, 156, 95};
    private ArrayList<Double> hueTargets;

    public ColorDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;

        colorDetections = new ArrayList<Integer>();
    }

    @Override
    public Mat processFrame(Mat input) {
        //Crop image to center 1/9th
        Rect rectCrop = new Rect((int) (input.width() / 2), (int) (input.height() / 2), (int) (input.width() / 5), (int) (input.height() / 5));
        input = new Mat(input, rectCrop);

        //Sample center 5x5 pixels
        Rect sampleCrop = new Rect((int) (input.width() / 2), (int) (input.height() / 2), 15, 15);
        Mat colorSample = new Mat(input, sampleCrop);

        //convert image from RGB to HSV
        Mat hsvConvert = new Mat();
        cvtColor(colorSample, hsvConvert, Imgproc.COLOR_RGB2HSV);

        //Find average of each channel across sample pixels
        Mat avgColColor = new Mat();
        Mat avgColor = new Mat();
        Core.reduce(hsvConvert, avgColColor, 0, Core.REDUCE_AVG);
        Core.reduce(avgColColor, avgColor, 1, Core.REDUCE_AVG);
        ArrayList<Mat> channels = new ArrayList<Mat>(3);
        Core.split(avgColor, channels);
        Mat hueChannel = channels.get(0);

        hueTargets = new ArrayList<Double>();
        hueTargets.add(38.0);  //[ 38  ]
        hueTargets.add(156.0); //[ 156 ]
        hueTargets.add(95.0); //[ 195 ]

        //Compute distance from each target color hue
        ArrayList<Double> hueDiff = new ArrayList<Double>();
//
        for(int i=0; i<hueTargets.size(); i++) {
                hueDiff.add(Math.abs(hueTargets.get(i) - hueChannel.get(0, 0)[0]));
            }

//        Core.absdiff(hueTargets, hueChannel, hueDiff);
//
//        //Find location of min hue difference
        int result = -1;
        int minIndex = 2;
        double min = hueDiff.get(2);
        for(int i=1; i>=0; i--) {
            if(hueDiff.get(i) < min) {
                minIndex = i;
                min = hueDiff.get(i);
            }
        }

//        telemetry.addLine("Sighted Hue: " + hueChannel.get(0, 0)[0]);
//        telemetry.addLine("Hue Difference: " + hueDiff.get(minIndex));
//        telemetry.addLine("Result is: " + result);
//        telemetry.addLine("MinIndex is: " + minIndex);
//        telemetry.addLine("" + hueDiff.get(0));
//        telemetry.addLine("" + hueDiff.get(1));
//        telemetry.addLine("" + hueDiff.get(2));
//
//        telemetry.update();

        //Add color detection hue diff is small enough and not already detected
        if(hueDiff.get(minIndex) < hueThreshold ) {
            result = minIndex;
            boolean included = false;
            for (int detection : colorDetections)
                if (result == detection) included = true;
            if(!included) colorDetections.add(new Integer(result));
            //colorDetections.remove(0);
        }

        //Draw rectangle around center 30x30 pixels to help line up cameraq

        Point upperLeft = new Point((int)input.cols()/2 - 10, (int)input.cols()/2 + 10);
        Point bottomRight = new Point((int)input.cols()/2 + 10, (int)input.cols()/2 - 10);
        rectangle(input, upperLeft, bottomRight, new Scalar(255, 25, 25), 2);

        return input;
    }

    public ArrayList<Integer> getColorDetections() {
        return colorDetections;
    }

}