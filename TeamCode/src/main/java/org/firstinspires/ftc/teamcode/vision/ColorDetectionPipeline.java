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

    private ArrayList<Integer> colorDetections;
    final int hueThreshold = 15;

    //RGB
//    final int[] lime = {182, 255, 0};
//    final int[] magenta = {255, 0, 198};
//    final int[] cyan = {0, 207, 255};

    //OpenCV HSV
    final int[] lime = {38, 255, 255};
    final int[] magenta = {156, 255, 255};
    final int[] cyan = {95, 255, 255};
    final int[] hues = {38, 156, 195};
    private Mat hueTargets;

    public ColorDetectionPipeline() {
        colorDetections = new ArrayList<Integer>();

        hueTargets = new Mat(3, 1, CvType.CV_32SC1);
        hueTargets.put(0, 0, 38.0);  //[ 38  ]
        hueTargets.put(1, 0, 156.0); //[ 156 ]
        hueTargets.put(2, 0, 195.0); //[ 195 ]

    }

    @Override
    public Mat processFrame(Mat input) {
        //Crop image to center 1/9th
        Rect rectCrop = new Rect((int) (input.width() / 2), (int) (input.height() / 2), (int) (input.width() / 6), (int) (input.height() / 6));
        input = new Mat(input, rectCrop);

        //Sample center 5x5 pixels
        Rect sampleCrop = new Rect((int) (input.width() / 2), (int) (input.height() / 2), 5, 5);
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

        //Compute distance from each target color hue
        Mat hueDiff = new Mat();
        Mat hueDupe = new Mat(3, 1, hueChannel.type());
        hueDupe.put(0, 0, hueChannel.get(0, 0)[0]);
        hueDupe.put(1, 0, hueChannel.get(0, 0)[0]);
        hueDupe.put(2, 0, hueChannel.get(0,0)[0]);
        hueTargets.convertTo(hueTargets, hueChannel.type());

        Core.absdiff(hueTargets, hueChannel, hueDiff);

        //Find location of min hue difference
        int result = (int)Core.minMaxLoc(hueDiff).minLoc.y;

        //Add color detection hue diff is small enough and not already detected
        if(hueDiff.get(result, 0)[0] < hueThreshold ) {
            boolean included = false;
            for (int detection : colorDetections)
                if (result == detection) included = true;
            if (!included) colorDetections.add(new Integer(result));
        }

        //Draw rectangle around center 30x30 pixels to help line up camera
        Point upperLeft = new Point((int)input.cols()/2 - 15, (int)input.cols()/2 + 15);
        Point bottomRight = new Point((int)input.cols()/2 + 15, (int)input.cols()/2 - 15);
        rectangle(input, upperLeft, bottomRight, new Scalar(255, 25, 25), 5);

        return input;
    }

    public ArrayList<Integer> getColorDetections() {
        return colorDetections;
    }

}