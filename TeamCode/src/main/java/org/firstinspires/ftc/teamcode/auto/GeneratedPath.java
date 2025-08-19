package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.PathChain;

public class GeneratedPath {
    private final PathChain pathChain;

    public GeneratedPath() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1 - Move to scoring position 1
                        new BezierLine(
                                new Point(9.000, 63.000, Point.CARTESIAN),
                                new Point(36.000, 63.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2 - Move to sample pickup 1
                        new BezierCurve(
                                new Point(36.000, 63.000, Point.CARTESIAN),
                                new Point(13.000, 52.000, Point.CARTESIAN),
                                new Point(16.000, 22.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3 - Move to sample pickup 2
                        new BezierLine(
                                new Point(16.000, 22.000, Point.CARTESIAN),
                                new Point(16.000, 13.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4 - Move to sample pickup 3
                        new BezierLine(
                                new Point(16.000, 13.000, Point.CARTESIAN),
                                new Point(16.000, 10.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-14))
                .addPath(
                        // Line 5 - Move to specimen pickup 1
                        new BezierCurve(
                                new Point(16.000, 10.000, Point.CARTESIAN),
                                new Point(8.000, 33.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 6 - Move to scoring 2
                        new BezierLine(
                                new Point(8.000, 33.000, Point.CARTESIAN),
                                new Point(36.000, 63.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 7 - Move to specimen pickup 2
                        new BezierLine(
                                new Point(36.000, 63.000, Point.CARTESIAN),
                                new Point(8.000, 33.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 8 - Move to scoring 3
                        new BezierLine(
                                new Point(8.000, 33.000, Point.CARTESIAN),
                                new Point(36.000, 63.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 9 - Move to specimen pickup 3
                        new BezierLine(
                                new Point(36.000, 63.000, Point.CARTESIAN),
                                new Point(8.000, 33.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 10 - Move to scoring 4
                        new BezierLine(
                                new Point(8.000, 33.000, Point.CARTESIAN),
                                new Point(36.000, 63.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 11 - Move to specimen pickup 4
                        new BezierLine(
                                new Point(36.000, 63.000, Point.CARTESIAN),
                                new Point(8.000, 33.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 12 - Move to scoring 5
                        new BezierLine(
                                new Point(8.000, 33.000, Point.CARTESIAN),
                                new Point(36.000, 63.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 13 - Move to parking
                        new BezierLine(
                                new Point(36.000, 63.000, Point.CARTESIAN),
                                new Point(8.000, 29.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0));

        this.pathChain = builder.build();
    }

    public PathChain getPathChain() {
        return pathChain;
    }
}