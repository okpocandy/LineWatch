using System;
using System.Collections.Generic;
using UnityEngine;

namespace LineWatch
{
    public static class ScaraInverseKinematics
    {
        public enum FailureReason { None, InvalidInput, Unreachable, JointLimit, Indeterminate, NumericalError }

        public readonly struct JointValues
        {
            public float Q1Degrees { get; }
            public float Q2Degrees { get; }
            public float D3Metres { get; }
            public float Q4Degrees { get; }

            public JointValues(float q1Degrees, float q2Degrees, float d3Metres, float q4Degrees)
            {
                Q1Degrees = q1Degrees;
                Q2Degrees = q2Degrees;
                D3Metres = d3Metres;
                Q4Degrees = q4Degrees;
            }
        }

        // Each rotary range covers at most one turn; the current axes are bounded, not continuous.
        public readonly struct JointLimits
        {
            public JointValues Minimum { get; }
            public JointValues Maximum { get; }
            public JointLimits(JointValues minimum, JointValues maximum)
            {
                Minimum = minimum;
                Maximum = maximum;
            }
        }

        public readonly struct Solution
        {
            public JointValues Joints { get; }
            public bool IsSingular { get; }
            // Sign of the analytic elbow branch, independent of equivalent 360-degree values.
            public int ElbowSign { get; }
            internal Solution(JointValues joints, bool singular, int elbowSign)
            {
                Joints = joints;
                IsSingular = singular;
                ElbowSign = elbowSign;
            }
        }

        public sealed class Result
        {
            public FailureReason Failure { get; }
            public bool Success => Failure == FailureReason.None;
            public IReadOnlyList<Solution> Solutions { get; }
            internal Result(FailureReason failure, List<Solution> solutions = null)
            {
                Failure = failure;
                Solutions = solutions == null ? (IReadOnlyList<Solution>)Array.Empty<Solution>() : solutions.AsReadOnly();
            }
        }

        private const double Degrees = 180.0 / Math.PI;
        private const double CosineTolerance = 1e-6;
        private const double AngleRoundoffDegrees = 1e-4;
        private const double SlideRoundoffMetres = 1e-7;

        // Pure calculation in RobotFrame coordinates. No scene, Transform or motor is accessed.
        public static Result Solve(Vector3 targetMetres, float targetYawDegrees,
            ScaraForwardKinematics.Geometry geometry, JointLimits limits)
        {
            if (!Finite(targetMetres) || !Finite(targetYawDegrees) || !Valid(geometry) || !Valid(limits))
                return new Result(FailureReason.InvalidInput);

            double toolDegrees = Normalize((double)targetYawDegrees - geometry.HomeToolYawDegrees);
            double toolRadians = toolDegrees / Degrees;
            double sin = Math.Sin(toolRadians), cos = Math.Cos(toolRadians);
            var offset = geometry.ToolOffsetMetres;
            double u = targetMetres.x - (offset.x * cos + offset.z * sin);
            double v = -(targetMetres.z - (-offset.x * sin + offset.z * cos));
            double l1 = geometry.Link1Metres, l2 = geometry.Link2Metres;
            double d = (u * u + v * v - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
            if (d < -1.0 - CosineTolerance || d > 1.0 + CosineTolerance)
                return new Result(FailureReason.Unreachable);
            d = Math.Max(-1.0, Math.Min(1.0, d));
            // Equal links folded exactly onto the shoulder have infinitely many shoulder angles.
            if (u == 0.0 && v == 0.0 && l1 == l2)
                return new Result(FailureReason.Indeterminate);

            double slide = (double)geometry.Height12Metres + geometry.Height23Metres + offset.y - targetMetres.y;
            if (!TryBound(slide, limits.Minimum.D3Metres, limits.Maximum.D3Metres, SlideRoundoffMetres, out float d3))
                return new Result(FailureReason.JointLimit);
            double elbowSin = Math.Sqrt(Math.Max(0.0, 1.0 - d * d));
            bool singular = 1.0 - Math.Abs(d) <= 1e-12;
            if (singular) { d = d >= 0.0 ? 1.0 : -1.0; elbowSin = 0.0; }
            var solutions = new List<Solution>();
            bool failedResidual = false;
            for (int branch = 0; branch < (singular ? 1 : 2); branch++)
            {
                double s2 = branch == 0 ? elbowSin : -elbowSin;
                double q2 = Math.Atan2(s2, d) * Degrees;
                double q1 = (Math.Atan2(v, u) - Math.Atan2(l2 * s2, l1 + l2 * d)) * Degrees;
                foreach (float j1 in Equivalents(q1, limits.Minimum.Q1Degrees, limits.Maximum.Q1Degrees))
                foreach (float j2 in Equivalents(q2, limits.Minimum.Q2Degrees, limits.Maximum.Q2Degrees))
                foreach (float j4 in Equivalents(toolDegrees - j1 - j2, limits.Minimum.Q4Degrees, limits.Maximum.Q4Degrees))
                {
                    var joints = new JointValues(j1, j2, d3, j4);
                    bool matches;
                    try
                    {
                        var pose = ScaraForwardKinematics.Calculate(j1, j2, d3, j4, geometry);
                        matches = Vector3.Distance(pose.PositionMetres, targetMetres) <= 0.000002f &&
                            Mathf.Abs(Mathf.DeltaAngle(pose.YawDegrees, targetYawDegrees)) <= 0.001f;
                    }
                    catch (ArgumentException) { matches = false; }
                    if (!matches) { failedResidual = true; continue; }
                    if (!solutions.Exists(existing => Same(existing.Joints, joints)))
                        solutions.Add(new Solution(joints, singular, singular ? 0 : branch == 0 ? 1 : -1));
                }
            }
            return solutions.Count > 0 ? new Result(FailureReason.None, solutions)
                : new Result(failedResidual ? FailureReason.NumericalError : FailureReason.JointLimit);
        }

        private static IEnumerable<float> Equivalents(double angle, float minimum, float maximum)
        {
            angle = Normalize(angle);
            double first = Math.Ceiling((minimum - angle - AngleRoundoffDegrees) / 360.0);
            double last = Math.Floor((maximum - angle + AngleRoundoffDegrees) / 360.0);
            // A range no wider than 360 degrees has at most two equivalent endpoint values.
            for (int i = 0; i < 2 && first + i <= last; i++)
                if (TryBound(angle + 360.0 * (first + i), minimum, maximum, AngleRoundoffDegrees, out float value))
                    yield return value;
        }

        private static bool TryBound(double value, float minimum, float maximum, double tolerance, out float bounded)
        {
            bounded = 0f;
            if (value < minimum - tolerance || value > maximum + tolerance) return false;
            bounded = (float)Math.Max(minimum, Math.Min(maximum, value));
            return true;
        }

        private static bool Same(JointValues a, JointValues b) =>
            Math.Abs((double)a.Q1Degrees - b.Q1Degrees) <= AngleRoundoffDegrees &&
            Math.Abs((double)a.Q2Degrees - b.Q2Degrees) <= AngleRoundoffDegrees &&
            Math.Abs((double)a.Q4Degrees - b.Q4Degrees) <= AngleRoundoffDegrees &&
            Math.Abs((double)a.D3Metres - b.D3Metres) <= SlideRoundoffMetres;

        private static double Normalize(double degrees)
        {
            double value = degrees % 360.0;
            return value > 180.0 ? value - 360.0 : value < -180.0 ? value + 360.0 : value;
        }

        private static bool Valid(ScaraForwardKinematics.Geometry g) =>
            Finite(g.Link1Metres) && g.Link1Metres > 0f && Finite(g.Link2Metres) && g.Link2Metres > 0f &&
            Finite(g.Height12Metres) && Finite(g.Height23Metres) && Finite(g.ToolOffsetMetres) && Finite(g.HomeToolYawDegrees);

        private static bool Valid(JointLimits limits)
        {
            var min = limits.Minimum;
            var max = limits.Maximum;
            return Range(min.Q1Degrees, max.Q1Degrees, true) && Range(min.Q2Degrees, max.Q2Degrees, true) &&
                Range(min.Q4Degrees, max.Q4Degrees, true) && Range(min.D3Metres, max.D3Metres, false);
        }

        private static bool Range(float minimum, float maximum, bool rotary) =>
            Finite(minimum) && Finite(maximum) && minimum <= maximum && (!rotary || (double)maximum - minimum <= 360.0);
        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static bool Finite(Vector3 value) => Finite(value.x) && Finite(value.y) && Finite(value.z);
    }
}
