using System;
using System.Collections.Generic;
using UnityEngine;
using IK = LineWatch.ScaraInverseKinematics;

namespace LineWatch
{
    public static class ScaraMotionPlanner
    {
        public enum Failure { None, InvalidInput, PathUnreachable, ConfigurationChangeRequired, ResolutionLimit }
        public const float LineToleranceMetres = 0.0001f;
        private const int MaximumNodes = 4096;
        private const int MaximumDepth = 20;

        public readonly struct Node
        {
            public float Fraction { get; }
            public IK.JointValues Joints { get; }
            public float SecondsFromPrevious { get; }
            internal Node(float fraction, IK.JointValues joints, float seconds)
            { Fraction = fraction; Joints = joints; SecondsFromPrevious = seconds; }
        }

        public sealed class Plan
        {
            public IReadOnlyList<Node> Nodes { get; }
            public IK.Solution End { get; }
            public float DurationSeconds { get; }
            internal Plan(List<Node> nodes, IK.Solution end)
            {
                Nodes = nodes.AsReadOnly();
                End = end;
                float seconds = 0f;
                foreach (var node in nodes) seconds += node.SecondsFromPrevious;
                DurationSeconds = seconds;
            }
        }

        public static bool Joint(IK.JointValues start, IK.Solution end, IK.JointLimits limits, Vector4 speeds,
            out Plan plan, out Failure failure)
        {
            plan = null;
            failure = Failure.InvalidInput;
            if (!ValidLimits(limits) || !Within(start, limits) || !Within(end.Joints, limits) || !ValidSpeeds(speeds)) return false;
            plan = new Plan(new List<Node> { new Node(0f, start, 0f), new Node(1f, end.Joints, Seconds(start, end.Joints, speeds)) }, end);
            failure = Failure.None;
            return true;
        }

        public static bool Linear(IK.JointValues start, Vector3 target, float targetYaw,
            ScaraForwardKinematics.Geometry geometry, IK.JointLimits limits, Vector4 speeds, int requestedBranch,
            out Plan plan, out Failure failure)
        {
            plan = null;
            failure = Failure.InvalidInput;
            if (!ValidLimits(limits) || !Within(start, limits) || !ValidSpeeds(speeds) || requestedBranch < -1 || requestedBranch > 1) return false;
            var endpoint = IK.Solve(target, targetYaw, geometry, limits);
            if (!endpoint.Success)
            {
                failure = endpoint.Failure == IK.FailureReason.Unreachable ? Failure.PathUnreachable
                    : endpoint.Failure == IK.FailureReason.JointLimit ? Failure.ConfigurationChangeRequired : Failure.InvalidInput;
                return false;
            }
            int initialBranch = Mathf.Abs(start.Q2Degrees) <= 0.0001f ? 0 : Math.Sign(start.Q2Degrees);
            if (requestedBranch != 0 && initialBranch != 0 && initialBranch != requestedBranch)
            { failure = Failure.ConfigurationChangeRequired; return false; }
            ScaraForwardKinematics.TcpPose origin;
            try { origin = ScaraForwardKinematics.Calculate(start.Q1Degrees, start.Q2Degrees, start.D3Metres, start.Q4Degrees, geometry); }
            catch (ArgumentException) { return false; }
            double startYaw = (double)start.Q1Degrees + start.Q2Degrees + start.Q4Degrees + geometry.HomeToolYawDegrees;
            float yawDelta = Mathf.DeltaAngle(origin.YawDegrees, targetYaw);
            int firstBranch = initialBranch != 0 ? initialBranch : requestedBranch != 0 ? requestedBranch : 1;
            int attempts = initialBranch == 0 && requestedBranch == 0 ? 2 : 1;
            double bestCost = double.PositiveInfinity;
            for (int attempt = 0; attempt < attempts; attempt++)
            {
                int branch = attempt == 0 ? firstBranch : -firstBranch;
                var nodes = new List<Node> { new Node(0f, start, 0f) };
                Failure pathFailure = Failure.None;
                IK.Solution end = default;

                bool SolveAt(float fraction, IK.JointValues seed, out IK.Solution chosen)
                {
                    chosen = default;
                    Vector3 position = Vector3.Lerp(origin.PositionMetres, target, fraction);
                    double yaw = startYaw + yawDelta * fraction;
                    var result = IK.Solve(position, (float)yaw, geometry, limits);
                    if (!result.Success)
                    {
                        pathFailure = result.Failure == IK.FailureReason.Unreachable ? Failure.PathUnreachable : Failure.ConfigurationChangeRequired;
                        return false;
                    }
                    double best = double.PositiveInfinity;
                    bool found = false;
                    foreach (var candidate in result.Solutions)
                    {
                        if (candidate.ElbowSign != 0 && candidate.ElbowSign != branch) continue;
                        var raw = candidate.Joints;
                        // Lift angle representations continuously; limits apply to physical turns.
                        float q1 = seed.Q1Degrees + Mathf.DeltaAngle(seed.Q1Degrees, raw.Q1Degrees);
                        float q2 = seed.Q2Degrees + Mathf.DeltaAngle(seed.Q2Degrees, raw.Q2Degrees);
                        float q4 = (float)(yaw - geometry.HomeToolYawDegrees - q1 - q2);
                        var lifted = new IK.JointValues(Snap(q1, limits.Minimum.Q1Degrees, limits.Maximum.Q1Degrees),
                            Snap(q2, limits.Minimum.Q2Degrees, limits.Maximum.Q2Degrees), raw.D3Metres,
                            Snap(q4, limits.Minimum.Q4Degrees, limits.Maximum.Q4Degrees));
                        if (!Within(lifted, limits)) continue;
                        var check = ScaraForwardKinematics.Calculate(lifted.Q1Degrees, lifted.Q2Degrees, lifted.D3Metres, lifted.Q4Degrees, geometry);
                        if (Vector3.Distance(check.PositionMetres, position) > 0.000003f || Mathf.Abs(Mathf.DeltaAngle(check.YawDegrees, (float)yaw)) > 0.001f) continue;
                        double cost = Cost(seed, lifted);
                        if (cost >= best) continue;
                        best = cost;
                        chosen = new IK.Solution(lifted, candidate.IsSingular, candidate.ElbowSign);
                        found = true;
                    }
                    if (!found) pathFailure = Failure.ConfigurationChangeRequired;
                    return found;
                }

                bool Extend(float from, IK.JointValues previous, float to, int depth)
                {
                    if (nodes.Count >= MaximumNodes || depth > MaximumDepth)
                    { pathFailure = Failure.ResolutionLimit; return false; }
                    if (!SolveAt(to, previous, out var next)) return false;
                    // Linear interpolation error <= max|p''|/8. Each rotating link contributes
                    // length * (joint-angle change in radians)^2; the vertical term is linear.
                    double d1 = (next.Joints.Q1Degrees - previous.Q1Degrees) * Math.PI / 180.0;
                    double d2 = (next.Joints.Q2Degrees - previous.Q2Degrees) * Math.PI / 180.0;
                    double d4 = (next.Joints.Q4Degrees - previous.Q4Degrees) * Math.PI / 180.0;
                    double radius = Math.Sqrt((double)geometry.ToolOffsetMetres.x * geometry.ToolOffsetMetres.x +
                        (double)geometry.ToolOffsetMetres.z * geometry.ToolOffsetMetres.z);
                    double bound = (geometry.Link1Metres * d1 * d1 + geometry.Link2Metres * (d1 + d2) * (d1 + d2) +
                        radius * (d1 + d2 + d4) * (d1 + d2 + d4)) / 8.0 + 0.000003;
                    if (bound > LineToleranceMetres)
                    {
                        float mid = (from + to) * 0.5f;
                        if (!Extend(from, previous, mid, depth + 1)) return false;
                        return Extend(mid, nodes[nodes.Count - 1].Joints, to, depth + 1);
                    }
                    nodes.Add(new Node(to, next.Joints, Seconds(previous, next.Joints, speeds)));
                    end = next;
                    return true;
                }

                if (Extend(0f, start, 1f, 0))
                {
                    double cost = Cost(start, end.Joints);
                    if (cost < bestCost) { bestCost = cost; plan = new Plan(nodes, end); }
                }
                else if (plan == null) failure = pathFailure;
            }
            if (plan == null) return false;
            failure = Failure.None;
            return true;
        }

        public static IK.JointValues Lerp(IK.JointValues a, IK.JointValues b, float t) => new IK.JointValues(
            Mathf.Lerp(a.Q1Degrees, b.Q1Degrees, t), Mathf.Lerp(a.Q2Degrees, b.Q2Degrees, t),
            Mathf.Lerp(a.D3Metres, b.D3Metres, t), Mathf.Lerp(a.Q4Degrees, b.Q4Degrees, t));

        private static float Seconds(IK.JointValues a, IK.JointValues b, Vector4 speeds) => 1.00001f * Mathf.Max(
            Mathf.Abs(b.Q1Degrees - a.Q1Degrees) / speeds.x, Mathf.Abs(b.Q2Degrees - a.Q2Degrees) / speeds.y,
            Mathf.Abs(b.D3Metres - a.D3Metres) / speeds.z, Mathf.Abs(b.Q4Degrees - a.Q4Degrees) / speeds.w);
        private static bool ValidSpeeds(Vector4 speeds) => Finite(speeds.x) && speeds.x > 0f && Finite(speeds.y) && speeds.y > 0f &&
            Finite(speeds.z) && speeds.z > 0f && Finite(speeds.w) && speeds.w > 0f;
        private static bool ValidLimits(IK.JointLimits limits) => Range(limits.Minimum.Q1Degrees, limits.Maximum.Q1Degrees, true) &&
            Range(limits.Minimum.Q2Degrees, limits.Maximum.Q2Degrees, true) && Range(limits.Minimum.D3Metres, limits.Maximum.D3Metres, false) &&
            Range(limits.Minimum.Q4Degrees, limits.Maximum.Q4Degrees, true);
        private static bool Range(float min, float max, bool rotary) => Finite(min) && Finite(max) && min <= max && (!rotary || (double)max - min <= 360.0);
        private static double Cost(IK.JointValues a, IK.JointValues b)
        {
            double d1 = (double)b.Q1Degrees - a.Q1Degrees, d2 = (double)b.Q2Degrees - a.Q2Degrees, d4 = (double)b.Q4Degrees - a.Q4Degrees;
            return d1 * d1 + d2 * d2 + d4 * d4;
        }
        private static float Snap(float value, float min, float max) => value < min && min - value <= 0.0001f ? min
            : value > max && value - max <= 0.0001f ? max : value;
        private static bool Within(IK.JointValues j, IK.JointLimits limits) =>
            Finite(j.Q1Degrees) && j.Q1Degrees >= limits.Minimum.Q1Degrees && j.Q1Degrees <= limits.Maximum.Q1Degrees &&
            Finite(j.Q2Degrees) && j.Q2Degrees >= limits.Minimum.Q2Degrees && j.Q2Degrees <= limits.Maximum.Q2Degrees &&
            Finite(j.D3Metres) && j.D3Metres >= limits.Minimum.D3Metres && j.D3Metres <= limits.Maximum.D3Metres &&
            Finite(j.Q4Degrees) && j.Q4Degrees >= limits.Minimum.Q4Degrees && j.Q4Degrees <= limits.Maximum.Q4Degrees;
        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
