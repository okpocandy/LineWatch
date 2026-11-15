using System;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

namespace LineWatch
{
    public static class FanucInverseKinematics
    {
        public const float PositionToleranceMetres = .0001f;
        public const float RotationToleranceDegrees = .05f;
        private const double RadiansPerDegree = Math.PI / 180;
        private const double LengthScaleMetres = .5;

        public enum Failure
        {
            None, InvalidInput, InvalidGeometry, InvalidSeed, OutsideReachBound,
            IterationLimit, TimeLimit, Stalled, JointLimitStall, NumericalError
        }

        public readonly struct Budget
        {
            public int MaximumIterations { get; }
            public double MaximumMilliseconds { get; }
            public Budget(int maximumIterations, double maximumMilliseconds)
            {
                MaximumIterations = maximumIterations;
                MaximumMilliseconds = maximumMilliseconds;
            }
        }

        public sealed class Result
        {
            public bool Succeeded => Reason == Failure.None;
            public Failure Reason { get; }
            public IReadOnlyList<float> JointDegrees { get; }
            public float? PositionErrorMetres { get; }
            public float? RotationErrorDegrees { get; }
            public int Iterations { get; }
            public int SeedsTried { get; }
            public int SeedIndex { get; }
            public double ElapsedMilliseconds { get; }

            internal Result(Failure reason, float[] solution, Sample sample, int iterations,
                int seedsTried, int seedIndex, double elapsed)
            {
                Reason = reason;
                JointDegrees = Array.AsReadOnly(solution == null ? Array.Empty<float>() : (float[])solution.Clone());
                PositionErrorMetres = sample?.PositionError;
                RotationErrorDegrees = sample?.RotationError;
                Iterations = iterations;
                SeedsTried = seedsTried;
                SeedIndex = seedIndex;
                ElapsedMilliseconds = elapsed;
            }
        }

        internal sealed class Sample
        {
            internal float[] Degrees;
            internal Vector3 Position;
            internal double[] Error;
            internal double Cost;
            internal float PositionError;
            internal float RotationError;
            internal bool Converged => PositionError <= PositionToleranceMetres && RotationError <= RotationToleranceDegrees;
        }

        // Pure pose search. Seeds and solutions are degrees; internal increments are radians.
        // A failed search never exposes a joint solution or changes a Transform/motor.
        public static Result Solve(Vector3 targetPositionMetres, Quaternion targetRotation,
            FanucKinematicGeometry geometry, IReadOnlyList<float> minimumDegrees,
            IReadOnlyList<float> maximumDegrees, IReadOnlyList<float[]> initialSeeds, Budget? requestedBudget = null)
        {
            var watch = Stopwatch.StartNew();
            var budget = requestedBudget ?? new Budget(240, 50);
            int iterations = 0, seedsTried = 0, bestSeed = -1;
            Sample best = null;
            Result Finish(Failure reason, Sample success = null, int seed = -1) => new Result(reason,
                success?.Degrees, success ?? best, iterations, seedsTried, success == null ? bestSeed : seed, watch.Elapsed.TotalMilliseconds);
            bool Expired() => watch.Elapsed.TotalMilliseconds >= budget.MaximumMilliseconds;
            void Remember(Sample sample, int seed)
            {
                if (best != null && sample.Cost >= best.Cost) return;
                best = sample;
                bestSeed = seed;
            }

            if (!Finite(targetPositionMetres) || !Unit(targetRotation) || budget.MaximumIterations < 0 ||
                !Finite(budget.MaximumMilliseconds) || budget.MaximumMilliseconds <= 0 ||
                minimumDegrees == null || maximumDegrees == null || minimumDegrees.Count != 6 || maximumDegrees.Count != 6 ||
                initialSeeds == null || initialSeeds.Count == 0) return Finish(Failure.InvalidInput);
            var lower = new double[6]; var upper = new double[6];
            for (int i = 0; i < 6; i++)
            {
                if (!Finite(minimumDegrees[i]) || !Finite(maximumDegrees[i]) || minimumDegrees[i] > maximumDegrees[i])
                    return Finish(Failure.InvalidInput);
                lower[i] = minimumDegrees[i] * RadiansPerDegree;
                upper[i] = maximumDegrees[i] * RadiansPerDegree;
            }
            var seeds = new double[initialSeeds.Count][];
            for (int s = 0; s < seeds.Length; s++)
            {
                var input = initialSeeds[s];
                if (input == null || input.Length != 6) return Finish(Failure.InvalidSeed);
                seeds[s] = new double[6];
                for (int i = 0; i < 6; i++)
                {
                    if (!Finite(input[i]) || input[i] < minimumDegrees[i] || input[i] > maximumDegrees[i])
                        return Finish(Failure.InvalidSeed);
                    seeds[s][i] = input[i] * RadiansPerDegree;
                }
                if (Expired()) return Finish(Failure.TimeLimit);
            }
            try { FanucForwardKinematics.Calculate(new float[6], geometry); }
            catch (ArgumentException) { return Finish(Failure.InvalidGeometry); }
            double radius = Norm(geometry.ToolPositionMetres);
            foreach (var home in geometry.Joints) radius += Norm(home.PositionMetres);
            if (Norm(targetPositionMetres) > radius + .00001 + radius * .000001) return Finish(Failure.OutsideReachBound);
            targetRotation = Quaternion.Normalize(targetRotation);

            Failure lastReason = Failure.Stalled;
            try
            {
                for (int seedIndex = 0; seedIndex < seeds.Length; seedIndex++)
                {
                    if (Expired()) return Finish(Failure.TimeLimit);
                    seedsTried++;
                    var q = seeds[seedIndex];
                    var current = Evaluate(q, targetPositionMetres, targetRotation, geometry, minimumDegrees, maximumDegrees);
                    Remember(current, seedIndex);
                    if (Expired()) return Finish(Failure.TimeLimit);
                    if (current.Converged) return Finish(Failure.None, current, seedIndex);
                    int remaining = budget.MaximumIterations - iterations;
                    int share = remaining <= 0 ? 0 : (int)Math.Ceiling((double)remaining / (seeds.Length - seedIndex));
                    double damping = .01;
                    lastReason = Failure.IterationLimit;
                    for (int local = 0; local < share; local++)
                    {
                        if (Expired()) return Finish(Failure.TimeLimit);
                        if (iterations >= budget.MaximumIterations) return Finish(Failure.IterationLimit);
                        iterations++;
                        var jacobian = Jacobian(current, geometry);
                        bool accepted = false, touchedLimit = false;
                        for (int attempt = 0; attempt < 7 && !accepted; attempt++)
                        {
                            if (Expired()) return Finish(Failure.TimeLimit);
                            if (!Step(jacobian, current.Error, damping, q, lower, upper, out var step, out bool activeLimit))
                                return Finish(Failure.NumericalError);
                            touchedLimit |= activeLimit;
                            double norm = 0;
                            for (int i = 0; i < 6; i++) norm += step[i] * step[i];
                            if (!Finite(norm)) return Finish(Failure.NumericalError);
                            if (norm < 1e-24) break;
                            // Internal trust radius only; it does not reject large final joint changes.
                            double trust = Math.Min(1, .35 / Math.Sqrt(norm));
                            for (int backtrack = 0; backtrack < 12; backtrack++)
                            {
                                if (Expired()) return Finish(Failure.TimeLimit);
                                double scale = trust * Math.Pow(.5, backtrack);
                                var candidate = new double[6];
                                for (int i = 0; i < 6; i++)
                                {
                                    double raw = q[i] + scale * step[i];
                                    candidate[i] = Math.Max(lower[i], Math.Min(upper[i], raw));
                                    touchedLimit |= candidate[i] != raw;
                                }
                                var trial = Evaluate(candidate, targetPositionMetres, targetRotation, geometry, minimumDegrees, maximumDegrees);
                                Remember(trial, seedIndex);
                                if (Expired()) return Finish(Failure.TimeLimit);
                                if (trial.Cost >= current.Cost) continue;
                                if (trial.Converged) return Finish(Failure.None, trial, seedIndex);
                                q = candidate;
                                current = trial;
                                damping = Math.Max(.000001, damping * .5);
                                accepted = true;
                                break;
                            }
                            if (!accepted) damping = Math.Min(10000, damping * 10);
                        }
                        if (accepted) continue;
                        bool onLimit = false;
                        for (int i = 0; i < 6; i++) onLimit |= q[i] <= lower[i] + 1e-9 || q[i] >= upper[i] - 1e-9;
                        lastReason = touchedLimit && onLimit ? Failure.JointLimitStall : Failure.Stalled;
                        break;
                    }
                }
                return Finish(iterations >= budget.MaximumIterations ? Failure.IterationLimit : lastReason);
            }
            catch (ArgumentException) { return Finish(Failure.NumericalError); }
        }

        private static Sample Evaluate(double[] q, Vector3 targetPosition, Quaternion targetRotation,
            FanucKinematicGeometry geometry, IReadOnlyList<float> lower, IReadOnlyList<float> upper)
        {
            var degrees = new float[6];
            for (int i = 0; i < 6; i++) degrees[i] = Mathf.Clamp((float)(q[i] / RadiansPerDegree), lower[i], upper[i]);
            var pose = FanucForwardKinematics.Calculate(degrees, geometry);
            var linear = targetPosition - pose.PositionMetres;
            var angular = RotationVector(targetRotation, pose.Rotation);
            var error = new[] { linear.x / LengthScaleMetres, linear.y / LengthScaleMetres, linear.z / LengthScaleMetres,
                (double)angular.x, angular.y, angular.z };
            double cost = 0;
            foreach (double value in error) cost += value * value;
            if (!Finite(cost)) throw new ArgumentException("Pose residual exceeded the numeric range.");
            return new Sample { Degrees = degrees, Position = pose.PositionMetres, Error = error, Cost = cost,
                PositionError = linear.magnitude, RotationError = angular.magnitude * Mathf.Rad2Deg };
        }

        private static double[,] Jacobian(Sample sample, FanucKinematicGeometry geometry)
        {
            var jacobian = new double[6, 6];
            var position = Vector3.zero; var rotation = Quaternion.identity;
            for (int i = 0; i < 6; i++)
            {
                var home = geometry.Joints[i];
                position += rotation * home.PositionMetres;
                var beforeJoint = rotation * home.Rotation;
                var axis = (beforeJoint * home.LocalRotationAxis.normalized).normalized;
                var linear = Vector3.Cross(axis, sample.Position - position);
                for (int row = 0; row < 3; row++)
                {
                    jacobian[row, i] = linear[row] / LengthScaleMetres;
                    jacobian[row + 3, i] = axis[row];
                }
                rotation = Quaternion.Normalize(beforeJoint * Quaternion.AngleAxis(sample.Degrees[i], home.LocalRotationAxis.normalized));
            }
            return jacobian;
        }

        private static bool Step(double[,] j, double[] error, double damping, double[] q,
            double[] lower, double[] upper, out double[] step, out bool touchedLimit)
        {
            var normal = new double[6, 7];
            for (int a = 0; a < 6; a++)
            {
                for (int row = 0; row < 6; row++) normal[a, 6] += j[row, a] * error[row];
                for (int b = 0; b < 6; b++)
                    for (int row = 0; row < 6; row++) normal[a, b] += j[row, a] * j[row, b];
                normal[a, a] += damping * damping;
            }
            var fixedAxes = new bool[6];
            touchedLimit = false;
            step = null;
            for (int pass = 0; pass <= 6; pass++)
            {
                var matrix = (double[,])normal.Clone();
                for (int i = 0; i < 6; i++) if (fixedAxes[i])
                {
                    for (int k = 0; k < 6; k++) { matrix[i, k] = 0; matrix[k, i] = 0; }
                    matrix[i, i] = 1; matrix[i, 6] = 0;
                }
                if (!LinearSolve(matrix, out step)) return false;
                bool changed = false;
                for (int i = 0; i < 6; i++)
                {
                    if (fixedAxes[i] || !((q[i] <= lower[i] + 1e-9 && step[i] < 0) ||
                        (q[i] >= upper[i] - 1e-9 && step[i] > 0))) continue;
                    fixedAxes[i] = true;
                    changed = true;
                    touchedLimit = true;
                }
                if (!changed) return true;
            }
            return false;
        }

        private static bool LinearSolve(double[,] matrix, out double[] result)
        {
            result = new double[6];
            for (int col = 0; col < 6; col++)
            {
                int pivot = col;
                for (int row = col + 1; row < 6; row++)
                    if (Math.Abs(matrix[row, col]) > Math.Abs(matrix[pivot, col])) pivot = row;
                if (!Finite(matrix[pivot, col]) || Math.Abs(matrix[pivot, col]) < 1e-18) return false;
                for (int k = col; k <= 6; k++) { double value = matrix[col, k]; matrix[col, k] = matrix[pivot, k]; matrix[pivot, k] = value; }
                for (int row = col + 1; row < 6; row++)
                {
                    double factor = matrix[row, col] / matrix[col, col];
                    for (int k = col; k <= 6; k++) matrix[row, k] -= factor * matrix[col, k];
                }
            }
            for (int row = 5; row >= 0; row--)
            {
                double value = matrix[row, 6];
                for (int col = row + 1; col < 6; col++) value -= matrix[row, col] * result[col];
                result[row] = value / matrix[row, row];
                if (!Finite(result[row])) return false;
            }
            return true;
        }

        private static Vector3 RotationVector(Quaternion desired, Quaternion actual)
        {
            var d = Quaternion.Normalize(desired * Quaternion.Inverse(actual));
            if (d.w < 0) d = new Quaternion(-d.x, -d.y, -d.z, -d.w);
            double size = Math.Sqrt((double)d.x * d.x + (double)d.y * d.y + (double)d.z * d.z);
            double factor = size < 1e-10 ? 2 : 2 * Math.Atan2(size, d.w) / size;
            return new Vector3((float)(factor * d.x), (float)(factor * d.y), (float)(factor * d.z));
        }
        private static double Norm(Vector3 p) => Math.Sqrt((double)p.x * p.x + (double)p.y * p.y + (double)p.z * p.z);
        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
        private static bool Finite(Vector3 p) => Finite(p.x) && Finite(p.y) && Finite(p.z);
        private static bool Unit(Quaternion q) => Finite(q.x) && Finite(q.y) && Finite(q.z) && Finite(q.w) &&
            Math.Abs((double)q.x * q.x + (double)q.y * q.y + (double)q.z * q.z + (double)q.w * q.w - 1) <= .0001;
    }
}
