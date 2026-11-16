using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using SIK = LineWatch.ScaraInverseKinematics;
using FIK = LineWatch.FanucInverseKinematics;

public static class VerifyRobotWaypoints
{
    private const string Folder = "Assets/04.ScriptableObject";
    private const string ScaraPath = Folder + "/ScaraLabWaypoints.asset";
    private const string FanucPath = Folder + "/FanucLabWaypoints.asset";
    private static readonly string[] ProtectedPaths = { "Assets/01.Scenes/RobotLab.unity", "Assets/03.Prefabs/ScaraRobot_Work.prefab",
        "Assets/03.Prefabs/FanucRobot_Work.prefab", "ProjectSettings/ProjectSettings.asset" };

    public static object Probe()
    {
        Require(!EditorApplication.isPlaying, "Probe in Edit Mode.");
        var scara = UnityEngine.Object.FindFirstObjectByType<ScaraJointLab>();
        var fanuc = UnityEngine.Object.FindFirstObjectByType<FanucJointLab>();
        Require(scara != null && fanuc != null, "Open RobotLab.");
        var geometry = ScaraGeometry(scara);
        var work = ScaraForwardKinematics.Calculate(20f, 60f, .1f, -80f, geometry);
        var retreat = ScaraForwardKinematics.Calculate(5f, 60f, .03f, -65f, geometry);
        var fwork = FanucForwardKinematics.Calculate(new[] { 15f, -20f, 25f, -15f, 20f, 10f }, fanuc.Geometry);
        var seeds = FanucSeeds(); var lower = fanuc.Joints.Select(j => j.MinimumDegrees).ToArray();
        var upper = fanuc.Joints.Select(j => j.MaximumDegrees).ToArray();
        var candidates = new[] { fwork.PositionMetres, fwork.PositionMetres + Vector3.up * .06f,
            fwork.PositionMetres + new Vector3(.03f, .06f, .02f) };
        return new { scara = new { work = V(work.PositionMetres), approach = V(work.PositionMetres + Vector3.up * .07f),
                retreat = V(retreat.PositionMetres), yaw = work.YawDegrees },
            fanuc = candidates.Select(p => { var r = FIK.Solve(p, fwork.Rotation, fanuc.Geometry, lower, upper, seeds);
                return new { position = V(p), rotation = Q(fwork.Rotation), r.Succeeded, reason = r.Reason.ToString(),
                    joints = r.JointDegrees.ToArray(), r.PositionErrorMetres, r.RotationErrorDegrees }; }).ToArray() };
    }

    public static object CreateDefaults()
    {
        Require(!EditorApplication.isPlaying, "Create in Edit Mode.");
        Require(AssetDatabase.LoadAssetAtPath<RobotWaypointSet>(ScaraPath) == null &&
            AssetDatabase.LoadAssetAtPath<RobotWaypointSet>(FanucPath) == null, "Defaults already exist; do not overwrite authored data.");
        var scara = AssetDatabase.LoadAssetAtPath<GameObject>(ProtectedPaths[1]).GetComponent<ScaraJointLab>();
        var fanuc = AssetDatabase.LoadAssetAtPath<GameObject>(ProtectedPaths[2]).GetComponent<FanucJointLab>();
        var g = ScaraGeometry(scara);
        var sw = ScaraForwardKinematics.Calculate(20f, 60f, .1f, -80f, g);
        var sr = ScaraForwardKinematics.Calculate(5f, 60f, .03f, -65f, g);
        var fw = FanucForwardKinematics.Calculate(new[] { 15f, -20f, 25f, -15f, 20f, 10f }, fanuc.Geometry);
        Create(ScaraPath, RobotWaypointSet.RobotType.Scara, sw.PositionMetres + Vector3.up * .07f,
            sw.PositionMetres, sr.PositionMetres, Quaternion.Euler(0, sw.YawDegrees, 0));
        Create(FanucPath, RobotWaypointSet.RobotType.Fanuc, fw.PositionMetres + Vector3.up * .06f,
            fw.PositionMetres, fw.PositionMetres + new Vector3(.03f, .06f, .02f), fw.Rotation);
        AssetDatabase.SaveAssets();
        return Edit();
    }

    public static object Edit()
    {
        Require(!EditorApplication.isPlaying, "Verify in Edit Mode.");
        var hashes = ProtectedPaths.ToDictionary(p => p, Hash);
        hashes[ScaraPath] = Hash(ScaraPath); hashes[FanucPath] = Hash(FanucPath);
        hashes["Assets/02.Scripts/LineWatch/RobotWaypointSet.cs"] = Hash("Assets/02.Scripts/LineWatch/RobotWaypointSet.cs");
        var rows = new List<object>();
        var sroot = PrefabUtility.LoadPrefabContents(ProtectedPaths[1]);
        var froot = PrefabUtility.LoadPrefabContents(ProtectedPaths[2]);
        float maxModelPosition = 0, maxModelRotation = 0, maxLine = 0, maxFanucDrift = 0, maxFanucOrientationDrift = 0;
        float minFanucMargin = float.MaxValue;
        int modelChecks = 0, scaraNodes = 0, fanucSamples = 0;
        try
        {
            var scara = sroot.GetComponent<ScaraJointLab>(); var fanuc = froot.GetComponent<FanucJointLab>();
            var sset = AssetDatabase.LoadAssetAtPath<RobotWaypointSet>(ScaraPath);
            var fset = AssetDatabase.LoadAssetAtPath<RobotWaypointSet>(FanucPath);
            Require(sset.Robot == RobotWaypointSet.RobotType.Scara && fset.Robot == RobotWaypointSet.RobotType.Fanuc, "Wrong robot data binding.");
            Require(sset.TryValidate(out var error), error); Require(fset.TryValidate(out error), error);
            int rejectedDataCases = CheckContracts(sset, fset);
            var sg = ScaraGeometry(scara);
            var sl = new SIK.JointLimits(J(scara.Axis1.MinimumDegrees, scara.Axis2.MinimumDegrees, scara.Axis3.MinimumMetres, scara.Axis4.MinimumDegrees),
                J(scara.Axis1.MaximumDegrees, scara.Axis2.MaximumDegrees, scara.Axis3.MaximumMetres, scara.Axis4.MaximumDegrees));
            var speeds = new Vector4(scara.Axis1.SpeedDegreesPerSecond, scara.Axis2.SpeedDegreesPerSecond, scara.Axis3.SpeedMetresPerSecond, scara.Axis4.SpeedDegreesPerSecond);
            var sh = new[] { scara.Axis1.transform.localRotation, scara.Axis2.transform.localRotation, scara.Axis4.transform.localRotation };
            var slideHome = scara.Axis3.transform.localPosition;
            var fh = fanuc.Joints.Select(j => j.transform.localRotation).ToArray();
            var fl = fanuc.Joints.Select(j => j.MinimumDegrees).ToArray(); var fu = fanuc.Joints.Select(j => j.MaximumDegrees).ToArray();
            var poses = new[] { new Pose(sroot.transform.position, sroot.transform.rotation), new Pose(new Vector3(2, 1, -3), Quaternion.Euler(15, 37, -10)) };
            var scurrent = J(0, 0, 0, 0); var fcurrent = new float[6];
            var roles = new[] { "Approach", "Work", "Lift", "Retreat" };
            var stargets = new[] { sset.Approach, sset.Work, sset.Approach, sset.Retreat };
            var ftargets = new[] { fset.Approach, fset.Work, fset.Approach, fset.Retreat };
            for (int index = 0; index < roles.Length; index++)
            {
                var st = stargets[index]; float yaw = Yaw(st.Rotation);
                var sr = SIK.Solve(st.PositionMetres, yaw, sg, sl);
                Require(sr.Success, "SCARA endpoint IK failed: " + roles[index]);
                var solution = sr.Solutions.Single(v => v.ElbowSign == 1);
                bool planned = index == 0
                    ? ScaraMotionPlanner.Joint(scurrent, solution, sl, speeds, out var path, out var failure)
                    : ScaraMotionPlanner.Linear(scurrent, st.PositionMetres, yaw, sg, sl, speeds, 1, out path, out failure);
                Require(planned, "SCARA segment failed: " + roles[index] + " " + failure);
                var origin = ScaraForwardKinematics.Calculate(scurrent.Q1Degrees, scurrent.Q2Degrees, scurrent.D3Metres, scurrent.Q4Degrees, sg).PositionMetres;
                for (int n = 1; n < path.Nodes.Count; n++)
                {
                    for (int k = 0; k <= 20; k++)
                    {
                        float t = k / 20f; var q = Lerp(path.Nodes[n - 1].Joints, path.Nodes[n].Joints, t);
                        var fk = ScaraForwardKinematics.Calculate(q.Q1Degrees, q.Q2Degrees, q.D3Metres, q.Q4Degrees, sg);
                        if (index > 0) maxLine = Mathf.Max(maxLine, LineDistance(fk.PositionMetres, origin, st.PositionMetres));
                        scaraNodes++;
                    }
                }
                scurrent = path.End.Joints;
                var ft = ftargets[index];
                var seeds = (new[] { fcurrent }).Concat(FanucSeeds()).ToArray();
                var fr = FIK.Solve(ft.PositionMetres, ft.Rotation, fanuc.Geometry, fl, fu, seeds);
                Require(fr.Succeeded, "FANUC endpoint IK failed: " + roles[index] + " " + fr.Reason);
                var fend = fr.JointDegrees.ToArray();
                var forigin = FanucForwardKinematics.Calculate(fcurrent, fanuc.Geometry).PositionMetres;
                for (int k = 0; k <= 100; k++)
                {
                    var q = fcurrent.Select((v, i) => Mathf.Lerp(v, fend[i], k / 100f)).ToArray();
                    for (int i = 0; i < 6; i++)
                    {
                        Require(q[i] >= fl[i] && q[i] <= fu[i], "FANUC sampled joint outside limits.");
                        minFanucMargin = Mathf.Min(minFanucMargin, q[i] - fl[i], fu[i] - q[i]);
                    }
                    var fk = FanucForwardKinematics.Calculate(q, fanuc.Geometry);
                    if (index > 0)
                    {
                        maxFanucDrift = Mathf.Max(maxFanucDrift, LineDistance(fk.PositionMetres, forigin, ft.PositionMetres));
                        maxFanucOrientationDrift = Mathf.Max(maxFanucOrientationDrift, Angle(fk.Rotation, ft.Rotation));
                    }
                    fanucSamples++;
                }
                fcurrent = fend;
                foreach (var placement in poses)
                {
                    sroot.transform.SetPositionAndRotation(placement.position, placement.rotation);
                    froot.transform.SetPositionAndRotation(placement.position, placement.rotation);
                    scara.Axis1.transform.localRotation = sh[0] * Quaternion.AngleAxis(scurrent.Q1Degrees, scara.Axis1.LocalRotationAxis);
                    scara.Axis2.transform.localRotation = sh[1] * Quaternion.AngleAxis(scurrent.Q2Degrees, scara.Axis2.LocalRotationAxis);
                    scara.Axis3.transform.localPosition = slideHome + Vector3.down * scurrent.D3Metres;
                    scara.Axis4.transform.localRotation = sh[2] * Quaternion.AngleAxis(scurrent.Q4Degrees, scara.Axis4.LocalRotationAxis);
                    for (int i = 0; i < 6; i++) fanuc.Joints[i].transform.localRotation = fh[i] * Quaternion.AngleAxis(fend[i], fanuc.Joints[i].LocalRotationAxis);
                    Measure(scara.RobotFrame, scara.Tcp, st, ref maxModelPosition, ref maxModelRotation);
                    Measure(fanuc.RobotFrame, fanuc.Tcp, ft, ref maxModelPosition, ref maxModelRotation);
                    modelChecks += 2;
                }
                rows.Add(new { role = roles[index], scaraPosition = V(st.PositionMetres), scaraYawDegrees = yaw,
                    scaraJoints = new[] { scurrent.Q1Degrees, scurrent.Q2Degrees, scurrent.D3Metres, scurrent.Q4Degrees },
                    fanucPosition = V(ft.PositionMetres), fanucRotation = Q(ft.Rotation), fanucJoints = fend,
                    fanucPositionResidual = fr.PositionErrorMetres, fanucRotationResidual = fr.RotationErrorDegrees });
            }
            Require(maxLine <= ScaraMotionPlanner.LineToleranceMetres + .000001f, "SCARA line sampling exceeded tolerance.");
            Require(hashes.All(p => Hash(p.Key) == p.Value), "Source asset changed.");
            var report = new { passed = true, utc = DateTime.UtcNow.ToString("o"), uniqueTargets = 6, routeEndpoints = 8,
                modelChecks, placements = 2, maxModelPositionErrorMetres = maxModelPosition, maxModelRotationErrorDegrees = maxModelRotation,
                scaraSampledPathPoses = scaraNodes, maxScaraLineDeviationMetres = maxLine,
                fanucSampledJointPoses = fanucSamples, minimumFanucJointLimitMarginDegrees = minFanucMargin,
                maxFanucJointPathLineDeviationMetres = maxFanucDrift, maxFanucJointPathOrientationDriftDegrees = maxFanucOrientationDrift,
                scaraApproachHeightMetres = sset.Approach.PositionMetres.y - sset.Work.PositionMetres.y,
                fanucApproachHeightMetres = fset.Approach.PositionMetres.y - fset.Work.PositionMetres.y,
                rejectedDataCases, quaternionSignEquivalencePassed = true,
                sourceAssetsUnchanged = true, hashes, directButtonClicks = 0, actualRobotMoves = 0,
                scope = "LW-08 process 1: authored RobotFrame targets, pure endpoint IK/path samples and isolated prefab model checks; no runtime sequence, grasp, shared handoff, collision, acceleration or industrial safety validation", rows };
            Directory.CreateDirectory("Logs/LW08"); File.WriteAllText("Logs/LW08/waypoints-edit-verification.json", JsonConvert.SerializeObject(report, Formatting.Indented));
            return new { report.passed, report.uniqueTargets, report.routeEndpoints, report.modelChecks, report.maxModelPositionErrorMetres,
                report.maxModelRotationErrorDegrees, report.scaraSampledPathPoses, report.maxScaraLineDeviationMetres,
                report.fanucSampledJointPoses, report.minimumFanucJointLimitMarginDegrees, report.maxFanucJointPathLineDeviationMetres,
                report.maxFanucJointPathOrientationDriftDegrees, report.rejectedDataCases, report.quaternionSignEquivalencePassed, report.sourceAssetsUnchanged };
        }
        finally { PrefabUtility.UnloadPrefabContents(sroot); PrefabUtility.UnloadPrefabContents(froot); }
    }

    private static int CheckContracts(RobotWaypointSet scara, RobotWaypointSet fanuc)
    {
        int rejected = 0;
        void Reject(Action<SerializedObject> edit)
        {
            var copy = UnityEngine.Object.Instantiate(scara);
            try
            {
                var s = new SerializedObject(copy); edit(s); s.ApplyModifiedPropertiesWithoutUndo();
                Require(!copy.TryValidate(out var error) && !string.IsNullOrEmpty(error), "Invalid waypoint data accepted."); rejected++;
            }
            finally { UnityEngine.Object.DestroyImmediate(copy); }
        }
        Reject(s => s.FindProperty("work").FindPropertyRelative("positionMetres").vector3Value = new Vector3(float.NaN, 0, 0));
        Reject(s => s.FindProperty("work").FindPropertyRelative("rotation").quaternionValue = default);
        Reject(s => s.FindProperty("approach").FindPropertyRelative("positionMetres").vector3Value = scara.Work.PositionMetres - Vector3.up * .01f);
        Reject(s => { foreach (string role in new[] { "approach", "work", "retreat" })
            s.FindProperty(role).FindPropertyRelative("rotation").quaternionValue = Quaternion.Euler(5, 0, 0); });
        var fcopy = UnityEngine.Object.Instantiate(fanuc);
        try
        {
            var q = fcopy.Work.Rotation; var s = new SerializedObject(fcopy);
            s.FindProperty("work").FindPropertyRelative("rotation").quaternionValue = new Quaternion(-q.x, -q.y, -q.z, -q.w);
            s.ApplyModifiedPropertiesWithoutUndo(); Require(fcopy.TryValidate(out var error), "Quaternion sign changed the represented orientation: " + error);
        }
        finally { UnityEngine.Object.DestroyImmediate(fcopy); }
        return rejected;
    }

    private static void Create(string path, RobotWaypointSet.RobotType robot, Vector3 approach, Vector3 work, Vector3 retreat, Quaternion rotation)
    {
        var asset = ScriptableObject.CreateInstance<RobotWaypointSet>();
        var s = new SerializedObject(asset); s.FindProperty("robot").enumValueIndex = (int)robot;
        Set(s, "approach", approach, rotation); Set(s, "work", work, rotation); Set(s, "retreat", retreat, rotation);
        s.ApplyModifiedPropertiesWithoutUndo(); Require(asset.TryValidate(out var error), error); AssetDatabase.CreateAsset(asset, path);
    }
    private static void Set(SerializedObject s, string name, Vector3 p, Quaternion r)
    { var t = s.FindProperty(name); t.FindPropertyRelative("positionMetres").vector3Value = p; t.FindPropertyRelative("rotation").quaternionValue = r; }
    private static void Measure(Transform frame, Transform tcp, RobotWaypointSet.Target target, ref float maxP, ref float maxR)
    {
        float p = Vector3.Distance(frame.InverseTransformPoint(tcp.position), target.PositionMetres);
        float r = Angle(Quaternion.Inverse(frame.rotation) * tcp.rotation, target.Rotation);
        Require(p <= .000101f && r <= .051f, "Solved target does not match isolated model TCP.");
        maxP = Mathf.Max(maxP, p); maxR = Mathf.Max(maxR, r);
    }
    private static float Yaw(Quaternion q) => Vector3.SignedAngle(Vector3.forward, q * Vector3.forward, Vector3.up);
    private static SIK.JointValues J(float a, float b, float c, float d) => new SIK.JointValues(a, b, c, d);
    private static SIK.JointValues Lerp(SIK.JointValues a, SIK.JointValues b, float t) => J(Mathf.Lerp(a.Q1Degrees, b.Q1Degrees, t),
        Mathf.Lerp(a.Q2Degrees, b.Q2Degrees, t), Mathf.Lerp(a.D3Metres, b.D3Metres, t), Mathf.Lerp(a.Q4Degrees, b.Q4Degrees, t));
    private static float LineDistance(Vector3 p, Vector3 a, Vector3 b)
    { var d = b - a; return Vector3.Distance(p, a + d * Mathf.Clamp01(Vector3.Dot(p - a, d) / d.sqrMagnitude)); }
    private static float Angle(Quaternion a, Quaternion b)
    { var d = Quaternion.Inverse(a) * b; return (float)(2 * Math.Atan2(Math.Sqrt((double)d.x * d.x + (double)d.y * d.y + (double)d.z * d.z), Math.Abs(d.w)) * 180 / Math.PI); }

    private static ScaraForwardKinematics.Geometry ScaraGeometry(ScaraJointLab lab)
    {
        var s = new SerializedObject(lab);
        return new ScaraForwardKinematics.Geometry(s.FindProperty("fkLink1Metres").floatValue,
            s.FindProperty("fkLink2Metres").floatValue, s.FindProperty("fkHeight12Metres").floatValue,
            s.FindProperty("fkHeight23Metres").floatValue, s.FindProperty("fkToolOffsetMetres").vector3Value,
            s.FindProperty("fkHomeToolYawDegrees").floatValue);
    }
    private static float[][] FanucSeeds() => new[] { new float[6], new[] { 10f, -15f, 25f, 15f, 20f, -10f }, new[] { -10f, 15f, -25f, -15f, -20f, 10f } };
    private static float[] V(Vector3 v) => new[] { v.x, v.y, v.z };
    private static float[] Q(Quaternion q) => new[] { q.x, q.y, q.z, q.w };
    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))); }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
