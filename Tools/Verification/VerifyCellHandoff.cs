using System;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;

public static class VerifyCellHandoff
{
    private static readonly string[] Protected = {
        "Assets/01.Scenes/RobotLab.unity", "ProjectSettings/ProjectSettings.asset",
        "Assets/03.Prefabs/ScaraRobot_Work.prefab", "Assets/03.Prefabs/FanucRobot_Work.prefab",
        "Assets/04.ScriptableObject/ScaraLabWaypoints.asset", "Assets/04.ScriptableObject/FanucLabWaypoints.asset"
    };

    public static object Edit()
    {
        Require(!EditorApplication.isPlaying, "Use Edit Mode.");
        var hashes = Protected.ToDictionary(p => p, Hash);
        var sr = PrefabUtility.LoadPrefabContents(Protected[2]);
        var fr = PrefabUtility.LoadPrefabContents(Protected[3]);
        try
        {
            var s = sr.GetComponent<ScaraJointLab>();
            var f = fr.GetComponent<FanucJointLab>();
            var sg = sr.GetComponent<RobotGripper>(); var fg = fr.GetComponent<RobotGripper>();
            var set = AssetDatabase.LoadAssetAtPath<RobotWaypointSet>(Protected[4]);
            var limits = new ScaraInverseKinematics.JointLimits(
                new ScaraInverseKinematics.JointValues(s.Axis1.MinimumDegrees, s.Axis2.MinimumDegrees, s.Axis3.MinimumMetres, s.Axis4.MinimumDegrees),
                new ScaraInverseKinematics.JointValues(s.Axis1.MaximumDegrees, s.Axis2.MaximumDegrees, s.Axis3.MaximumMetres, s.Axis4.MaximumDegrees));
            var si = ScaraInverseKinematics.Solve(set.Retreat.PositionMetres, 0, s.Geometry, limits);
            Require(si.Success, "SCARA candidate failed.");
            var sq = si.Solutions.Single(v => v.ElbowSign == 1).Joints;
            var fq = new[] { 0f, -20f, 20f, -45f, 0f, -45f };
            var fp = FanucForwardKinematics.Calculate(fq, f.Geometry);
            var st = new Pose(set.Retreat.PositionMetres, set.Retreat.Rotation);
            var ft = new Pose(fp.PositionMetres, fp.Rotation);
            var sc = ContactPose(sg, st); var fc = ContactPose(fg, ft);
            var station = new Vector3(0, 1.2f, 0);
            var sy = Quaternion.identity; var fy = Quaternion.Euler(0, 180, 0);
            Require(RobotGripper.RotationError(Quaternion.Inverse(sr.transform.rotation) * s.RobotFrame.rotation, Quaternion.identity) < .005f &&
                RobotGripper.RotationError(Quaternion.Inverse(fr.transform.rotation) * f.RobotFrame.rotation, Quaternion.identity) < .005f, "Unexpected prefab-to-frame rotation.");
            var spos = station - sy * (sr.transform.InverseTransformPoint(s.RobotFrame.position) + sc.position);
            var fpos = station - fy * (fr.transform.InverseTransformPoint(f.RobotFrame.position) + fc.position);
            sr.transform.SetPositionAndRotation(spos, sy); fr.transform.SetPositionAndRotation(fpos, fy);
            s.Axis1.transform.localRotation *= Quaternion.AngleAxis(sq.Q1Degrees, s.Axis1.LocalRotationAxis);
            s.Axis2.transform.localRotation *= Quaternion.AngleAxis(sq.Q2Degrees, s.Axis2.LocalRotationAxis);
            s.Axis3.transform.localPosition += Vector3.down * sq.D3Metres;
            s.Axis4.transform.localRotation *= Quaternion.AngleAxis(sq.Q4Degrees, s.Axis4.LocalRotationAxis);
            for (int i = 0; i < 6; i++) f.Joints[i].transform.localRotation *= Quaternion.AngleAxis(fq[i], f.Joints[i].LocalRotationAxis);
            float modelPosition = Vector3.Distance(sg.Contact.position, fg.Contact.position);
            float modelAngle = RobotGripper.RotationError(sg.Contact.rotation, fg.Contact.rotation);
            Require(modelPosition <= .00001f && modelAngle <= .005f, "Candidate model contact poses disagree: " + modelPosition + "m / " + modelAngle + "deg; S=" + sg.Contact.position + " F=" + fg.Contact.position + " rot=" + sg.Contact.rotation + " / " + fg.Contact.rotation);
            Require(Vector3.Distance(sg.Contact.position, station) <= .00001f, "World station mismatch.");
            var lower = f.Joints.Select(j => j.MinimumDegrees).ToArray();
            var upper = f.Joints.Select(j => j.MaximumDegrees).ToArray();
            float margin = fq.Select((q, i) => Mathf.Min(q - lower[i], upper[i] - q)).Min();
            var rows = new System.Collections.Generic.List<object>();
            var current = new float[6];
            foreach (var offset in new[] { Vector3.up * .06f, Vector3.zero, Vector3.up * .06f, new Vector3(.03f, .06f, .02f) })
            {
                // Match the runtime policy: no authored solution is injected as an IK seed.
                var result = FanucInverseKinematics.Solve(ft.position + offset, ft.rotation, f.Geometry, lower, upper,
                    new[] { current, new float[6], new[] { 10f, -15f, 25f, 15f, 20f, -10f }, new[] { -10f, 15f, -25f, -15f, -20f, 10f } });
                rows.Add(new { offset = V(offset), result.Succeeded, reason = result.Reason.ToString(), result.PositionErrorMetres, result.RotationErrorDegrees });
                if (result.Succeeded) current = result.JointDegrees.ToArray();
            }
            Require(margin == 0, "Expected boundary candidate changed; reassess the fixture.");
            Require(hashes.All(p => Hash(p.Key) == p.Value), "Protected source changed.");
            var report = new { checksPassed = true, candidateAccepted = false, utc = DateTime.UtcNow.ToString("o"),
                worldGripPointMetres = V(station), scaraPrefabRootMetres = V(spos), fanucPrefabRootMetres = V(fpos), fanucBaseYawDegrees = 180,
                scaraJoints = new[] { sq.Q1Degrees, sq.Q2Degrees, sq.D3Metres, sq.Q4Degrees }, fanucJoints = fq,
                sharedModelPositionErrorMetres = modelPosition, sharedModelRotationErrorDegrees = modelAngle,
                minimumFanucJointMarginDegrees = margin, fanucRoute = rows, orientationGrid = Probe(),
                reason = "Matching the existing single GripPoint orientation produces a zero-margin wrist candidate. This is a sampled candidate, not a proof of global infeasibility. Do not freeze the cell layout on it.",
                next = "Define robot-specific grasp frames relative to one product pose, then validate the contact geometry and IK before authoring the cell scene.",
                sourcesUnchanged = true, hashes, directButtonClicks = 0, actualRobotMoves = 0,
                scope = "LW-09 process 1: isolated geometric candidate and fixed-budget IK diagnostics only; no scene, motion, collision or handoff validation" };
            Directory.CreateDirectory("Logs/LW09");
            File.WriteAllText("Logs/LW09/handoff-candidate-verification.json", JsonConvert.SerializeObject(report, Formatting.Indented));
            return JsonConvert.SerializeObject(new { report.checksPassed, report.candidateAccepted, report.sharedModelPositionErrorMetres,
                report.sharedModelRotationErrorDegrees, report.minimumFanucJointMarginDegrees, report.fanucRoute, report.sourcesUnchanged });
        }
        finally { PrefabUtility.UnloadPrefabContents(sr); PrefabUtility.UnloadPrefabContents(fr); }
    }

    private static Pose ContactPose(RobotGripper g, Pose tcp) => new Pose(tcp.position + tcp.rotation * g.Tcp.InverseTransformPoint(g.Contact.position),
        tcp.rotation * (Quaternion.Inverse(g.Tcp.rotation) * g.Contact.rotation));
    private static float[] V(Vector3 v) => new[] { v.x, v.y, v.z };
    private static string Hash(string p) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(p))); }
    private static void Require(bool ok, string message) { if (!ok) throw new InvalidOperationException(message); }

    public static object Probe()
    {
        if (EditorApplication.isPlaying) throw new InvalidOperationException("Use Edit Mode.");
        var root = PrefabUtility.LoadPrefabContents("Assets/03.Prefabs/FanucRobot_Work.prefab");
        try
        {
            var f = root.GetComponent<FanucJointLab>();
            float best = -1, bestMargin = -1;
            float[] selected = null;
            Quaternion rotation = Quaternion.identity;
            foreach (float a in new[] { -45f, -30f, 0f, 30f, 45f })
            foreach (float b in new[] { -45f, -30f, 0f, 30f, 45f })
            foreach (float c in new[] { -45f, -30f, 0f, 30f, 45f })
            foreach (float d in new[] { -45f, -30f, 0f, 30f, 45f })
            foreach (float e in new[] { -45f, -30f, 0f, 30f, 45f })
            {
                var q = new[] { 0f, a, b, c, d, e };
                var pose = FanucForwardKinematics.Calculate(q, f.Geometry);
                float dot = Vector3.Dot(pose.Rotation * Vector3.up, Vector3.up);
                float margin = q.Min(v => 45f - Mathf.Abs(v));
                if (dot < best - .000001f || (Mathf.Abs(dot - best) <= .000001f && margin <= bestMargin)) continue;
                bestMargin = margin;
                best = dot; selected = q; rotation = pose.Rotation;
            }
            return JsonConvert.SerializeObject(new { bestUpAlignment = best, bestMargin, joints = selected, rotation = new[] { rotation.x, rotation.y, rotation.z, rotation.w } });
        }
        finally { PrefabUtility.UnloadPrefabContents(root); }
    }
}
