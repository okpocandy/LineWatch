using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading.Tasks;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using SIK = LineWatch.ScaraInverseKinematics;

public static class VerifyCellGrasps
{
    private const string PartPath = "Assets/03.Prefabs/HandoffTrialPart.prefab";
    private static readonly Vector3 Station = new Vector3(0, 1.2f, 0);
    private static readonly Quaternion FanucBaseRotation = Quaternion.Euler(0, 180, 0);
    private static readonly float[] FanucWorkJoints = { 0, -20, 20, 0, 20, 0 };
    private static readonly string[] Protected = {
        "Assets/01.Scenes/RobotLab.unity", "ProjectSettings/ProjectSettings.asset",
        "Assets/03.Prefabs/ScaraRobot_Work.prefab", "Assets/03.Prefabs/FanucRobot_Work.prefab",
        "Assets/03.Prefabs/GripTrialPart.prefab", "Assets/04.ScriptableObject/ScaraLabWaypoints.asset",
        "Assets/04.ScriptableObject/FanucLabWaypoints.asset",
        "Assets/02.Scripts/LineWatch/GrippableItem.cs", "Assets/02.Scripts/LineWatch/RobotGripper.cs",
        "Assets/02.Scripts/LineWatch/RobotGripperLab.cs", "Assets/02.Scripts/LineWatch/RobotWaypointSequence.cs",
        "Assets/02.Scripts/LineWatch/ScaraForwardKinematics.cs", "Assets/02.Scripts/LineWatch/FanucForwardKinematics.cs",
        "Assets/02.Scripts/LineWatch/ScaraInverseKinematics.cs", "Assets/02.Scripts/LineWatch/FanucInverseKinematics.cs"
    };

    public static object Create()
    {
        Require(!EditorApplication.isPlaying, "Use Edit Mode.");
        if (AssetDatabase.LoadAssetAtPath<GameObject>(PartPath) != null) return Edit();
        var root = PrefabUtility.LoadPrefabContents(Protected[4]);
        try
        {
            root.name = "HandoffTrialPart";
            root.GetComponent<BoxCollider>().size = Vector3.one * .012f;
            root.transform.Find("Visual").localScale = Vector3.one * .012f;
            var scaraPoint = root.transform.Find("GripPoint"); scaraPoint.name = "SCARA_GripPoint";
            var fanucPoint = new GameObject("FANUC_GripPoint").transform; fanucPoint.SetParent(root.transform, false);
            var fanuc = AssetDatabase.LoadAssetAtPath<GameObject>(Protected[3]).GetComponent<FanucJointLab>();
            fanucPoint.localRotation = FanucBaseRotation * FanucForwardKinematics.Calculate(FanucWorkJoints, fanuc.Geometry).Rotation;
            var serialized = new SerializedObject(root.GetComponent<GrippableItem>());
            serialized.FindProperty("scaraGripPoint").objectReferenceValue = scaraPoint;
            serialized.FindProperty("fanucGripPoint").objectReferenceValue = fanucPoint;
            serialized.ApplyModifiedPropertiesWithoutUndo();
            Require(PrefabUtility.SaveAsPrefabAsset(root, PartPath) != null, "Part save failed.");
        }
        finally { PrefabUtility.UnloadPrefabContents(root); }
        return Edit();
    }

    public static object Edit()
    {
        Require(!EditorApplication.isPlaying, "Use Edit Mode.");
        var hashes = Hashes();
        var sr = PrefabUtility.LoadPrefabContents(Protected[2]);
        var fr = PrefabUtility.LoadPrefabContents(Protected[3]);
        var pr = PrefabUtility.LoadPrefabContents(PartPath);
        try
        {
            var s = sr.GetComponent<ScaraJointLab>(); var f = fr.GetComponent<FanucJointLab>();
            var sg = sr.GetComponent<RobotGripper>(); var fg = fr.GetComponent<RobotGripper>(); var part = pr.GetComponent<GrippableItem>();
            var sw = ScaraWork(); var fw = FanucWork(f);
            Place(sr.transform, sg, sw, part.GetGripPoint(RobotWaypointSet.RobotType.Scara), Quaternion.identity);
            Place(fr.transform, fg, fw, part.GetGripPoint(RobotWaypointSet.RobotType.Fanuc), FanucBaseRotation);
            var limits = new SIK.JointLimits(J(s.Axis1.MinimumDegrees, s.Axis2.MinimumDegrees, s.Axis3.MinimumMetres, s.Axis4.MinimumDegrees),
                J(s.Axis1.MaximumDegrees, s.Axis2.MaximumDegrees, s.Axis3.MaximumMetres, s.Axis4.MaximumDegrees));
            var speeds = new Vector4(s.Axis1.SpeedDegreesPerSecond, s.Axis2.SpeedDegreesPerSecond, s.Axis3.SpeedMetresPerSecond, s.Axis4.SpeedDegreesPerSecond);
            var sh = new[] { s.Axis1.transform.localRotation, s.Axis2.transform.localRotation, s.Axis4.transform.localRotation };
            var slide = s.Axis3.transform.localPosition; var fh = f.Joints.Select(j => j.transform.localRotation).ToArray();
            var lower = f.Joints.Select(j => j.MinimumDegrees).ToArray(); var upper = f.Joints.Select(j => j.MaximumDegrees).ToArray();
            var scurrent = J(0, 0, 0, 0); var fcurrent = new float[6];
            float minMargin = float.MaxValue, maxModelPosition = 0, maxModelAngle = 0;
            int pathSamples = 0; var rows = new List<object>();
            for (int i = 0; i < 4; i++)
            {
                var sp = sw.position + (i == 1 ? Vector3.zero : Vector3.up * .07f) + (i == 3 ? new Vector3(.02f, 0, .02f) : Vector3.zero);
                var fp = fw.position + (i == 1 ? Vector3.zero : Vector3.up * .06f) + (i == 3 ? new Vector3(.03f, 0, .02f) : Vector3.zero);
                var si = SIK.Solve(sp, 0, s.Geometry, limits); Require(si.Success, "SCARA endpoint failed at " + i + ": " + si.Failure);
                var sq = si.Solutions.Single(v => v.ElbowSign == 1);
                bool planned = i == 0 ? ScaraMotionPlanner.Joint(scurrent, sq, limits, speeds, out var path, out var failure)
                    : ScaraMotionPlanner.Linear(scurrent, sp, 0, s.Geometry, limits, speeds, 1, out path, out failure);
                Require(planned, "SCARA path failed: " + failure);
                scurrent = path.End.Joints;
                var fi = FanucInverseKinematics.Solve(fp, fw.rotation, f.Geometry, lower, upper, Seeds(fcurrent));
                Require(fi.Succeeded, "FANUC endpoint failed: " + fi.Reason);
                var next = fi.JointDegrees.ToArray();
                for (int n = 0; n <= 100; n++)
                {
                    for (int a = 0; a < 6; a++)
                    {
                        float q = Mathf.Lerp(fcurrent[a], next[a], n / 100f);
                        Require(q >= lower[a] && q <= upper[a], "FANUC sampled joint outside limits.");
                        minMargin = Mathf.Min(minMargin, q - lower[a], upper[a] - q);
                    }
                    pathSamples++;
                }
                fcurrent = next;
                s.Axis1.transform.localRotation = sh[0] * Quaternion.AngleAxis(scurrent.Q1Degrees, s.Axis1.LocalRotationAxis);
                s.Axis2.transform.localRotation = sh[1] * Quaternion.AngleAxis(scurrent.Q2Degrees, s.Axis2.LocalRotationAxis);
                s.Axis3.transform.localPosition = slide + Vector3.down * scurrent.D3Metres;
                s.Axis4.transform.localRotation = sh[2] * Quaternion.AngleAxis(scurrent.Q4Degrees, s.Axis4.LocalRotationAxis);
                for (int a = 0; a < 6; a++) f.Joints[a].transform.localRotation = fh[a] * Quaternion.AngleAxis(next[a], f.Joints[a].LocalRotationAxis);
                CheckModel(sg, new Pose(sp, sw.rotation), ref maxModelPosition, ref maxModelAngle);
                CheckModel(fg, new Pose(fp, fw.rotation), ref maxModelPosition, ref maxModelAngle);
                if (i == 1)
                {
                    CheckPartPose(sg, part.GetGripPoint(RobotWaypointSet.RobotType.Scara));
                    CheckPartPose(fg, part.GetGripPoint(RobotWaypointSet.RobotType.Fanuc));
                    CheckFit(sg, fg, pr.transform);
                }
                rows.Add(new { step = i, scaraTcp = V(sp), fanucTcp = V(fp), fanucJoints = next });
            }
            Require(minMargin > 10, "Candidate lacks expected joint margin.");
            Require(Unchanged(hashes), "Protected source changed.");
            var report = new { passed = true, endpoints = 8, modelChecks = 8, fanucJointSamples = pathSamples,
                minimumFanucJointMarginDegrees = minMargin, maxModelPositionErrorMetres = maxModelPosition, maxModelRotationErrorDegrees = maxModelAngle,
                partSizeMetres = V(pr.GetComponent<BoxCollider>().size), worldItemPositionMetres = V(Station), worldItemRotation = new[] { 0, 0, 0, 1 },
                scaraRootMetres = V(sr.transform.position), fanucRootMetres = V(fr.transform.position), fanucBaseYawDegrees = 180,
                robotGraspRotationDifferenceDegrees = RobotGripper.RotationError(part.GetGripPoint(RobotWaypointSet.RobotType.Scara).localRotation, part.GetGripPoint(RobotWaypointSet.RobotType.Fanuc).localRotation),
                sourceAssetsUnchanged = true, hashes, rows,
                scope = "LW-09 process 2: isolated product-specific grasp frames, static mesh bounds and joint-limited route candidates; no collision-free path, force grip, cell scene or coordinated handoff claim" };
            Write("grasp-edit-verification.json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.endpoints, report.fanucJointSamples, report.minimumFanucJointMarginDegrees,
                report.maxModelPositionErrorMetres, report.maxModelRotationErrorDegrees, report.robotGraspRotationDifferenceDegrees, report.sourceAssetsUnchanged });
        }
        finally { PrefabUtility.UnloadPrefabContents(sr); PrefabUtility.UnloadPrefabContents(fr); PrefabUtility.UnloadPrefabContents(pr); }
    }

    public static async Task<object> Play()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh unpaused RobotLab Play Mode.");
        var hashes = Hashes();
        var s = UnityEngine.Object.FindFirstObjectByType<ScaraJointLab>(); var f = UnityEngine.Object.FindFirstObjectByType<FanucJointLab>();
        var sg = s.GetComponent<RobotGripper>(); var fg = f.GetComponent<RobotGripper>();
        var robots = new[] { sg, fg };
        var sequences = robots.Select(g => g.GetComponent<RobotWaypointSequence>()).ToArray();
        var labs = robots.Select(g => g.GetComponent<RobotGripperLab>()).ToArray();
        Require(sequences.All(v => v.State == RobotWaypointSequence.RunState.Idle) && robots.All(g => g.HeldItem == null), "Fresh fixtures required.");
        var roots = robots.Select(g => new Pose(g.transform.position, g.transform.rotation)).ToArray();
        var oldSets = sequences.Select(v => new SerializedObject(v).FindProperty("waypoints").objectReferenceValue).ToArray();
        var oldLabSets = labs.Select(v => new SerializedObject(v).FindProperty("waypoints").objectReferenceValue).ToArray();
        var oldParts = labs.Select(v => new SerializedObject(v).FindProperty("trialPrefab").objectReferenceValue).ToArray();
        var copies = new RobotWaypointSet[2]; GrippableItem item = null;
        float scale = Time.timeScale; int rejected = 0; float maxPosition = 0, maxAngle = 0;
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PartPath).GetComponent<GrippableItem>();
        void Reject(Func<bool> request, string name)
        {
            var p = item.Body.position; var q = item.Body.rotation; var owner = item.Owner;
            Require(!request() && item.Body.position.Equals(p) && item.Body.rotation.Equals(q) && item.Owner == owner, "Rejected request mutated product: " + name);
            rejected++;
        }
        void Measure(RobotGripper g)
        {
            var kind = g.Scara != null ? RobotWaypointSet.RobotType.Scara : RobotWaypointSet.RobotType.Fanuc;
            Require(item.TryGetGripPose(kind, out var pose), "Missing selected frame.");
            float p = Vector3.Distance(g.Contact.position, pose.position), a = RobotGripper.RotationError(g.Contact.rotation, pose.rotation);
            Require(p <= .002f && a <= .5f, "Actual selected frame mismatch.");
            maxPosition = Mathf.Max(maxPosition, p); maxAngle = Mathf.Max(maxAngle, a);
        }
        try
        {
            Time.timeScale = 10;
            var sw = ScaraWork(); var fw = FanucWork(f);
            Place(s.transform, sg, sw, prefab.GetGripPoint(RobotWaypointSet.RobotType.Scara), Quaternion.identity);
            Place(f.transform, fg, fw, prefab.GetGripPoint(RobotWaypointSet.RobotType.Fanuc), FanucBaseRotation);
            var works = new[] { sw, fw };
            // Exercise the existing preparation API with both robot-specific frames, without UI callbacks.
            for (int i = 0; i < 2; i++)
            {
                copies[i] = UnityEngine.Object.Instantiate((RobotWaypointSet)oldSets[i]);
                Configure(copies[i], works[i], i == 0 ? .07f : .06f, i == 0 ? new Vector3(.02f, 0, .02f) : new Vector3(.03f, 0, .02f));
                SetObject(sequences[i], "waypoints", copies[i]); SetObject(labs[i], "waypoints", copies[i]); SetObject(labs[i], "trialPrefab", prefab);
                Require(labs[i].TryPrepare(out var error), error);
                var prepared = labs[i].TrialItem;
                Require(Vector3.Distance(prepared.Body.position, Station) < .00001f && RobotGripper.RotationError(prepared.Body.rotation, Quaternion.identity) < .005f,
                    "Preparation must preserve canonical product orientation for either robot.");
                UnityEngine.Object.Destroy(prepared.gameObject);
            }
            await Task.Delay(30);
            item = UnityEngine.Object.Instantiate(prefab);
            item.Body.position = Station; item.Body.rotation = Quaternion.identity;
            int itemId = item.GetInstanceID();
            Reject(() => item.TryGetGripPose((RobotWaypointSet.RobotType)99, out _), "unknown robot");
            var point = item.GetGripPoint(RobotWaypointSet.RobotType.Fanuc);
            point.gameObject.SetActive(false);
            Reject(() => fg.TryGrip(item, out _), "inactive paired frame");
            point.gameObject.SetActive(true);
            // Accepting then pausing before a tick verifies automatic preflight uses the selected frame.
            Require(sequences[0].TryStartPickPlace(sg, item, out var e), e); sequences[0].Pause();
            Require(s.TryMoveTcp(sw.position, 0, ScaraJointLab.IkSelection.PositiveElbow, ScaraJointLab.MotionMode.Joint, out _, out e), e);
            await Until(() => Rest(sg)); Measure(sg);
            var before = item.Body.position; var beforeRotation = item.Body.rotation;
            Require(sg.TryGrip(item, out e), e);
            Require(item.Owner == sg && item.Body.position.Equals(before) && item.Body.rotation.Equals(beforeRotation), "SCARA claim snapped product.");
            Reject(() => fg.TryGrip(item, out _), "second owner");
            Require(sg.TryRelease(out e), e);
            Require(item.Owner == null && item.Body.position.Equals(before) && item.Body.rotation.Equals(beforeRotation), "SCARA release changed product pose.");
            // Withdraw the first robot before testing the second. This is not a cell collision/occupancy controller.
            Require(s.TryMoveTcp(sw.position + Vector3.up * .07f, 0, ScaraJointLab.IkSelection.PositiveElbow, ScaraJointLab.MotionMode.Linear, out _, out e), e);
            await Until(() => Rest(sg));
            Require(sequences[1].TryStartPickPlace(fg, item, out e), e); sequences[1].Pause();
            Require(f.TryMoveTcp(fw.position, fw.rotation, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out e), e);
            await Until(() => Rest(fg)); Measure(fg);
            Require(RobotGripper.RotationError(item.GripPose.rotation, fg.Contact.rotation) > 80, "Fixture does not distinguish legacy and FANUC frames.");
            var frameRotation = point.localRotation; point.localRotation *= Quaternion.Euler(0, 3, 0);
            Reject(() => fg.TryGrip(item, out _), "wrong selected orientation"); point.localRotation = frameRotation;
            item.Body.position += Vector3.up * .01f;
            Reject(() => fg.TryGrip(item, out _), "10mm physical pose mismatch"); item.Body.position = before;
            Time.timeScale = 0; Reject(() => fg.TryGrip(item, out _), "paused clock"); Time.timeScale = 10;
            Require(fg.TryGrip(item, out e), e);
            Require(item.Owner == fg && item.GetInstanceID() == itemId && item.Body.position.Equals(before) && item.Body.rotation.Equals(beforeRotation), "FANUC claim changed product or identity.");
            Require(f.TryMoveTcp(fw.position + Vector3.up * .06f, fw.rotation, FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out e), e);
            await Until(() => Rest(fg)); await Task.Delay(30); Measure(fg);
            float liftError = Vector3.Distance(item.Body.position, Station + Vector3.up * .06f);
            float orientationError = RobotGripper.RotationError(item.Body.rotation, Quaternion.identity);
            Require(item.Owner == fg && liftError <= .002f && orientationError <= .5f, "Selected-frame carry lost the intended product pose.");
            Require(fg.TryRelease(out e), e); var released = item.Body.position; var releasedRotation = item.Body.rotation;
            await Task.Delay(50);
            Require(item.Owner == null && fg.HeldItem == null && item.Body.position.Equals(released) && item.Body.rotation.Equals(releasedRotation), "Released product drifted.");
            Require(Unchanged(hashes), "Source assets changed during Play.");
            var report = new { passed = true, preparations = 2, automaticPreflights = 2, sameItemGrips = 2, releases = 2,
                rejectedRequests = rejected, maxSelectedFramePositionErrorMetres = maxPosition, maxSelectedFrameRotationErrorDegrees = maxAngle,
                fanucLiftErrorMetres = liftError, finalProductOrientationErrorDegrees = orientationError,
                identityAndCanonicalPosePreserved = true, sourceAssetsUnchanged = true, hashes, directButtonClicks = 0,
                scope = "10x Play API verification of grasp-frame selection, one unchanged shared item, ownership, physical pose, lifting and release; no autonomous two-robot cell handoff or collision/force validation" };
            Write("grasp-play-verification.json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.preparations, report.automaticPreflights, report.sameItemGrips, report.releases,
                report.rejectedRequests, report.fanucLiftErrorMetres, report.finalProductOrientationErrorDegrees, report.sourceAssetsUnchanged });
        }
        finally
        {
            Time.timeScale = scale;
            for (int i = 0; i < 2; i++)
            {
                sequences[i].Pause(); robots[i].Stop(); if (robots[i].HeldItem != null) robots[i].TryRelease(out _);
                robots[i].transform.SetPositionAndRotation(roots[i].position, roots[i].rotation);
                SetObject(sequences[i], "waypoints", oldSets[i]); SetObject(labs[i], "waypoints", oldLabSets[i]); SetObject(labs[i], "trialPrefab", oldParts[i]);
                if (copies[i] != null) UnityEngine.Object.Destroy(copies[i]);
                if (labs[i].TrialItem != null) UnityEngine.Object.Destroy(labs[i].TrialItem.gameObject);
            }
            if (item != null) UnityEngine.Object.Destroy(item.gameObject);
        }
    }

    private static void Configure(RobotWaypointSet set, Pose work, float height, Vector3 retreat)
    {
        var serialized = new SerializedObject(set);
        foreach (var role in new[] { "approach", "work", "retreat" })
        {
            var target = serialized.FindProperty(role);
            target.FindPropertyRelative("positionMetres").vector3Value = work.position + (role == "work" ? Vector3.zero : Vector3.up * height) + (role == "retreat" ? retreat : Vector3.zero);
            target.FindPropertyRelative("rotation").quaternionValue = work.rotation;
        }
        serialized.ApplyModifiedPropertiesWithoutUndo(); Require(set.TryValidate(out var error), error);
    }

    private static void SetObject(UnityEngine.Object target, string field, UnityEngine.Object value)
    { var serialized = new SerializedObject(target); serialized.FindProperty(field).objectReferenceValue = value; serialized.ApplyModifiedPropertiesWithoutUndo(); }
    private static bool Rest(RobotGripper g) => g.Scara != null ? !g.Scara.IsTrajectoryMoving && !g.Scara.Axis1.IsMoving && !g.Scara.Axis2.IsMoving && !g.Scara.Axis3.IsMoving && !g.Scara.Axis4.IsMoving
        : !g.Fanuc.IsTcpMoving && g.Fanuc.Joints.All(j => !j.IsMoving);
    private static async Task Until(Func<bool> condition)
    { var watch = Stopwatch.StartNew(); while (!condition()) { Require(watch.Elapsed.TotalSeconds < 15, "Motion wait timed out."); await Task.Delay(1); } }

    private static void CheckFit(RobotGripper scara, RobotGripper fanuc, Transform product)
    {
        product.SetPositionAndRotation(Station, Quaternion.identity);
        var shape = product.GetComponent<BoxCollider>();
        var box = BoxInFrame(product, shape.center, shape.size, fanuc.Tcp);
        var left = MeshBounds(fanuc.Tcp.GetComponentsInChildren<Transform>().Single(t => t.name == "LeftFinger"), fanuc.Tcp);
        var right = MeshBounds(fanuc.Tcp.GetComponentsInChildren<Transform>().Single(t => t.name == "RightFinger"), fanuc.Tcp);
        var oldShape = AssetDatabase.LoadAssetAtPath<GameObject>(Protected[4]).GetComponent<BoxCollider>();
        var oldBox = BoxInFrame(product, oldShape.center, oldShape.size, fanuc.Tcp);
        Require(oldBox.size.y > left.min.y - right.max.y, "Legacy part should exceed this side-grasp opening.");
        Require(box.min.y >= right.max.y - .00001f && box.max.y <= left.min.y + .00001f, "Product exceeds open finger gap.");
        Require(Mathf.Abs((left.min.y - right.max.y - .005f) - box.size.y) < .00001f, "Closed finger gap does not match product thickness.");
        Require(box.min.x >= Mathf.Max(left.min.x, right.min.x) && box.max.x <= Mathf.Min(left.max.x, right.max.x) &&
            box.min.z >= Mathf.Max(left.min.z, right.min.z) && box.max.z <= Mathf.Min(left.max.z, right.max.z), "Product outside finger pad bounds.");
        var palm = MeshBounds(fanuc.Tcp.GetComponentsInChildren<MeshFilter>().Single(m => m.name == "SchunkEGH80Gripper").transform, fanuc.Tcp, true);
        Require(box.min.z > palm.max.z, "Product overlaps palm envelope.");
        var sbox = BoxInFrame(product, shape.center, shape.size, scara.Tcp);
        var scaraMesh = MeshBounds(scara.Tcp.parent, scara.Tcp);
        Require(sbox.max.y <= scaraMesh.min.y + .00001f && Mathf.Abs(sbox.max.y - scaraMesh.min.y) < .00001f, "SCARA end-contact plane mismatch.");
        Write("grasp-fit-verification.json", new { passed = true, fanucOpenGapMetres = left.min.y - right.max.y,
            fanucClosedGapMetres = left.min.y - right.max.y - .005f, fanucProductProjectedSizeMetres = V(box.size),
            legacyPartSideGraspWidthMetres = oldBox.size.y,
            fanucPalmClearanceMetres = box.min.z - palm.max.z, scaraProductTopInTcpMetres = sbox.max.y, scaraMeshBottomInTcpMetres = scaraMesh.min.y,
            scope = "Static mesh bounding envelopes only; SCARA has a fixed visual gripper, no articulated fingers or contact-force simulation." });
    }

    private static Bounds MeshBounds(Transform root, Transform frame, bool rootOnly = false)
    {
        var meshes = rootOnly ? root.GetComponents<MeshFilter>() : root.GetComponentsInChildren<MeshFilter>();
        Require(meshes.Length > 0, "Missing mesh bounds.");
        var b = BoxInFrame(meshes[0].transform, meshes[0].sharedMesh.bounds.center, meshes[0].sharedMesh.bounds.size, frame);
        foreach (var mesh in meshes.Skip(1)) b.Encapsulate(BoxInFrame(mesh.transform, mesh.sharedMesh.bounds.center, mesh.sharedMesh.bounds.size, frame));
        return b;
    }

    private static Bounds BoxInFrame(Transform source, Vector3 center, Vector3 size, Transform frame)
    {
        var result = new Bounds(frame.InverseTransformPoint(source.TransformPoint(center)), Vector3.zero);
        for (int i = 0; i < 8; i++) result.Encapsulate(frame.InverseTransformPoint(source.TransformPoint(center + Vector3.Scale(size * .5f,
            new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1)))));
        return result;
    }

    private static void Place(Transform root, RobotGripper g, Pose tcp, Transform itemPoint, Quaternion rotation)
    {
        var contact = tcp.position + tcp.rotation * g.Tcp.InverseTransformPoint(g.Contact.position);
        var frameOffset = root.InverseTransformPoint(g.RobotFrame.position);
        Require(RobotGripper.RotationError(Quaternion.Inverse(root.rotation) * g.RobotFrame.rotation, Quaternion.identity) < .005f, "Unexpected frame rotation.");
        root.SetPositionAndRotation(Station + itemPoint.localPosition - rotation * (frameOffset + contact), rotation);
    }

    private static void CheckPartPose(RobotGripper g, Transform point)
    {
        var rotation = g.Contact.rotation * Quaternion.Inverse(point.localRotation);
        Require(Vector3.Distance(g.Contact.position - rotation * point.localPosition, Station) < .000101f &&
            RobotGripper.RotationError(rotation, Quaternion.identity) < .051f, "Both robots must infer one unchanged product pose.");
    }

    private static void CheckModel(RobotGripper g, Pose expected, ref float position, ref float angle)
    {
        float p = Vector3.Distance(g.RobotFrame.InverseTransformPoint(g.Tcp.position), expected.position);
        float r = RobotGripper.RotationError(Quaternion.Inverse(g.RobotFrame.rotation) * g.Tcp.rotation, expected.rotation);
        Require(p < .000101f && r < .051f, "Model/TCP mismatch."); position = Mathf.Max(position, p); angle = Mathf.Max(angle, r);
    }

    private static Pose ScaraWork() { var set = AssetDatabase.LoadAssetAtPath<RobotWaypointSet>(Protected[5]); return new Pose(set.Work.PositionMetres, set.Work.Rotation); }
    private static Pose FanucWork(FanucJointLab f) { var p = FanucForwardKinematics.Calculate(FanucWorkJoints, f.Geometry); return new Pose(p.PositionMetres, p.Rotation); }
    private static float[][] Seeds(float[] current) => new[] { current, new float[6], new[] { 10f, -15f, 25f, 15f, 20f, -10f }, new[] { -10f, 15f, -25f, -15f, -20f, 10f } };
    private static SIK.JointValues J(float a, float b, float c, float d) => new SIK.JointValues(a, b, c, d);
    private static Dictionary<string, string> Hashes() => Protected.Concat(new[] { PartPath }).ToDictionary(p => p, Hash);
    private static bool Unchanged(Dictionary<string, string> hashes) => hashes.All(p => Hash(p.Key) == p.Value);
    private static string Hash(string p) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(p))); }
    private static float[] V(Vector3 p) => new[] { p.x, p.y, p.z };
    private static void Write(string name, object report) { Directory.CreateDirectory("Logs/LW09"); File.WriteAllText("Logs/LW09/" + name, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
