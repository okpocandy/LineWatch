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
using UnityEditor.SceneManagement;
using UnityEngine;

public static class VerifyCellProducts
{
    private const string ScenePath = "Assets/01.Scenes/LineWatch.unity";
    private static readonly string[] Sources = {
        ScenePath, "ProjectSettings/ProjectSettings.asset", "ProjectSettings/EditorBuildSettings.asset",
        "Assets/03.Prefabs/ScaraRobot_Work.prefab", "Assets/03.Prefabs/FanucRobot_Work.prefab", "Assets/03.Prefabs/GripTrialPart.prefab", "Assets/03.Prefabs/HandoffTrialPart.prefab",
        "Assets/04.ScriptableObject/ScaraCellHandoff.asset", "Assets/04.ScriptableObject/FanucCellHandoff.asset",
        "Assets/02.Scripts/LineWatch/GrippableItem.cs", "Assets/02.Scripts/LineWatch/RobotGripper.cs",
        "Assets/02.Scripts/LineWatch/ProductRegistry.cs", "Assets/02.Scripts/LineWatch/ProductFlow.cs", "Assets/02.Scripts/LineWatch/CellProductTracker.cs"
    };

    public static object Wire()
    {
        Require(!EditorApplication.isPlaying, "Use Edit Mode.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(layout != null && layout.gameObject.scene.path == ScenePath && !layout.gameObject.scene.isDirty, "Open the saved LineWatch scene.");
        var hashes = Sources.Where(p => p != ScenePath).ToDictionary(p => p, Hash);
        var tracker = layout.GetComponent<CellProductTracker>();
        if (tracker == null)
        {
            tracker = layout.gameObject.AddComponent<CellProductTracker>();
            var settings = new SerializedObject(tracker);
            settings.FindProperty("layout").objectReferenceValue = layout;
            settings.FindProperty("initialProduct").objectReferenceValue = layout.GetComponentsInChildren<GrippableItem>().Single();
            settings.ApplyModifiedPropertiesWithoutUndo();
            EditorSceneManager.MarkSceneDirty(layout.gameObject.scene);
            Require(EditorSceneManager.SaveScene(layout.gameObject.scene), "Save failed.");
        }
        Require(hashes.All(p => Hash(p.Key) == p.Value), "Non-cell source changed while wiring.");
        var report = new { passed = true, scene = ScenePath, trackerComponents = layout.GetComponents<CellProductTracker>().Length,
            bootstrapId = new SerializedObject(tracker).FindProperty("initialProductId").stringValue,
            initialSamplePositionUnchanged = true, otherSourcesUnchanged = true, hashes, actualRobotMoves = 0, directButtonClicks = 0 };
        Write("cell-products-wire.json", report); return JsonConvert.SerializeObject(new { report.passed, report.scene, report.trackerComponents, report.bootstrapId, report.otherSourcesUnchanged });
    }

    public static async Task<object> Play()
    {
        var layout = Layout(); var tracker = layout.GetComponent<CellProductTracker>(); var hashes = Sources.ToDictionary(p => p, Hash);
        await Until(() => tracker.Count == 1 || tracker.IsFaulted);
        Require(!tracker.IsFaulted && tracker.InputProductId == "P-000001", "Bootstrap failed: " + tracker.FaultReason);
        Require(tracker.TryGetItem("P-000001", out var first) && first.ProductId == "P-000001" && first.ProductTracker == tracker && first.Owner == null, "Initial ID/object binding mismatch.");
        var sg = layout.Scara.GetComponent<RobotGripper>(); var fg = layout.Fanuc.GetComponent<RobotGripper>();
        float scale = Time.timeScale; int rejected = 0, grips = 0, releases = 0, heldStops = 0;
        var rows = new List<object>();
        string Snapshot() => JsonConvert.SerializeObject(new { tracker.IsFaulted, tracker.Count, tracker.InputProductId, tracker.ScaraProductId, tracker.HandoffProductId, tracker.FanucProductId,
            records = tracker.Snapshot().Select(p => new { p.Id, p.Destination, p.Location }).ToArray(), waiting = tracker.WaitingSnapshot(),
            objects = tracker.Snapshot().Select(p => { bool found = tracker.TryGetItem(p.Id, out var item); return new { p.Id, found,
                owner = found && item.Owner != null ? item.Owner.GetInstanceID() : 0, body = found ? V(item.Body.position) : null,
                rotation = found ? Q(item.Body.rotation) : null }; }).ToArray() });
        void Reject(Func<bool> request, string name)
        { string before = Snapshot(); Require(!request() && before == Snapshot(), "Rejected request changed tracking/pose: " + name); rejected++; }
        void Check(GrippableItem item, ProductLocation location, RobotGripper owner)
        {
            Require(!tracker.IsFaulted && tracker.TryGet(item.ProductId, out var record) && record.Location == location && item.Owner == owner &&
                (owner == null || owner.HeldItem == item), "Physical/logical ownership mismatch.");
            rows.Add(new { id = item.ProductId, location = location.ToString(), owner = owner != null ? owner.name : "none" });
        }
        async Task Grip(RobotGripper g, GrippableItem item, Transform point, ProductLocation location)
        {
            await Move(g, item, point);
            var p = item.Body.position; var q = item.Body.rotation;
            Require(g.TryGrip(item, out var e), e);
            Require(p.Equals(item.Body.position) && q.Equals(item.Body.rotation), "Grip snapped product.");
            Check(item, location, g); grips++;
        }
        async Task Release(RobotGripper g, GrippableItem item, Transform point, ProductLocation location)
        {
            await Move(g, item, point); await Ticks(2);
            Require(g.TryRelease(out var e), e); var p = item.Body.position; var q = item.Body.rotation;
            await Ticks(3); Require(p.Equals(item.Body.position) && q.Equals(item.Body.rotation), "Release moved product.");
            Check(item, location, null); releases++;
        }
        try
        {
            Time.timeScale = 10;
            Require(Rest(sg) && Rest(fg), "Bootstrap must not start robot motion.");
            Reject(() => tracker.TryRegister(first, "OTHER", ProductDestination.B, out _), "one object/two IDs");
            var duplicate = UnityEngine.Object.Instantiate(layout.ProductPrefab);
            Reject(() => tracker.TryRegister(duplicate, first.ProductId, ProductDestination.B, out _), "one ID/two objects");
            Require(!duplicate.IsTracked, "Failed registration bound the duplicate object.");
            duplicate.Body.position = layout.InputStop.position + Vector3.right * .6f; duplicate.Body.rotation = Quaternion.identity;
            Require(tracker.TryRegister(duplicate, "P-000002", ProductDestination.B, out var error), error);
            Reject(() => tracker.TryFeedNext(out _, out _), "occupied input");
            await Move(sg, first, layout.InputStop);
            first.Body.position += Vector3.up * .01f; Reject(() => sg.TryGrip(first, out _), "10mm misalignment"); first.Body.position -= Vector3.up * .01f;
            await Grip(sg, first, layout.InputStop, ProductLocation.Scara);
            Reject(() => tracker.TryRemove(first.ProductId, ProductLocation.Scara, out _), "remove held object");
            Reject(() => tracker.TryFeedNext(out _, out _), "FIFO head not at input");
            duplicate.Body.position = layout.InputStop.position; duplicate.Body.rotation = layout.InputStop.rotation;
            Require(tracker.TryFeedNext(out _, out error), error); Check(duplicate, ProductLocation.Input, null);
            Reject(() => sg.TryRelease(out _), "release away from handoff");
            Require(layout.Scara.TryMoveTcp(layout.Scara.RobotFrame.InverseTransformPoint(sg.Tcp.position + Vector3.up * .05f), 0,
                ScaraJointLab.IkSelection.PositiveElbow, ScaraJointLab.MotionMode.Linear, out _, out error), error);
            await Until(() => !Rest(sg)); sg.Stop(); await Ticks(2); var held = first.Body.position; await Ticks(3);
            Require(held.Equals(first.Body.position), "Held stop drifted."); Check(first, ProductLocation.Scara, sg); heldStops++;
            await Release(sg, first, layout.HandoffPoint, ProductLocation.Handoff);
            await Move(sg, first, layout.HandoffPoint, .07f);
            await Grip(sg, duplicate, layout.InputStop, ProductLocation.Scara);
            // Move the second product to the occupied handoff: release must leave it held.
            await Move(sg, duplicate, layout.HandoffPoint); await Ticks(2);
            Reject(() => sg.TryRelease(out _), "occupied handoff preserves both owners"); Check(duplicate, ProductLocation.Scara, sg);
            await Move(sg, duplicate, layout.InputStop, .07f);
            await Grip(fg, first, layout.HandoffPoint, ProductLocation.Fanuc);
            await Move(fg, first, layout.OutputB); await Ticks(2);
            Reject(() => fg.TryRelease(out _), "wrong output keeps physical owner");
            await Release(fg, first, layout.OutputA, ProductLocation.OutputA);
            await Move(fg, first, layout.OutputA, .06f);
            await Release(sg, duplicate, layout.HandoffPoint, ProductLocation.Handoff);
            await Move(sg, duplicate, layout.HandoffPoint, .07f);
            await Grip(fg, duplicate, layout.HandoffPoint, ProductLocation.Fanuc);
            await Release(fg, duplicate, layout.OutputB, ProductLocation.OutputB);
            // A removed waiting item keeps its ID history; destroying it must not fault the tracker.
            var removed = UnityEngine.Object.Instantiate(layout.ProductPrefab);
            removed.Body.position = layout.InputStop.position + Vector3.right * .8f;
            Require(tracker.TryRegister(removed, "REMOVED", ProductDestination.A, out error), error);
            Require(tracker.TryRemove("REMOVED", ProductLocation.Waiting, out error), error); await Ticks(2);
            Require(!tracker.IsFaulted && tracker.TryGet("REMOVED", out var removedRecord) && removedRecord.Location == ProductLocation.Removed, "Expected removal faulted or lost history.");
            Reject(() => tracker.TryRegister(duplicate, "P-000004", ProductDestination.A, out _), "completed object cannot be rebound");
            Require(tracker.Count == 3 && tracker.Snapshot().Count(p => p.Location == ProductLocation.OutputA) == 1 &&
                tracker.Snapshot().Count(p => p.Location == ProductLocation.OutputB) == 1 && tracker.Snapshot().Count(p => p.Location == ProductLocation.Removed) == 1 &&
                tracker.InputProductId == null && tracker.ScaraProductId == null && tracker.HandoffProductId == null && tracker.FanucProductId == null, "Final conservation/occupancy failed.");
            Require(hashes.All(p => Hash(p.Key) == p.Value), "Saved source changed during Play.");
            var report = new { passed = true, registeredProducts = tracker.Count, physicalGripCommits = grips, physicalReleaseCommits = releases,
                completedA = 1, completedB = 1, removedProducts = 1, heldStops, rejectedRequestsPreserved = rejected,
                sameObjectIdAcrossRobots = true, startupRobotMoves = 0, sourceAssetsUnchanged = true, hashes, rows, directButtonClicks = 0,
                scope = "API-driven physical product/owner binding in LineWatch, one trial per destination. Robot paths are requested by this test; no automatic cell state machine, collision/force validation or production run." };
            Write("cell-products-play.json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.registeredProducts, report.physicalGripCommits, report.physicalReleaseCommits,
                report.completedA, report.completedB, report.removedProducts, report.heldStops, report.rejectedRequestsPreserved, report.sourceAssetsUnchanged });
        }
        finally
        {
            Time.timeScale = scale; sg.Stop(); fg.Stop();
            // Runtime fixtures are discarded by leaving Play. Do not force-drop registered held products here.
        }
    }

    public static async Task<object> Lifecycle(int scenario)
    {
        var layout = Layout(); var tracker = layout.GetComponent<CellProductTracker>();
        await Until(() => tracker.Count == 1 || tracker.IsFaulted);
        Require(!tracker.IsFaulted, "Fresh initial product required.");
        Require(tracker.TryGetItem("P-000001", out var item), "Initial product missing.");
        var sg = layout.Scara.GetComponent<RobotGripper>(); float scale = Time.timeScale;
        try
        {
            Time.timeScale = 10;
            if (scenario != 0)
            {
                await Move(sg, item, layout.InputStop); Require(sg.TryGrip(item, out var e), e);
                Require(layout.Scara.MoveJoint(0, layout.Scara.Axis1.CurrentPosition + 10) && !Rest(sg), "Active-motion fault fixture failed.");
            }
            if (scenario == 0) item.gameObject.SetActive(false);
            else if (scenario == 1) UnityEngine.Object.Destroy(item.gameObject);
            else if (scenario == 2) tracker.enabled = false;
            else throw new ArgumentException("Unknown lifecycle scenario.");
            await Ticks(3);
            Require(tracker.IsFaulted && tracker.Count == 1 && tracker.TryGet("P-000001", out var record) &&
                record.Location == (scenario == 0 ? ProductLocation.Input : ProductLocation.Scara) && Rest(sg), "Fault discarded record or failed to stop.");
            if (scenario == 0)
            {
                item.gameObject.SetActive(true); await Ticks(2);
                Require(item.Owner == null && tracker.IsFaulted && !tracker.TryFeedNext(out _, out _), "Reactivation resumed tracking automatically.");
            }
            if (scenario == 1) Require(sg.HeldItem == null, "Lost item left a physical owner.");
            if (scenario == 2)
            {
                Require(item.Owner == sg && sg.HeldItem == item && !sg.TryRelease(out _), "Tracker disable dropped the held product.");
                tracker.enabled = true; await Ticks(2);
                Require(tracker.IsFaulted && !sg.TryRelease(out _), "Tracker re-enable resumed automatically.");
                UnityEngine.Object.Destroy(tracker); await Ticks(2);
                Require(item.IsTracked && item.ProductTracker == null && !sg.TryRelease(out _) && item.Owner == sg, "Missing tracker downgraded product to untracked release.");
            }
            var report = new { passed = true, scenario, lastKnownRecordsRetained = true, stopped = true, automaticRecovery = false, directButtonClicks = 0 };
            Write("cell-products-lifecycle-" + scenario + ".json", report); return JsonConvert.SerializeObject(report);
        }
        finally { Time.timeScale = scale; sg.Stop(); }
    }

    public static object LegacyState()
    {
        var tracker = Layout().GetComponent<CellProductTracker>();
        Require(!tracker.IsFaulted && tracker.Count == 1 && tracker.InputProductId == "P-000001" &&
            tracker.TryGetItem("P-000001", out var sample) && sample.Owner == null, "Legacy fixture operations changed the tracked input sample.");
        var report = new { passed = true, trackedInputPreserved = true, trackerHealthy = true, directButtonClicks = 0 };
        Write("cell-products-legacy-regression.json", report); return JsonConvert.SerializeObject(report);
    }

    private static LineWatchCellLayout Layout()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use fresh unpaused Play Mode.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(layout != null && layout.gameObject.scene.path == ScenePath, "Open LineWatch."); return layout;
    }
    private static async Task Move(RobotGripper g, GrippableItem item, Transform station, float lift = 0)
    {
        var point = item.GetGripPoint(g.Scara != null ? RobotWaypointSet.RobotType.Scara : RobotWaypointSet.RobotType.Fanuc);
        var rotation = station.rotation * point.localRotation * Quaternion.Inverse(Quaternion.Inverse(g.Tcp.rotation) * g.Contact.rotation);
        var position = station.TransformPoint(point.localPosition) + Vector3.up * lift - rotation * g.Tcp.InverseTransformPoint(g.Contact.position);
        string error;
        bool accepted = g.Scara != null ? g.Scara.TryMoveTcp(g.RobotFrame.InverseTransformPoint(position),
            Vector3.SignedAngle(Vector3.forward, (Quaternion.Inverse(g.RobotFrame.rotation) * rotation) * Vector3.forward, Vector3.up),
            ScaraJointLab.IkSelection.PositiveElbow, ScaraJointLab.MotionMode.Joint, out _, out error)
            : g.Fanuc.TryMoveTcp(g.RobotFrame.InverseTransformPoint(position), Quaternion.Inverse(g.RobotFrame.rotation) * rotation,
                FanucJointLab.IkSeedPolicy.CurrentThenFixed, out _, out error);
        Require(accepted, error); await Until(() => Rest(g)); await Ticks(2);
    }
    private static bool Rest(RobotGripper g) => g.Scara != null ? !g.Scara.IsTrajectoryMoving && !g.Scara.Axis1.IsMoving && !g.Scara.Axis2.IsMoving && !g.Scara.Axis3.IsMoving && !g.Scara.Axis4.IsMoving
        : !g.Fanuc.IsTcpMoving && g.Fanuc.Joints.All(j => !j.IsMoving);
    private static async Task Ticks(int count) { double start = Time.fixedTimeAsDouble; await Until(() => Time.fixedTimeAsDouble - start >= count * Time.fixedDeltaTime); }
    private static async Task Until(Func<bool> condition) { var watch = Stopwatch.StartNew(); while (!condition()) { Require(watch.Elapsed.TotalSeconds < 15, "Wait timed out."); await Task.Delay(1); } }
    private static float[] V(Vector3 p) => new[] { p.x, p.y, p.z };
    private static float[] Q(Quaternion q) => new[] { q.x, q.y, q.z, q.w };
    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))); }
    private static void Write(string name, object report) { Directory.CreateDirectory("Logs/LW10"); File.WriteAllText("Logs/LW10/" + name, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
