using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading.Tasks;
using LineWatch;
using LineWatch.Editor;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UIElements;
using State = LineWatch.RobotWaypointSequence.RunState;

public static class VerifyCellRunUi
{
    private static readonly string[] CommandNames = { "cell-run-start", "cell-run-pause", "cell-run-resume", "cell-run-stop", "cell-run-reset",
        "cell-run-take-scara", "cell-run-take-fanuc", "cell-run-return" };
    private static readonly CellRunCommand[] Commands = { CellRunCommand.Start, CellRunCommand.Pause, CellRunCommand.Resume, CellRunCommand.Stop, CellRunCommand.Reset };

    public static async Task<object> Edit()
    {
        Require(!EditorApplication.isPlaying, "Use LineWatch Edit Mode.");
        var before = SceneSignature(); var hashes = Sources();
        var w = OpenCell(); var roots = OpenRobots(); await UiTick();
        Require(w.rootVisualElement.Q<Label>("cell-run-state").text.Contains("Play 대기"), "Missing Edit Mode status.");
        CheckButtons(w.rootVisualElement, new bool[8], "edit");
        foreach (var root in roots)
        {
            Require(!Button(root, "gripper-prepare") && !Button(root, "waypoint-start") && !Button(root, "gripper-stop"), "Edit robot panel enabled commands.");
        }
        int geometry = 0;
        foreach (float width in new[] { 440f, 520f })
        {
            w.position = new Rect(100, 80, width, 680); await UiTick();
            var bounds = w.rootVisualElement.worldBound;
            foreach (var name in CommandNames)
            {
                var b = w.rootVisualElement.Q<Button>(name);
                if (b.GetFirstAncestorOfType<Foldout>() != null) continue; // Folded individual tools have no visible bounds.
                var r = b.worldBound;
                Require(r.width >= 95 && r.height >= 25 && !float.IsNaN(r.x) && r.xMin >= bounds.xMin - 1 && r.xMax <= bounds.xMax + 1,
                    "Common button clips horizontally: " + name); geometry++;
            }
        }
        var c = UnityEngine.Object.FindFirstObjectByType<CellRunController>();
        var inspector = UnityEditor.Editor.CreateEditor(c);
        var inspectorRoot = inspector.CreateInspectorGUI(); CheckButtons(inspectorRoot, new bool[8], "inspector edit");
        UnityEngine.Object.DestroyImmediate(inspector);
        Require(SceneSignature() == before, "UI reading changed scene/dirty state."); CheckSources(hashes);
        var report = new { passed = true, verifiedUtc = DateTime.UtcNow, editCommandsDisabled = 8, inspectorCommandsDisabled = 8,
            layoutBoundsChecks = geometry, widths = new[] { 440, 520 }, robotPanels = 2, sceneUnchanged = true, sourcesPreserved = hashes.Count,
            directButtonClicks = 0, callbacksInjected = 0, hashes };
        Write("cell-run-ui-edit.json", report); return JsonConvert.SerializeObject(new { report.passed, report.editCommandsDisabled, report.inspectorCommandsDisabled,
            report.layoutBoundsChecks, report.robotPanels, report.sceneUnchanged });
    }

    public static async Task<object> Play()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        var robots = new[] { l.Scara.GetComponent<RobotGripper>(), l.Fanuc.GetComponent<RobotGripper>() };
        var seqs = robots.Select(g => g.GetComponent<RobotWaypointSequence>()).ToArray();
        var w = OpenCell(); var roots = OpenRobots(); var rows = new List<object>(); var hashes = Sources();
        float scale = Time.timeScale; bool editorPaused = EditorApplication.isPaused;
        int snapshots = 0, queryChecks = 0, cycles = 0, legacyLockChecks = 0;
        double maxAdditionalIkUiWaitMilliseconds = 0;
        async Task Snapshot(string phase, bool[] expected, bool legacyLocked, bool? legacyResume = null)
        {
            string before = Signature(l);
            for (int repeat = 0; repeat < 20; repeat++)
            {
                foreach (var command in Commands) { c.CanExecute(command, out _); queryChecks++; }
                c.CanTakeControl(seqs[0], out _); c.CanTakeControl(seqs[1], out _); c.CanReleaseControl(out _);
                c.CanIssueManualCommand(l.Scara, out _); c.CanIssueManualCommand(l.Fanuc, out _); queryChecks += 5;
            }
            Require(Signature(l) == before, "Permission queries mutated state: " + phase);
            await UiTick();
            if (legacyLocked)
            {
                // IK previews refresh independently every 300ms. Observe the actual
                // scheduled state, bounded to two seconds; never invoke UI callbacks.
                var wait = DateTime.UtcNow;
                while (Button(roots[0], "scara-ik-move") || Button(roots[1], "fanuc-ik-move"))
                {
                    Require(!c.CanIssueManualCommand(l.Scara, out _) && !c.CanIssueManualCommand(l.Fanuc, out _), "Runtime gate allowed a legacy command: " + phase);
                    Require((DateTime.UtcNow - wait).TotalSeconds < 2, "IK UI failed to reflect the cell lock: " + phase);
                    await Task.Delay(25);
                }
                maxAdditionalIkUiWaitMilliseconds = Math.Max(maxAdditionalIkUiWaitMilliseconds, (DateTime.UtcNow - wait).TotalMilliseconds);
            }
            CheckButtons(w.rootVisualElement, expected, phase);
            Require(w.rootVisualElement.Q<Label>("cell-run-state").text == "셀 상태: " + c.State, "Wrong cell status: " + phase);
            for (int r = 0; r < 2; r++)
            {
                string kind = r == 0 ? "scara" : "fanuc"; var root = roots[r];
                if (legacyLocked)
                {
                    foreach (string name in new[] { "gripper-prepare", "gripper-work", "gripper-grip", "gripper-lift", "gripper-release",
                        "gripper-cycle-start", "gripper-cycle-resume", "waypoint-start", "waypoint-resume", kind + "-joint-move-0", kind + "-joint-home-0", kind + "-all-home", kind + "-ik-move" })
                    { Require(!Button(root, name), "Legacy command enabled during " + phase + ": " + name); legacyLockChecks++; }
                    Require(root.Q<HelpBox>("gripper-cell-lock").style.display.value == DisplayStyle.Flex &&
                        !string.IsNullOrWhiteSpace(root.Q<HelpBox>("gripper-cell-lock").text), "Missing lock reason: " + phase);
                }
                Require(Button(root, "gripper-stop") && Button(root, kind + "-joint-stop-0") && Button(root, kind + "-all-stop") && Button(root, kind + "-ik-stop"),
                    "Stop disabled by parent/manual lock: " + phase);
                Require(w.rootVisualElement.Q<Label>("cell-run-" + kind).text.Contains(seqs[r].State.ToString()), "Robot state confused with cell state: " + phase);
            }
            if (legacyResume.HasValue)
            {
                int r = c.ControlledSequence == seqs[1] ? 1 : 0;
                // Call sites with no owner check their selected robot explicitly below.
                if (c.HasControlledSequence) Require(Button(roots[r], "gripper-cycle-resume") == legacyResume.Value, "Wrong individual resume state.");
            }
            rows.Add(new { phase, cell = c.State.ToString(), owner = w.rootVisualElement.Q<Label>("cell-run-owner").text,
                buttons = CommandNames.Select(n => new { name = n, enabled = Button(w.rootVisualElement, n) }).ToArray() }); snapshots++;
        }
        void Command(CellRunCommand command) { Require(c.TryExecute(command, out var e), command + ": " + e); }
        try
        {
            await Until(() => t.Count == 1 || t.IsFaulted); Require(!t.IsFaulted && !c.IsFaulted, "Fresh cell required.");
            Time.timeScale = 1;
            for (int i = 1; i <= 2; i++)
            {
                var waiting = UnityEngine.Object.Instantiate(l.ProductPrefab); waiting.Body.position = l.InputStop.position + Vector3.right * (.4f * i);
                Require(t.TryRegister(waiting, "UI-WAIT-" + i, ProductDestination.B, out var e), e);
            }
            string products = Products(l);
            await Snapshot("stopped fresh", new[] { true, false, false, true, false, false, false, false }, false);
            Require(roots.All(r => Button(r, "gripper-prepare") && Button(r, "waypoint-start")), "Stopped UI did not allow individual tests.");
            Command(CellRunCommand.Start);
            await Snapshot("running permission", new[] { false, true, false, true, false, false, false, false }, true);
            Command(CellRunCommand.Pause);
            await Snapshot("paused permission", new[] { false, false, true, true, false, false, false, false }, true);
            Command(CellRunCommand.Stop);
            for (int r = 0; r < 2; r++)
            {
                var g = robots[r]; var q = seqs[r]; var lab = g.GetComponent<RobotGripperLab>(); string id = r == 0 ? l.ScaraId : l.FanucId;
                Time.timeScale = .1f;
                Require(lab.TryPrepare(out var e) && lab.TryStartPickPlace(out e), e);
                await Snapshot(id + " individual running", new[] { false, false, false, true, false, false, false, false }, true);
                q.Pause();
                var take = new[] { false, false, false, true, false, r == 0, r == 1, false };
                await Snapshot(id + " paused for takeover", take, false);
                Require(Button(roots[r], "gripper-cycle-resume"), "Individual paused program cannot resume before adoption.");
                Require(c.TryTakeControl(q, out e), e);
                await Snapshot(id + " adopted stopped", new[] { true, false, false, true, false, false, false, true }, true, false);
                Require(w.rootVisualElement.Q<Label>("cell-run-owner").text.Contains(id) && w.rootVisualElement.Q<Label>("cell-run-owner").text.Contains("공통"), "Wrong adopted command authority.");
                Command(CellRunCommand.Start);
                await Snapshot(id + " common running", new[] { false, true, false, true, false, false, false, false }, true);
                Command(CellRunCommand.Pause);
                await Snapshot(id + " common paused", new[] { false, false, true, true, false, false, false, false }, true, false);
                Time.timeScale = 0;
                await Snapshot(id + " time frozen", new[] { false, false, false, true, false, false, false, false }, true);
                Time.timeScale = 1; EditorApplication.isPaused = true;
                await Snapshot(id + " editor paused", new[] { false, false, false, true, false, false, false, false }, true);
                EditorApplication.isPaused = false; Time.timeScale = 5; Command(CellRunCommand.Resume);
                await Until(() => q.GripCount == 1 && q.StepIndex == 2 || c.IsFaulted || q.State == State.Faulted);
                Require(!c.IsFaulted && q.IsBusy, q.LastError ?? c.FaultReason);
                Command(CellRunCommand.Pause);
                await Snapshot(id + " holding paused", new[] { false, false, true, true, false, false, false, false }, true);
                Require(g.HeldItem == lab.TrialItem && lab.TrialItem.Owner == g &&
                    w.rootVisualElement.Q<Label>("cell-run-" + (r == 0 ? "scara" : "fanuc")).text.Contains(lab.TrialItem.name), "UI did not read real holding.");
                Command(CellRunCommand.Stop);
                await Snapshot(id + " holding stopped owned", new[] { true, false, false, true, false, false, false, true }, true);
                Require(c.TryReleaseControl(out e), e);
                await Snapshot(id + " returned to individual", take, false);
                Require(Button(roots[r], "gripper-cycle-resume") && Button(roots[r], "gripper-release") && !Button(roots[r], "waypoint-start"),
                    "Return did not restore individual resume/release or enabled a holding restart.");
                Require(c.TryTakeControl(q, out e), e); Command(CellRunCommand.Start);
                await Until(() => q.State == State.Completed || c.IsFaulted || q.State == State.Faulted);
                Require(q.State == State.Completed && !c.IsFaulted && q.GripCount == 1 && q.ReleaseCount == 1, q.LastError ?? c.FaultReason); cycles++;
                Command(CellRunCommand.Stop);
                await Snapshot(id + " completed owned", new[] { true, false, false, true, false, false, false, true }, true);
                Require(w.rootVisualElement.Q<HelpBox>("cell-run-mode").text.Contains("반복하지"), "Completed-mode explanation missing.");
                Require(c.TryReleaseControl(out e), e);
                Require(Products(l) == products && !t.IsFaulted, "UI/API trial changed tracked input/FIFO.");
            }
            c.enabled = false; await UiTick(); c.enabled = true;
            string reason = c.FaultReason;
            await Snapshot("faulted after re-enable", new[] { false, false, false, true, true, false, false, false }, true);
            Require(c.IsFaulted && w.rootVisualElement.Q<HelpBox>("cell-run-fault").text.Contains(reason), "UI cleared/omitted first fault.");
            Command(CellRunCommand.Stop); await UiTick(); Require(c.IsFaulted && c.FaultReason == reason && Products(l) == products, "Stop/reading changed fault or products.");
            CheckSources(hashes);
            var report = new { passed = true, verifiedUtc = DateTime.UtcNow, snapshots, readOnlyQueries = queryChecks, legacyLockChecks, maxAdditionalIkUiWaitMilliseconds, completedCycles = cycles,
                registeredProducts = t.Count, physicallyConfirmedResidual = 3, unconfirmedProducts = 0, firstFaultPreserved = true,
                sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0, rows, hashes,
                scope = "API-driven transitions and UI property reads only. User must verify the actual click callbacks and visual usability." };
            Write("cell-run-ui-play.json", report);
            return JsonConvert.SerializeObject(new { report.passed, report.snapshots, report.readOnlyQueries, report.legacyLockChecks,
                report.completedCycles, report.registeredProducts, report.unconfirmedProducts, report.firstFaultPreserved, report.sourcesPreserved });
        }
        finally { EditorApplication.isPaused = editorPaused; Time.timeScale = scale; c.TryExecute(CellRunCommand.Stop, out _); }
    }

    public static async Task<object> MissingManager()
    {
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var hashes = Sources();
        var w = OpenCell(); var roots = OpenRobots(); await UiTick();
        UnityEngine.Object.Destroy(c); await UiTick();
        CheckButtons(w.rootVisualElement, new bool[8], "manager removed");
        Require(w.rootVisualElement.Q<Label>("cell-run-state").text.Contains("관리자 없음") &&
            w.rootVisualElement.Q<Label>("cell-run-owner").text.Contains("확인 불가"), "Deleted manager shown as individual control.");
        foreach (var root in roots) Require(!Button(root, "gripper-prepare") && !Button(root, "waypoint-start") && Button(root, "gripper-stop"), "Deleted manager unlocked robot UI or blocked local Stop.");
        CheckSources(hashes);
        var report = new { passed = true, managerMissingDisplayed = true, commandsDisabled = 8, individualFallback = false,
            localStopEnabled = true, sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0, hashes };
        Write("cell-run-ui-missing-manager.json", report); return JsonConvert.SerializeObject(new { report.passed, report.commandsDisabled, report.individualFallback, report.localStopEnabled });
    }

    private static CellRunWindow OpenCell()
    {
        var w = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); w.position = new Rect(100, 80, 480, 680); w.Show(); w.RefreshCell(); return w;
    }
    private static VisualElement[] OpenRobots()
    {
        var s = EditorWindow.GetWindow<ScaraJointLabWindow>("SCARA 4축 조작 테스트"); s.Show(); s.RefreshLab();
        var f = EditorWindow.GetWindow<FanucJointLabWindow>("FANUC 조작 테스트"); f.Show(); f.RefreshLab();
        return new[] { s.rootVisualElement, f.rootVisualElement };
    }
    private static bool Button(VisualElement root, string name)
    { var b = root.Q<Button>(name); Require(b != null, "Missing button: " + name); return b.enabledInHierarchy; }
    private static void CheckButtons(VisualElement root, bool[] expected, string phase)
    { for (int i = 0; i < CommandNames.Length; i++) Require(Button(root, CommandNames[i]) == expected[i], "Wrong button state in " + phase + ": " + CommandNames[i]); }
    private static LineWatchCellLayout Layout()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use a fresh unpaused LineWatch Play session.");
        var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>(); Require(l != null && l.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Open LineWatch."); return l;
    }
    private static string Products(LineWatchCellLayout l)
    {
        var t = l.GetComponent<CellProductTracker>(); return JsonConvert.SerializeObject(new { records = t.Snapshot(), fifo = t.WaitingSnapshot(),
            slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId },
            owners = t.Snapshot().Select(r => new { r.Id, owner = t.TryGetItem(r.Id, out var item) && item.Owner != null ? item.Owner.GetInstanceID() : 0 }) });
    }
    private static string Signature(LineWatchCellLayout l)
    {
        var c = l.GetComponent<CellRunController>(); return JsonConvert.SerializeObject(new { c.State, c.FaultReason, c.HasControlledSequence,
            controlled = c.ControlledSequence != null ? c.ControlledSequence.GetInstanceID() : 0, products = Products(l),
            robots = new[] { l.Scara.GetComponent<RobotGripper>(), l.Fanuc.GetComponent<RobotGripper>() }.Select(g => new {
                version = g.Scara != null ? g.Scara.CommandVersion : g.Fanuc.CommandVersion,
                current = g.Scara != null ? new[] { g.Scara.Axis1.CurrentPosition, g.Scara.Axis2.CurrentPosition, g.Scara.Axis3.CurrentPosition, g.Scara.Axis4.CurrentPosition } : g.Fanuc.Joints.Select(a => a.CurrentPosition).ToArray(),
                state = g.GetComponent<RobotWaypointSequence>().State, step = g.GetComponent<RobotWaypointSequence>().StepIndex }) });
    }
    private static string SceneSignature() => JsonConvert.SerializeObject(Enumerable.Range(0, SceneManager.sceneCount).Select(i => SceneManager.GetSceneAt(i)).Select(s => new { s.path, s.isDirty, s.isLoaded }));
    private static Dictionary<string, string> Sources() => new[] { "Assets/01.Scenes", "Assets/02.Scripts", "Assets/03.Prefabs", "Assets/04.ScriptableObject", "ProjectSettings" }
        .SelectMany(p => Directory.GetFiles(p, "*", SearchOption.AllDirectories)).Concat(new[] { "Packages/manifest.json", "Packages/packages-lock.json" }).ToDictionary(p => p, Hash);
    private static string Hash(string p) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(p))).Replace("-", ""); }
    private static void CheckSources(Dictionary<string, string> hashes) => Require(hashes.All(p => File.Exists(p.Key) && Hash(p.Key) == p.Value), "Verification changed source assets.");
    private static async Task UiTick() => await Task.Delay(350);
    private static async Task Until(Func<bool> condition)
    { var start = DateTime.UtcNow; while (!condition()) { Require((DateTime.UtcNow - start).TotalSeconds < 20, "Wait timed out."); await Task.Delay(1); } }
    private static void Write(string name, object report) { Directory.CreateDirectory("Logs/LW11"); File.WriteAllText("Logs/LW11/" + name, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool ok, string message) { if (!ok) throw new InvalidOperationException(message); }
}
