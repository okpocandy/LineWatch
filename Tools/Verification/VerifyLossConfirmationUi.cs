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
using UnityEngine.UIElements;

public static class VerifyLossConfirmationUi
{
    public static async Task<string> Edit()
    {
        Require(!EditorApplication.isPlaying, "Use LineWatch Edit Mode.");
        var c = UnityEngine.Object.FindFirstObjectByType<CellRunController>(); Require(c != null, "Open LineWatch.");
        var hashes = Sources(); string scene = SceneSignature(); var w = Open(); int boundsChecks = 0;
        foreach (int width in new[] { 440, 520 })
        {
            w.position = new Rect(100, 80, width, 700); await Task.Delay(250);
            EmptyForm(w.rootVisualElement, false, 1);
            var r = w.rootVisualElement.Q<Foldout>("cell-loss-panel").worldBound; var bounds = w.rootVisualElement.worldBound;
            Require(r.width > 100 && r.xMin >= bounds.xMin - 1 && r.xMax <= bounds.xMax + 1, "Loss foldout header clips horizontally."); boundsChecks++;
        }
        var inspector = UnityEditor.Editor.CreateEditor(c); var root = inspector.CreateInspectorGUI(); EmptyForm(root, false, 1); UnityEngine.Object.DestroyImmediate(inspector);
        Require(SceneSignature() == scene, "UI creation/read changed scene/dirty state."); CheckSources(hashes);
        var report = new { passed = true, editForms = 2, headerBoundsChecks = boundsChecks, defaultCollapsed = true, automaticTargetSelection = false,
            sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0, uiValuesAssignedByVerifier = 0,
            expandedFormLayoutAndActualInputPending = true, hashes };
        Write("loss-ui-edit.json", report); return JsonConvert.SerializeObject(new { report.passed, report.editForms, report.headerBoundsChecks, report.defaultCollapsed, report.automaticTargetSelection });
    }

    public static async Task<string> Play(int scenario)
    {
        Require(scenario == 0 || scenario == 1, "Use 0=destroyed GameObject, 1=component only.");
        var l = Layout(); var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>(); var hashes = Sources();
        await Until(() => t.Count == 1); AddWaiting(l, "WAIT-A", .4f); AddWaiting(l, "WAIT-B", .8f);
        var w = Open(); var root = w.rootVisualElement; EmptyForm(root, true, 4);
        var target = root.Q<DropdownField>("cell-loss-target"); Require(target.choices.Contains("ID P-000001"), "Registered ID missing from choices.");
        Require(t.TryGet("P-000001", out var stale), "Original record missing.");
        Require(t.TryGetItem("P-000001", out var item), "Original item missing.");
        var originalObject = item.gameObject; const string reason = "UI 연동 API 검증: 원본 삭제 확인";
        string before = Full(l); int queries = 0;
        for (int i = 0; i < 100; i++) { Require(!t.CanConfirmDestroyedLoss(stale.Id, stale.Location, reason, out _), "Healthy original should not be loss-eligible."); queries++; EmptyForm(root, true, 4); }
        Require(Full(l) == before, "Reading UI/readiness changed healthy records/commands.");
        if (scenario == 0) UnityEngine.Object.Destroy(originalObject); else UnityEngine.Object.Destroy(item);
        await Until(() => t.IsFaulted && c.IsFaulted && !t.TryGetItem(stale.Id, out _));
        if (scenario == 0) Require(originalObject == null, "Original GameObject must actually be destroyed.");
        else Require(originalObject != null && originalObject.activeInHierarchy, "Component-only fixture must retain the physical GameObject.");
        await Until(() => root.Q<HelpBox>("cell-run-fault").text.Contains(c.FaultReason)); EmptyForm(root, true, 4);
        before = Full(l);
        for (int i = 0; i < 100; i++)
        { Require(t.CanConfirmDestroyedLoss(stale.Id, stale.Location, reason, out _) == (scenario == 0), "Backend deletion proof is wrong."); queries++; EmptyForm(root, true, 4); }
        Require(Full(l) == before, "Candidate refresh/read changed faults, product records or physical holding.");
        int backendConfirmations = 0;
        if (scenario == 0)
        {
            Require(t.TryConfirmDestroyedLoss(stale.Id, stale.Location, reason, out var error), error); backendConfirmations++;
            await Until(() => !target.choices.Contains("ID P-000001") && root.Q<Label>("cell-run-products").text.Contains(reason));
            EmptyForm(root, true, 3);
            Require(target.choices.SequenceEqual(new[] { "대상을 선택하세요", "ID WAIT-A", "ID WAIT-B" }), "Removed target caused automatic selection/reordering.");
            Require(t.TryGet(stale.Id, out var receipt) && receipt.Location == ProductLocation.Lost && receipt.LostFrom == stale.Location && receipt.LossReason == reason &&
                c.IsFaulted && t.IsFaulted && t.Count == 3 && t.WaitingSnapshot().SequenceEqual(new[] { "WAIT-A", "WAIT-B" }), "API receipt/UI refresh cleared faults or changed unrelated records.");
            before = Full(l); Require(!t.TryConfirmDestroyedLoss(stale.Id, stale.Location, "오래된 선택", out _) && Full(l) == before, "Stale captured request bypassed backend validation.");
        }
        else
        {
            Require(!t.TryConfirmDestroyedLoss(stale.Id, stale.Location, reason, out _) && Full(l) == before && t.Snapshot().All(r => r.Location != ProductLocation.Lost),
                "Component-only loss created a receipt or changed state.");
        }
        // We do not select a dropdown option, type a reason, assign a UI value or invoke
        // a button callback. These disabled inputs still update correctly on lifecycles.
        c.enabled = false; await Until(() => !root.Q<DropdownField>("cell-loss-target").enabledInHierarchy); EmptyForm(root, false, 1);
        c.enabled = true; await Until(() => target.enabledInHierarchy && target.choices.Count == (scenario == 0 ? 3 : 4)); EmptyForm(root, true, scenario == 0 ? 3 : 4);
        t.enabled = false; await Until(() => !target.enabledInHierarchy); EmptyForm(root, false, 1);
        t.enabled = true; await Until(() => target.enabledInHierarchy && target.choices.Count == (scenario == 0 ? 3 : 4)); EmptyForm(root, true, scenario == 0 ? 3 : 4);
        Require(c.IsFaulted && t.IsFaulted, "Reactivation implicitly acknowledged a fault.");
        Require(root.Q<HelpBox>("cell-loss-request").style.display.value == DisplayStyle.None, "A UI submission result was invented without clicking.");
        var inspector = UnityEditor.Editor.CreateEditor(c); var inspectorRoot = inspector.CreateInspectorGUI(); EmptyForm(inspectorRoot, true, scenario == 0 ? 3 : 4); UnityEngine.Object.DestroyImmediate(inspector);
        CheckSources(hashes);
        var report = new { passed = true, scenario, backendConfirmations, readOnlyQueries = queries, automaticTargetSelection = false,
            emptyReasonPreserved = true, missingGameObjectEligibilityVerified = scenario == 0, componentOnlyLossBlocked = scenario == 1,
            staleRequestBackendRejectionVerified = scenario == 0, faultsRemainLatched = true, registeredProducts = 3,
            confirmedLost = scenario == 0 ? 1 : 0, physicallyConfirmedResidual = 2, unconfirmedProducts = scenario == 0 ? 0 : 1,
            sourcesPreserved = hashes.Count, directButtonClicks = 0, callbacksInjected = 0, uiValuesAssignedByVerifier = 0,
            actualSelectionTextEntrySuccessfulCallbackAndDraftClearingPending = true, hashes };
        Write("loss-ui-play-" + scenario + ".json", report);
        return JsonConvert.SerializeObject(new { report.passed, report.scenario, report.backendConfirmations, report.readOnlyQueries, report.confirmedLost, report.unconfirmedProducts, report.actualSelectionTextEntrySuccessfulCallbackAndDraftClearingPending });
    }

    private static void EmptyForm(VisualElement root, bool inputsEnabled, int choices)
    {
        var panel = root.Q<Foldout>("cell-loss-panel"); var target = root.Q<DropdownField>("cell-loss-target"); var reason = root.Q<TextField>("cell-loss-reason"); var confirm = root.Q<Button>("cell-loss-confirm");
        Require(panel != null && target != null && reason != null && confirm != null, "Missing loss confirmation controls.");
        Require(!panel.value && target.value == "대상을 선택하세요" && string.IsNullOrEmpty(reason.value) && target.choices.Count == choices,
            "Loss form auto-selected/filled a request or lost its choice list.");
        Require(target.enabledInHierarchy == inputsEnabled && reason.enabledInHierarchy == inputsEnabled && !confirm.enabledInHierarchy,
            "Empty/edit/inactive loss form enabled a confirmation.");
    }
    private static CellRunWindow Open()
    { var w = EditorWindow.GetWindow<CellRunWindow>("셀 공통 조작"); w.Show(); w.RefreshCell(); return w; }
    private static LineWatchCellLayout Layout()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use a fresh unpaused LineWatch Play session.");
        var l = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>(); Require(l != null && l.gameObject.scene.path == "Assets/01.Scenes/LineWatch.unity", "Open LineWatch."); return l;
    }
    private static void AddWaiting(LineWatchCellLayout l, string id, float offset)
    { var item = UnityEngine.Object.Instantiate(l.ProductPrefab); item.Body.position = l.InputStop.position + Vector3.right * offset; Require(l.GetComponent<CellProductTracker>().TryRegister(item, id, ProductDestination.B, out var e), e); }
    private static string Full(LineWatchCellLayout l)
    {
        var c = l.GetComponent<CellRunController>(); var t = l.GetComponent<CellProductTracker>();
        return JsonConvert.SerializeObject(new { c.State, c.FaultReason, c.LastAcknowledgedFaultReason, t.IsFaulted, trackerCause = t.FaultReason, trackerArchive = t.LastAcknowledgedFaultReason,
            records = t.Snapshot(), fifo = t.WaitingSnapshot(), slots = new[] { t.InputProductId, t.ScaraProductId, t.HandoffProductId, t.FanucProductId },
            robots = new[] { l.Scara.GetComponent<RobotGripper>(), l.Fanuc.GetComponent<RobotGripper>() }.Select(g => {
                var q = g.GetComponent<RobotWaypointSequence>(); return new { q.State, q.StepIndex, q.GripCount, q.ReleaseCount, q.LastError,
                    version = g.Scara != null ? g.Scara.CommandVersion : g.Fanuc.CommandVersion,
                    current = g.Scara != null ? new[] { g.Scara.Axis1.CurrentPosition, g.Scara.Axis2.CurrentPosition, g.Scara.Axis3.CurrentPosition, g.Scara.Axis4.CurrentPosition } : g.Fanuc.Joints.Select(a => a.CurrentPosition).ToArray(),
                    targets = g.Scara != null ? new[] { g.Scara.Axis1.TargetPosition, g.Scara.Axis2.TargetPosition, g.Scara.Axis3.TargetPosition, g.Scara.Axis4.TargetPosition } : g.Fanuc.Joints.Select(a => a.TargetPosition).ToArray(), held = g.HeldItem != null ? g.HeldItem.GetInstanceID() : 0 }; }) });
    }
    private static string SceneSignature() => JsonConvert.SerializeObject(new { path = UnityEngine.SceneManagement.SceneManager.GetActiveScene().path, dirty = UnityEngine.SceneManagement.SceneManager.GetActiveScene().isDirty });
    private static Dictionary<string, string> Sources() => new[] { "Assets/01.Scenes", "Assets/02.Scripts", "Assets/03.Prefabs", "Assets/04.ScriptableObject", "ProjectSettings" }
        .SelectMany(p => Directory.GetFiles(p, "*", SearchOption.AllDirectories)).Concat(new[] { "Packages/manifest.json", "Packages/packages-lock.json" }).ToDictionary(p => p, Hash);
    private static string Hash(string p) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(p))).Replace("-", ""); }
    private static void CheckSources(Dictionary<string, string> hashes) => Require(hashes.All(p => File.Exists(p.Key) && Hash(p.Key) == p.Value), "UI verification changed code/scene/assets.");
    private static async Task Until(Func<bool> condition) { var start = DateTime.UtcNow; while (!condition()) { Require((DateTime.UtcNow - start).TotalSeconds < 5, "UI/state wait timed out."); await Task.Delay(10); } }
    private static void Write(string name, object report) { Directory.CreateDirectory("Logs/LW11"); File.WriteAllText("Logs/LW11/" + name, JsonConvert.SerializeObject(report, Formatting.Indented)); }
    private static void Require(bool ok, string message) { if (!ok) throw new InvalidOperationException(message); }
}
