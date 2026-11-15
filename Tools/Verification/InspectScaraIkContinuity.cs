using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading;
using System.Threading.Tasks;
using LineWatch;
using LineWatch.Editor;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

// Historical LW-07 threshold-policy probes. Verify/Moving retain that retired contract.
// Current Joint/Linear acceptance checks are in VerifyScaraMotionPlanning.
public static class InspectScaraIkContinuity
{
    public static Task<object> Run() => AuditPaths(false);
    public static Task<object> Verify() => AuditPaths(true);

    private static async Task<object> AuditPaths(bool verifyGuard)
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Unpaused RobotLab Play Mode required");
        var lab = UnityEngine.Object.FindFirstObjectByType<ScaraJointLab>();
        Require(lab != null && lab.IsReady, "SCARA references required");
        string[] paths = { "Assets/01.Scenes/RobotLab.unity", "Assets/03.Prefabs/ScaraRobot_Work.prefab", "ProjectSettings/ProjectSettings.asset" };
        var hashes = paths.ToDictionary(path => path, Hash);
        var settings = new SerializedObject(lab);
        var geometry = new ScaraForwardKinematics.Geometry(
            settings.FindProperty("fkLink1Metres").floatValue,settings.FindProperty("fkLink2Metres").floatValue,
            settings.FindProperty("fkHeight12Metres").floatValue,settings.FindProperty("fkHeight23Metres").floatValue,
            settings.FindProperty("fkToolOffsetMetres").vector3Value,settings.FindProperty("fkHomeToolYawDegrees").floatValue);
        float originalScale = Time.timeScale;
        var token = lab.destroyCancellationToken;
        var groups = new List<object>();
        int queries = 0, riskyPaths = 0, rejected = 0;
        float maxPositionResidual = 0f, maxYawResidual = 0f;

        async Task Audit(string name, Vector4 baseline, Vector4[] sourcePoses,bool expectRejection = false)
        {
            Require(lab.MoveJoint(0,baseline.x) && lab.MoveJoint(1,baseline.y) &&
                lab.MoveJoint(2,baseline.z) && lab.MoveJoint(3,baseline.w), "Baseline outside joint limits: " + name);
            await Arrive(lab,token);
            var commands = Targets(lab);
            var current = Current(lab);
            var previous = current;
            int previousSign = baseline.y > 0f ? 1 : baseline.y < 0f ? -1 : 0;
            var previousGoal = ScaraForwardKinematics.Calculate(baseline.x,baseline.y,baseline.z,baseline.w,geometry);
            float maxStep = 0f, maxTargetStep = 0f, maxYawStep = 0f;
            int branchChanges = 0, largeSteps = 0, rejectedInPath = 0;
            var rows = new List<object>();
            foreach (var p in sourcePoses)
            {
                // Some nominal source joints exceed limits to construct a nearby Cartesian
                // target whose other IK branch is still reachable. These joints are not driven.
                var goal = ScaraForwardKinematics.Calculate(p.x,p.y,p.z,p.w,geometry);
                bool valid = lab.TrySelectTcpSolution(goal.PositionMetres,goal.YawDegrees,ScaraJointLab.IkSelection.Nearest,
                    out var result,out var selected,out var error);
                queries++;
                Require(commands.Equals(Targets(lab)) && current.Equals(Current(lab)), "Preview mutated joint state");
                if (!valid)
                {
                    rejected++;
                    rejectedInPath++;
                    rows.Add(new { sourceJoints = Values(p), accepted = false, error });
                    continue;
                }
                var j = selected.Joints;
                var values = new Vector4(j.Q1Degrees,j.Q2Degrees,j.D3Metres,j.Q4Degrees);
                var fk = ScaraForwardKinematics.Calculate(values.x,values.y,values.z,values.w,geometry);
                float ep = Vector3.Distance(goal.PositionMetres,fk.PositionMetres);
                float ey = Mathf.Abs(Mathf.DeltaAngle(goal.YawDegrees,fk.YawDegrees));
                Require(ep <= .000002f && ey <= .001f, "Selected solution failed FK residual");
                maxPositionResidual = Mathf.Max(maxPositionResidual,ep);
                maxYawResidual = Mathf.Max(maxYawResidual,ey);
                float step = Mathf.Max(Mathf.Abs(values.x-previous.x),Mathf.Abs(values.y-previous.y),Mathf.Abs(values.w-previous.w));
                float targetStep = Vector3.Distance(goal.PositionMetres,previousGoal.PositionMetres);
                float yawStep = Mathf.Abs(Mathf.DeltaAngle(goal.YawDegrees,previousGoal.YawDegrees));
                bool switched = previousSign != 0 && selected.ElbowSign != 0 && previousSign != selected.ElbowSign &&
                    Mathf.Abs(previous.y) > 1f && Mathf.Abs(values.y) > 1f;
                if (switched) branchChanges++;
                if (step > 15f) largeSteps++;
                maxStep = Mathf.Max(maxStep,step); maxTargetStep = Mathf.Max(maxTargetStep,targetStep); maxYawStep = Mathf.Max(maxYawStep,yawStep);
                rows.Add(new { sourceJoints = Values(p), accepted = true, goalMetres = Values(goal.PositionMetres), goal.YawDegrees,
                    selectedJoints = Values(values), selected.ElbowSign, selected.IsSingular, candidates = result.Solutions.Count,
                    jointStepDegrees = step, targetStepMetres = targetStep, yawStepDegrees = yawStep, switched });
                previous = values;
                previousSign = selected.ElbowSign;
                previousGoal = goal;
            }
            bool risk = branchChanges > 0 || largeSteps > 0;
            if (risk) riskyPaths++;
            if (verifyGuard) Require(!risk && (expectRejection ? rejectedInPath > 0 && rejectedInPath < sourcePoses.Length : rejectedInPath == 0), "Continuity guard regression: " + name);
            groups.Add(new { name, baseline = Values(baseline), samples = sourcePoses.Length, continuityRisk = risk,
                maxAdjacentJointStepDegrees = maxStep, maxAdjacentTargetStepMetres = maxTargetStep,
                maxAdjacentYawStepDegrees = maxYawStep, branchChanges, stepsAbove15Degrees = largeSteps, rejectedSamples = rejectedInPath, rows });
        }

        try
        {
            Time.timeScale = 10f;
            await Audit("Ordinary small target changes",new Vector4(20f,60f,.05f,-80f),
                Enumerable.Range(0,41).Select(i => new Vector4(20f+i*.01f,60f-i*.01f,.05f+i*.000001f,-80f)).ToArray());
            await Audit("Positive elbow to full extension",new Vector4(0f,2f,.05f,-2f),
                Enumerable.Range(0,41).Select(i => new Vector4(0f,2f-i*.05f,.05f,-2f+i*.05f)).ToArray());
            await Audit("Negative elbow to full extension",new Vector4(0f,-2f,.05f,2f),
                Enumerable.Range(0,41).Select(i => new Vector4(0f,-2f+i*.05f,.05f,2f-i*.05f)).ToArray());
            await Audit("Yaw representation crosses 180",new Vector4(20f,60f,.05f,99.8f),
                Enumerable.Range(0,21).Select(i => new Vector4(20f,60f,.05f,99.8f+i*.02f)).ToArray());
            await Audit("Physical wrist limit crossing",new Vector4(0f,90f,.05f,179.9f),
                Enumerable.Range(0,21).Select(i => new Vector4(0f,90f,.05f,179.9f+i*.01f)).ToArray(),true);
            await Audit("Physical shoulder limit crossing",new Vector4(169.9f,-60f,.05f,-109.9f),
                Enumerable.Range(0,21).Select(i => new Vector4(169.9f+i*.01f,-60f,.05f,-109.9f-i*.01f)).ToArray(),true);
            lab.HomeAll();
            await Arrive(lab,token);
            bool unchanged = paths.All(path => hashes[path] == Hash(path));
            Require(unchanged, "Source assets changed");
            var report = new { auditCompleted = true, continuitySatisfied = riskyPaths == 0, utc = DateTime.UtcNow.ToString("O"),
                previewQueries = queries, pathCount = groups.Count, riskyPaths, rejectedQueries = rejected,
                maxFkPositionResidualMetres = maxPositionResidual, maxFkYawResidualDegrees = maxYawResidual,
                diagnosticLargeStepDegrees = 15f, sourceAssetsUnchanged = unchanged, sourceHashes = hashes,
                directButtonClicks = 0, riskyTransitionsExecuted = 0, verifyGuard,
                scope = "Baseline poses driven at 10x time; subsequent targets queried only, no preview target commands. Diagnostic threshold is not a control or safety policy.", groups };
            Directory.CreateDirectory("Logs/LW07");
            string reportPath = verifyGuard ? "Logs/LW07/continuity-guard-verification.json"
                : "Logs/LW07/continuity-audit-" + DateTime.UtcNow.ToString("yyyyMMdd-HHmmss-fffffff") + ".json";
            File.WriteAllText(reportPath,JsonConvert.SerializeObject(report,Formatting.Indented));
            return new { report.auditCompleted, report.continuitySatisfied, report.previewQueries, report.pathCount,
                report.riskyPaths, report.rejectedQueries, report.maxFkPositionResidualMetres, report.maxFkYawResidualDegrees,
                report.riskyTransitionsExecuted, report.sourceAssetsUnchanged };
        }
        finally
        {
            Time.timeScale = originalScale;
            if (lab != null) lab.StopAll();
        }
    }

    public static async Task<object> Moving()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Unpaused Play Mode required");
        var lab = UnityEngine.Object.FindFirstObjectByType<ScaraJointLab>();
        Require(lab != null && lab.IsReady, "SCARA references required");
        var token = lab.destroyCancellationToken;
        float scale = Time.timeScale;
        var geometry = new ScaraForwardKinematics.Geometry(.325f,.375f,.016783f,-.097783f,new Vector3(0f,-.034177f,0f));
        string[] paths = { "Assets/01.Scenes/RobotLab.unity", "Assets/03.Prefabs/ScaraRobot_Work.prefab", "ProjectSettings/ProjectSettings.asset" };
        var hashes = paths.ToDictionary(path => path,Hash);
        int refused = 0, explicitSelections = 0;
        var shoulder = new Vector4(169.9f,-60f,.05f,-109.9f);
        var shoulderNext = ScaraForwardKinematics.Calculate(170.05f,-60f,.05f,-110.05f,geometry);
        var wristNext = ScaraForwardKinematics.Calculate(0f,90f,.05f,180.05f,geometry);
        var rows = new List<object>();

        async Task SetPose(Vector4 j)
        {
            Require(lab.MoveJoint(0,j.x) && lab.MoveJoint(1,j.y) && lab.MoveJoint(2,j.z) && lab.MoveJoint(3,j.w), "Baseline setup failed");
            await Arrive(lab,token);
        }
        void Refuse(string name,Vector3 target,float yaw)
        {
            var targets = Targets(lab); var current = Current(lab);
            bool[] moving = { lab.Axis1.IsMoving,lab.Axis2.IsMoving,lab.Axis3.IsMoving,lab.Axis4.IsMoving };
            Require(!lab.TryMoveTcp(target,yaw,ScaraJointLab.IkSelection.Nearest,out _,out var error) && !string.IsNullOrEmpty(error), name+": accepted");
            Require(targets.Equals(Targets(lab)) && current.Equals(Current(lab)) && moving.SequenceEqual(new[] { lab.Axis1.IsMoving,lab.Axis2.IsMoving,lab.Axis3.IsMoving,lab.Axis4.IsMoving }), name+": command changed");
            refused++;
            rows.Add(new { name, error, passed = true });
        }
        void Explicit(string name,ScaraForwardKinematics.TcpPose goal,ScaraJointLab.IkSelection mode)
        {
            var before = Targets(lab);
            Require(lab.TrySelectTcpSolution(goal.PositionMetres,goal.YawDegrees,mode,out _,out _,out _), name+": explicit selection rejected");
            Require(before.Equals(Targets(lab)), name+": preview moved axes");
            explicitSelections++;
        }

        try
        {
            Time.timeScale = 10f;
            await SetPose(shoulder);
            Require(!lab.TrySelectTcpSolution(shoulderNext.PositionMetres,shoulderNext.YawDegrees,ScaraJointLab.IkSelection.Nearest,out var reachable,out _,out _) && reachable.Success, "Policy must distinguish valid IK from disallowed automatic transition");
            Refuse("Shoulder branch flip",shoulderNext.PositionMetres,shoulderNext.YawDegrees);
            Explicit("Shoulder explicit positive branch",shoulderNext,ScaraJointLab.IkSelection.PositiveElbow);

            var window = EditorWindow.GetWindow<ScaraJointLabWindow>();
            window.RefreshLab();
            var ui = window.rootVisualElement;
            ui.Q<FloatField>("scara-ik-x").value = shoulderNext.PositionMetres.x*1000f;
            ui.Q<FloatField>("scara-ik-y").value = shoulderNext.PositionMetres.y*1000f;
            ui.Q<FloatField>("scara-ik-z").value = shoulderNext.PositionMetres.z*1000f;
            ui.Q<FloatField>("scara-ik-yaw").value = shoulderNext.YawDegrees;
            await Task.Delay(200,token);
            Require(!ui.Q<Button>("scara-ik-move").enabledInHierarchy && ui.Q<HelpBox>("scara-ik-feedback").text.Contains("큰 관절") &&
                ui.Q<Label>("scara-ik-selected").text == "선택 예정: 없음", "Continuity refusal not visible in existing UI");
            ui.Q<DropdownField>("scara-ik-selection").value = "q2 양수 해";
            await Task.Delay(120,token);
            Require(ui.Q<Button>("scara-ik-move").enabledInHierarchy, "Explicit UI selection locked");
            window.RefreshLab();

            await SetPose(new Vector4(0f,90f,.05f,179.9f));
            Refuse("Wrist long transition",wristNext.PositionMetres,wristNext.YawDegrees);
            Explicit("Wrist explicit negative branch",wristNext,ScaraJointLab.IkSelection.NegativeElbow);

            await SetPose(Vector4.zero);
            var farGoal = ScaraForwardKinematics.Calculate(shoulder.x,shoulder.y,shoulder.z,shoulder.w,geometry);
            Require(lab.TryMoveTcp(farGoal.PositionMetres,farGoal.YawDegrees,ScaraJointLab.IkSelection.NegativeElbow,out _,out _), "Far deliberate move rejected");
            await Task.Delay(25,token);
            Require(lab.Axis1.IsMoving && lab.Axis1.CurrentPosition > 0f && lab.Axis1.CurrentPosition < 100f, "In-flight reference not exercised");
            Refuse("Invalid request preserves in-flight anchor",new Vector3(.8f,-.2f,0f),0f);
            Refuse("Nearby replacement preserves in-flight branch",shoulderNext.PositionMetres,shoulderNext.YawDegrees);
            lab.StopAll();
            Require(lab.TrySelectTcpSolution(shoulderNext.PositionMetres,shoulderNext.YawDegrees,ScaraJointLab.IkSelection.Nearest,out _,out _,out _), "Stop did not reset the old command anchor");

            await SetPose(Vector4.zero);
            var ordinary = ScaraForwardKinematics.Calculate(20f,60f,.05f,-80f,geometry);
            var nearby = ScaraForwardKinematics.Calculate(20.05f,59.95f,.050001f,-80f,geometry);
            Require(lab.TryMoveTcp(ordinary.PositionMetres,ordinary.YawDegrees,ScaraJointLab.IkSelection.Nearest,out var first,out _), "Ordinary move rejected");
            Require(lab.TryMoveTcp(nearby.PositionMetres,nearby.YawDegrees,ScaraJointLab.IkSelection.Nearest,out var second,out _) && first.ElbowSign == second.ElbowSign, "Legitimate nearby replacement rejected");
            await Arrive(lab,token);
            float arrival = Vector3.Distance(lab.ActualTcpPosition,nearby.PositionMetres);
            Require(arrival < .000002f, "Nearby replacement actual arrival mismatch");
            await SetPose(Vector4.zero);
            Require(lab.TrySelectTcpSolution(shoulderNext.PositionMetres,shoulderNext.YawDegrees,ScaraJointLab.IkSelection.Nearest,out _,out _,out _), "Manual control did not reset old anchor");
            bool unchanged = paths.All(path => hashes[path] == Hash(path));
            Require(unchanged,"Source assets changed");
            var report = new { passed = true, utc = DateTime.UtcNow.ToString("O"), refusedRequests = refused, explicitSelections,
                inFlightAnchorPreserved = true, stopAndManualReset = true, nearbyReplacementArrived = true,
                nearbyArrivalErrorMetres = arrival, existingUiRefusalVisible = true, sourceAssetsUnchanged = unchanged,
                directButtonClicks = 0, riskyTransitionsExecuted = 0, rows };
            Directory.CreateDirectory("Logs/LW07");
            File.WriteAllText("Logs/LW07/continuity-moving-verification.json",JsonConvert.SerializeObject(report,Formatting.Indented));
            return report;
        }
        finally { Time.timeScale = scale; if (lab != null) lab.StopAll(); }
    }

    private static Vector4 Targets(ScaraJointLab l) => new Vector4(l.Axis1.TargetPosition,l.Axis2.TargetPosition,l.Axis3.TargetPosition,l.Axis4.TargetPosition);
    private static Vector4 Current(ScaraJointLab l) => new Vector4(l.Axis1.CurrentPosition,l.Axis2.CurrentPosition,l.Axis3.CurrentPosition,l.Axis4.CurrentPosition);
    private static float[] Values(Vector4 v) => new[] { v.x,v.y,v.z,v.w };
    private static float[] Values(Vector3 v) => new[] { v.x,v.y,v.z };
    private static async Task Arrive(ScaraJointLab lab,CancellationToken token)
    {
        var watch = Stopwatch.StartNew();
        while (lab.Axis1.IsMoving || lab.Axis2.IsMoving || lab.Axis3.IsMoving || lab.Axis4.IsMoving ||
            !lab.Axis1.IsAtTarget || !lab.Axis2.IsAtTarget || !lab.Axis3.IsAtTarget || !lab.Axis4.IsAtTarget)
        {
            token.ThrowIfCancellationRequested();
            Require(!EditorApplication.isPaused, "Verification interrupted by pause");
            if (watch.Elapsed.TotalSeconds > 12) throw new TimeoutException("Baseline pose arrival timeout");
            await Task.Delay(20,token);
        }
    }
    private static string Hash(string path)
    {
        using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))).Replace("-", "");
    }
    private static void Require(bool value,string message) { if (!value) throw new InvalidOperationException(message); }
}
