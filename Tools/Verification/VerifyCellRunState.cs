using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine.SceneManagement;

public static class VerifyCellRunState
{
    private sealed class BaselineEntry { public string path; public string sha256; }

    public static object Edit()
    {
        Require(!EditorApplication.isPlaying, "Use Edit Mode for the pure cell policy checks.");
        var sceneBefore = SceneSignature();
        // Capture in memory so a clean checkout can run this check without ignored local logs.
        var baseline = CaptureBaseline();
        var sources = new[] { "Assets/02.Scripts/LineWatch/CellRunStateMachine.cs",
            "Assets/02.Scripts/LineWatch/CellRunStateMachine.cs.meta", "Tools/Verification/VerifyCellRunState.cs" };
        var hashes = sources.ToDictionary(p => p, Hash);
        var rows = new List<object>();
        var states = new[] { CellRunState.Stopped, CellRunState.Running, CellRunState.Paused, CellRunState.Faulted };
        var commands = new[] { CellRunCommand.Start, CellRunCommand.Pause, CellRunCommand.Resume, CellRunCommand.Stop, CellRunCommand.Reset };

        // Independent specification: rows are current states; columns are commands above.
        // null means reject without changing either state or the first fault reason.
        CellRunState?[,] expected = {
            { CellRunState.Running, null, null, CellRunState.Stopped, null },
            { null, CellRunState.Paused, null, CellRunState.Stopped, null },
            { null, null, CellRunState.Running, CellRunState.Stopped, null },
            { null, null, null, CellRunState.Faulted, CellRunState.Stopped }
        };
        int accepted = 0, rejected = 0, invalidCommands = 0, policyChecks = 0, invalidSources = 0;
        int faultEntries = 0, invalidFaults = 0, repeatedFaults = 0;
        for (int s = 0; s < states.Length; s++)
        {
            for (int c = 0; c < commands.Length; c++)
            {
                var model = At(states[s]);
                string before = Signature(model);
                bool can = model.CanExecute(commands[c], out var previewError);
                Require(can == expected[s, c].HasValue && Signature(model) == before &&
                    (can ? previewError == null : !string.IsNullOrWhiteSpace(previewError)), "State query disagreed with the table or mutated state.");
                bool ok = model.TryExecute(commands[c], out var error);
                Require(ok == expected[s, c].HasValue, "Acceptance differs from the transition table.");
                if (ok)
                {
                    Require(error == null && model.State == expected[s, c].Value, "Wrong accepted state/error.");
                    Require(model.FaultReason == (states[s] == CellRunState.Faulted && commands[c] != CellRunCommand.Reset ? "first fault" : null), "Wrong active fault reason.");
                    Require(model.LastAcknowledgedFaultReason == (commands[c] == CellRunCommand.Reset ? "first fault" : null), "Wrong acknowledged cause history.");
                    accepted++;
                }
                else
                {
                    Require(!string.IsNullOrWhiteSpace(error) && Signature(model) == before, "Rejected command mutated state.");
                    rejected++;
                }
                rows.Add(new { from = states[s].ToString(), command = commands[c].ToString(), accepted = ok,
                    to = model.State.ToString(), model.FaultReason, error });
            }
            foreach (int invalid in new[] { -1, 99 })
            {
                var model = At(states[s]); string before = Signature(model);
                Require(!model.TryExecute((CellRunCommand)invalid, out var error) && !string.IsNullOrWhiteSpace(error) && Signature(model) == before,
                    "Undefined command was accepted or mutated state.");
                invalidCommands++;
            }

            var policy = At(states[s]); string policyBefore = Signature(policy);
            bool manual = policy.CanIssueRobotCommand(CellRobotCommandSource.Manual, out var manualError);
            bool automatic = policy.CanIssueRobotCommand(CellRobotCommandSource.Automatic, out var automaticError);
            Require(manual == (states[s] == CellRunState.Stopped) && automatic == (states[s] == CellRunState.Running), "Wrong robot admission policy.");
            Require(!manual || !automatic, "Manual and automatic commands were both admitted.");
            Require(manual ? manualError == null : !string.IsNullOrWhiteSpace(manualError), "Missing manual rejection reason.");
            Require(automatic ? automaticError == null : !string.IsNullOrWhiteSpace(automaticError), "Missing automatic rejection reason.");
            Require(Signature(policy) == policyBefore, "Admission query mutated state.");
            policyChecks += 2;
            foreach (int invalid in new[] { -1, 99 })
            {
                Require(!policy.CanIssueRobotCommand((CellRobotCommandSource)invalid, out var error) && !string.IsNullOrWhiteSpace(error) && Signature(policy) == policyBefore,
                    "Undefined source was accepted or mutated state.");
                invalidSources++;
            }
            foreach (string reason in new[] { null, "", " \t" })
            {
                var model = At(states[s]); string before = Signature(model);
                Require(!model.TryFault(reason, out var error) && !string.IsNullOrWhiteSpace(error) && Signature(model) == before, "Invalid fault changed state.");
                invalidFaults++;
            }
            var fault = At(states[s]);
            if (states[s] != CellRunState.Faulted) Require(fault.TryFault("first fault", out var error) && error == null, "Fault entry rejected.");
            Require(fault.IsFaulted && fault.State == CellRunState.Faulted && fault.FaultReason == "first fault", "Fault did not latch.");
            faultEntries++;
            string faultBefore = Signature(fault);
            Require(!fault.TryFault("later fault", out var laterError) && !string.IsNullOrWhiteSpace(laterError) && Signature(fault) == faultBefore, "Repeated fault replaced first cause.");
            repeatedFaults++;
        }

        // Stateful sequence catches duplicate start, pause/start confusion, explicit resume,
        // repeated stop and the distinction between stop and fault acknowledgement.
        var sequence = new CellRunStateMachine();
        var sequenceCommands = new[] { CellRunCommand.Start, CellRunCommand.Start, CellRunCommand.Pause, CellRunCommand.Start,
            CellRunCommand.Pause, CellRunCommand.Resume, CellRunCommand.Resume, CellRunCommand.Stop, CellRunCommand.Stop,
            CellRunCommand.Resume, CellRunCommand.Start };
        var sequenceAccepted = new[] { true, false, true, false, false, true, false, true, true, false, true };
        var sequenceStates = new[] { CellRunState.Running, CellRunState.Running, CellRunState.Paused, CellRunState.Paused,
            CellRunState.Paused, CellRunState.Running, CellRunState.Running, CellRunState.Stopped, CellRunState.Stopped,
            CellRunState.Stopped, CellRunState.Running };
        for (int i = 0; i < sequenceCommands.Length; i++)
        {
            string before = Signature(sequence);
            bool ok = sequence.TryExecute(sequenceCommands[i], out var error);
            Require(ok == sequenceAccepted[i] && sequence.State == sequenceStates[i], "Stateful command sequence mismatch.");
            Require(ok ? error == null : !string.IsNullOrWhiteSpace(error) && Signature(sequence) == before, "Sequence rejection mutated state.");
        }
        Require(sequence.TryFault("product tracking interrupted", out _), "Sequence fault rejected.");
        string latched = Signature(sequence);
        for (int repeat = 0; repeat < 3; repeat++)
        {
            Require(sequence.TryExecute(CellRunCommand.Stop, out _), "Faulted stop rejected.");
            Require(!sequence.TryExecute(CellRunCommand.Start, out _), "Faulted start accepted.");
            Require(!sequence.TryExecute(CellRunCommand.Resume, out _), "Faulted resume accepted.");
            Require(sequence.CanExecute(CellRunCommand.Reset, out _), "Explicit acknowledgement not admitted by pure policy.");
            Require(Signature(sequence) == latched, "Fault cleared without explicit recovery.");
        }
        Require(sequence.TryExecute(CellRunCommand.Reset, out _) && sequence.State == CellRunState.Stopped && !sequence.IsFaulted &&
            sequence.FaultReason == null && sequence.LastAcknowledgedFaultReason == "product tracking interrupted", "Explicit acknowledgement did not stop/preserve cause.");
        string acknowledged = Signature(sequence);
        Require(!sequence.TryExecute(CellRunCommand.Reset, out _) && Signature(sequence) == acknowledged, "Duplicate acknowledgement changed state/history.");
        Require(new CellRunStateMachine().State == CellRunState.Stopped && new CellRunStateMachine().LastAcknowledgedFaultReason == null, "Separate model instances contaminated each other.");

        CheckBaseline(baseline);
        Require(hashes.All(p => Hash(p.Key) == p.Value), "Verification changed sources.");
        Require(SceneSignature() == sceneBefore, "Verification changed loaded scenes/dirty state.");
        var report = new { passed = true, verifiedUtc = DateTime.UtcNow, environment = "Unity " + UnityEngine.Application.unityVersion + " Editor / Edit Mode",
            transitionCases = 20, readOnlyCommandQueries = 20, acceptedTransitions = accepted, rejectedTransitionsPreserved = rejected, invalidCommandsPreserved = invalidCommands,
            robotAdmissionChecks = policyChecks, invalidSourcesPreserved = invalidSources, faultStateChecks = faultEntries,
            invalidFaultsPreserved = invalidFaults, repeatedFaultsPreserved = repeatedFaults,
            statefulCommands = sequenceCommands.Length, faultPersistenceCycles = 3,
            existingFilesPreserved = baseline.Length, sourceHashes = hashes, baseline,
            sceneSignature = sceneBefore, sceneUnchanged = true, rows,
            actualRobotMoves = 0, directButtonClicks = 0, callbacksInjected = 0,
            scope = "Pure state/admission/explicit acknowledgement policy. Scene recovery eligibility and physical/FIFO preservation are verified separately." };
        Directory.CreateDirectory("Logs/LW11");
        File.WriteAllText("Logs/LW11/cell-run-state-verification.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        return JsonConvert.SerializeObject(new { report.passed, report.transitionCases, report.acceptedTransitions, report.rejectedTransitionsPreserved,
            report.invalidCommandsPreserved, report.robotAdmissionChecks, report.invalidSourcesPreserved, report.faultStateChecks,
            report.invalidFaultsPreserved, report.repeatedFaultsPreserved, report.statefulCommands, report.faultPersistenceCycles, report.existingFilesPreserved, report.sceneUnchanged });
    }

    private static CellRunStateMachine At(CellRunState state)
    {
        var model = new CellRunStateMachine();
        if (state == CellRunState.Running || state == CellRunState.Paused) Require(model.TryExecute(CellRunCommand.Start, out _), "Fixture start failed.");
        if (state == CellRunState.Paused) Require(model.TryExecute(CellRunCommand.Pause, out _), "Fixture pause failed.");
        if (state == CellRunState.Faulted) Require(model.TryFault("first fault", out _), "Fixture fault failed.");
        Require(model.State == state, "Fixture state differs from specification.");
        return model;
    }

    private static string SceneSignature() => JsonConvert.SerializeObject(Enumerable.Range(0, SceneManager.sceneCount)
        .Select(i => SceneManager.GetSceneAt(i)).Select(s => new { s.path, s.isDirty, s.isLoaded }).ToArray());
    private static string Signature(CellRunStateMachine model) => JsonConvert.SerializeObject(new { model.State, model.IsFaulted, model.FaultReason, model.LastAcknowledgedFaultReason });
    private static BaselineEntry[] CaptureBaseline() => new[] { "Assets/01.Scenes", "Assets/02.Scripts", "Assets/03.Prefabs",
        "Assets/04.ScriptableObject", "ProjectSettings" }.SelectMany(p => Directory.GetFiles(p, "*", SearchOption.AllDirectories))
        .Concat(new[] { "Packages/manifest.json", "Packages/packages-lock.json" }).OrderBy(p => p, StringComparer.Ordinal)
        .Select(p => new BaselineEntry { path = p, sha256 = Hash(p) }).ToArray();
    private static void CheckBaseline(BaselineEntry[] baseline) => Require(baseline.All(p => File.Exists(p.path) && Hash(p.path) == p.sha256), "Existing source/asset or user change differs from the pre-edit baseline.");
    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))).Replace("-", ""); }
    private static void Require(bool value, string message) { if (!value) throw new InvalidOperationException(message); }
}
