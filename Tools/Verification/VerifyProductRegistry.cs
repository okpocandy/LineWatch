using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;

public static class VerifyProductRegistry
{
    private static readonly string[] Sources = {
        "Assets/01.Scenes/LineWatch.unity", "Assets/01.Scenes/RobotLab.unity",
        "ProjectSettings/ProjectSettings.asset", "ProjectSettings/EditorBuildSettings.asset",
        "Assets/03.Prefabs/ScaraRobot_Work.prefab", "Assets/03.Prefabs/FanucRobot_Work.prefab",
        "Assets/03.Prefabs/GripTrialPart.prefab", "Assets/03.Prefabs/HandoffTrialPart.prefab",
        "Assets/04.ScriptableObject/ScaraCellHandoff.asset", "Assets/04.ScriptableObject/FanucCellHandoff.asset",
        "Assets/02.Scripts/LineWatch/GrippableItem.cs", "Assets/02.Scripts/LineWatch/RobotGripper.cs",
        "Assets/02.Scripts/LineWatch/RobotGripperLab.cs", "Assets/02.Scripts/LineWatch/RobotWaypointSequence.cs",
        "Assets/02.Scripts/LineWatch/ProductRegistry.cs"
    };

    public static object Edit()
    {
        Require(!EditorApplication.isPlaying, "Use Edit Mode; no physical robot moves are required.");
        var hashes = Sources.ToDictionary(p => p, Hash);
        var registry = new ProductRegistry();
        int rejected = 0, transitions = 0; var rows = new List<object>();
        string Snapshot() => string.Join("|", registry.Snapshot().OrderBy(p => p.Id).Select(p => p.Id + ":" + p.Destination + ":" + p.Location));
        void RejectRegistration(string id, ProductDestination destination)
        {
            string before = Snapshot();
            Require(!registry.TryRegister(id, destination, out var product, out var error) && product == null && !string.IsNullOrEmpty(error) && before == Snapshot(), "Registration rejection changed records.");
            rejected++;
        }
        void RejectTransition(string id, ProductLocation expected, ProductLocation next)
        {
            string before = Snapshot();
            Require(!registry.TryTransition(id, expected, next, out var product, out var error) && product == null && !string.IsNullOrEmpty(error) && before == Snapshot(), "Transition rejection changed records.");
            rejected++;
        }
        void Register(string id, ProductDestination destination)
        {
            Require(registry.TryRegister(id, destination, out var record, out var error), error);
            Require(record.Id == id && record.Destination == destination && record.Location == ProductLocation.Waiting, "Registration fields changed.");
        }
        void Move(string id, ProductLocation expected, ProductLocation next)
        {
            Require(registry.TryGet(id, out var before), "Missing product.");
            Require(registry.TryTransition(id, expected, next, out var record, out var error), error);
            Require(record.Id == before.Id && record.Destination == before.Destination && record.Location == next && before.Location == expected, "Identity/destination or old snapshot changed.");
            transitions++; rows.Add(new { id, from = expected.ToString(), to = next.ToString() });
        }

        foreach (var id in new[] { null, "", " ", " A", "A " }) RejectRegistration(id, ProductDestination.A);
        RejectRegistration("BAD", (ProductDestination)99);
        Require(!registry.TryGet(null, out _) && !registry.TryGet("missing", out _), "Missing lookup accepted.");
        Register("A-001", ProductDestination.A); Register("B-001", ProductDestination.B);
        var initial = registry.Snapshot();
        RejectRegistration("A-001", ProductDestination.B);
        RejectTransition("missing", ProductLocation.Waiting, ProductLocation.Input);
        RejectTransition(null, ProductLocation.Waiting, ProductLocation.Input);
        RejectTransition("A-001", (ProductLocation)99, ProductLocation.Input);
        RejectTransition("A-001", ProductLocation.Waiting, (ProductLocation)99);
        RejectTransition("A-001", ProductLocation.Waiting, ProductLocation.Fanuc);
        RejectTransition("A-001", ProductLocation.Waiting, ProductLocation.Waiting);
        Move("A-001", ProductLocation.Waiting, ProductLocation.Input);
        Move("B-001", ProductLocation.Waiting, ProductLocation.Input);
        RejectTransition("A-001", ProductLocation.Input, ProductLocation.OutputA);
        Move("A-001", ProductLocation.Input, ProductLocation.Scara);
        RejectTransition("A-001", ProductLocation.Input, ProductLocation.Scara);
        RejectTransition("A-001", ProductLocation.Scara, ProductLocation.Fanuc);
        RejectTransition("A-001", ProductLocation.Scara, ProductLocation.Removed);
        Move("A-001", ProductLocation.Scara, ProductLocation.Handoff);
        RejectTransition("A-001", ProductLocation.Scara, ProductLocation.Handoff);
        Move("B-001", ProductLocation.Input, ProductLocation.Scara);
        Move("B-001", ProductLocation.Scara, ProductLocation.Handoff);
        Move("A-001", ProductLocation.Handoff, ProductLocation.Fanuc);
        RejectTransition("A-001", ProductLocation.Fanuc, ProductLocation.OutputB);
        RejectTransition("A-001", ProductLocation.Fanuc, ProductLocation.Removed);
        Move("B-001", ProductLocation.Handoff, ProductLocation.Fanuc);
        RejectTransition("B-001", ProductLocation.Fanuc, ProductLocation.OutputA);
        Move("A-001", ProductLocation.Fanuc, ProductLocation.OutputA);
        RejectTransition("A-001", ProductLocation.OutputA, ProductLocation.Input);
        RejectTransition("A-001", ProductLocation.OutputA, ProductLocation.Removed);
        Move("B-001", ProductLocation.Fanuc, ProductLocation.OutputB);
        RejectRegistration("A-001", ProductDestination.A);

        var removable = new[] { ProductLocation.Waiting, ProductLocation.Input, ProductLocation.Handoff };
        foreach (var location in removable)
        {
            string id = "REMOVED-" + location; Register(id, ProductDestination.A);
            if (location != ProductLocation.Waiting) Move(id, ProductLocation.Waiting, ProductLocation.Input);
            if (location == ProductLocation.Handoff)
            {
                Move(id, ProductLocation.Input, ProductLocation.Scara);
                Move(id, ProductLocation.Scara, ProductLocation.Handoff);
            }
            Move(id, location, ProductLocation.Removed);
            RejectRegistration(id, ProductDestination.B);
            RejectTransition(id, ProductLocation.Removed, ProductLocation.Input);
        }
        var final = registry.Snapshot();
        Require(registry.Count == 5 && final.Count == registry.Count && final.Select(p => p.Id).Distinct().Count() == registry.Count, "Product records were lost or duplicated.");
        Require(final.Count(p => p.Location == ProductLocation.OutputA) == 1 && final.Count(p => p.Location == ProductLocation.OutputB) == 1 && final.Count(p => p.Location == ProductLocation.Removed) == 3, "Conservation or destination mismatch.");
        Require(initial.Count == 2 && initial.All(p => p.Location == ProductLocation.Waiting), "Old snapshots changed after later transitions.");
        Require(hashes.All(p => Hash(p.Key) == p.Value), "Source changed during pure-data verification.");
        var report = new { passed = true, registeredProducts = registry.Count, completedA = 1, completedB = 1, removedProducts = 3,
            acceptedTransitions = transitions, rejectedRequestsPreserved = rejected, immutableSnapshotsPassed = true,
            idReuseAfterCompletionAndRemovalRejected = true, conservationPassed = true, sourceAssetsUnchanged = true,
            hashes, rows, actualRobotMoves = 0, directButtonClicks = 0,
            scope = "LW-10 process 1: pure logical product identity/destination/state records only. Station capacity, FIFO, physical grip/scene binding and actual output counting are not implemented yet." };
        Directory.CreateDirectory("Logs/LW10"); File.WriteAllText("Logs/LW10/product-registry-verification.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        return JsonConvert.SerializeObject(new { report.passed, report.registeredProducts, report.acceptedTransitions, report.rejectedRequestsPreserved,
            report.completedA, report.completedB, report.removedProducts, report.immutableSnapshotsPassed, report.conservationPassed, report.sourceAssetsUnchanged });
    }

    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))); }
    private static void Require(bool condition, string message) { if (!condition) throw new InvalidOperationException(message); }
}
