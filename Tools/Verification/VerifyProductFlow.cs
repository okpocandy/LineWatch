using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;

public static class VerifyProductFlow
{
    private static readonly string[] Sources = {
        "Assets/01.Scenes/LineWatch.unity",
        "ProjectSettings/ProjectSettings.asset", "ProjectSettings/EditorBuildSettings.asset",
        "Assets/03.Prefabs/ScaraRobot_Work.prefab", "Assets/03.Prefabs/FanucRobot_Work.prefab",
        "Assets/03.Prefabs/GripTrialPart.prefab", "Assets/03.Prefabs/HandoffTrialPart.prefab",
        "Assets/04.ScriptableObject/ScaraCellHandoff.asset", "Assets/04.ScriptableObject/FanucCellHandoff.asset",
        "Assets/02.Scripts/LineWatch/GrippableItem.cs", "Assets/02.Scripts/LineWatch/RobotGripper.cs",
        "Assets/02.Scripts/LineWatch/RobotGripperLab.cs", "Assets/02.Scripts/LineWatch/RobotWaypointSequence.cs",
        "Assets/02.Scripts/LineWatch/ProductRegistry.cs", "Assets/02.Scripts/LineWatch/ProductRegistry.cs.meta",
        "Assets/02.Scripts/LineWatch/ProductFlow.cs", "Assets/02.Scripts/LineWatch/ProductFlow.cs.meta"
    };

    public static object Edit()
    {
        Require(!EditorApplication.isPlaying, "Use Edit Mode for pure-data flow checks.");
        var hashes = Sources.ToDictionary(p => p, Hash);
        var flow = new ProductFlow();
        var destinations = new Dictionary<string, ProductDestination>(StringComparer.Ordinal);
        var fed = new List<string>(); var rows = new List<object>();
        int accepted = 0, rejected = 0, blocked = 0, invariantChecks = 0, transfers = 0, removals = 0;
        string Signature() => JsonConvert.SerializeObject(new {
            products = flow.Snapshot().OrderBy(p => p.Id).Select(p => new { p.Id, p.Destination, p.Location }).ToArray(),
            waiting = flow.WaitingSnapshot(), flow.InputProductId, flow.ScaraProductId, flow.HandoffProductId, flow.FanucProductId });
        void Check()
        {
            var records = flow.Snapshot(); var waiting = flow.WaitingSnapshot();
            Require(records.Count == flow.Count && records.Select(p => p.Id).Distinct(StringComparer.Ordinal).Count() == flow.Count, "Record lost/duplicated.");
            Require(waiting.Count == flow.WaitingCount && waiting.Distinct(StringComparer.Ordinal).Count() == waiting.Count, "Duplicate waiting ID.");
            Require(waiting.OrderBy(id => id, StringComparer.Ordinal).SequenceEqual(records.Where(p => p.Location == ProductLocation.Waiting).Select(p => p.Id).OrderBy(id => id, StringComparer.Ordinal)), "Queue/record membership mismatch.");
            var slots = new[] { (ProductLocation.Input, flow.InputProductId), (ProductLocation.Scara, flow.ScaraProductId),
                (ProductLocation.Handoff, flow.HandoffProductId), (ProductLocation.Fanuc, flow.FanucProductId) };
            foreach (var slot in slots)
            {
                var occupants = records.Where(p => p.Location == slot.Item1).ToArray();
                Require(occupants.Length <= 1 && (occupants.Length == 0 ? slot.Item2 == null : occupants[0].Id == slot.Item2), "Slot capacity/ownership mismatch.");
            }
            Require(records.All(p => destinations.TryGetValue(p.Id, out var destination) && destination == p.Destination), "Destination changed.");
            invariantChecks++;
        }
        void Failure(bool success, ProductRecord product, string error, string before, string request, bool expectedWait = false)
        {
            Require(!success && product == null && !string.IsNullOrEmpty(error) && Signature() == before, "Rejected request mutated flow: " + request);
            rejected++; if (expectedWait) blocked++;
            rows.Add(new { request, accepted = false, normalCapacityWait = expectedWait, error }); Check();
        }
        void Enqueue(string id, ProductDestination destination)
        {
            Require(flow.TryEnqueue(id, destination, out var p, out var e), e); destinations.Add(id, destination);
            Require(p.Id == id && p.Location == ProductLocation.Waiting, "Enqueue did not register waiting item.");
            accepted++; Check();
        }
        void Feed(string expectedId)
        {
            Require(flow.TryFeedNext(out var p, out var e), e); Require(p.Id == expectedId, "FIFO order changed.");
            fed.Add(p.Id); accepted++; Check();
        }
        void Transfer(string id, ProductLocation from, ProductLocation to)
        {
            Require(flow.TryTransfer(id, from, to, out var p, out var e), e); Require(p.Id == id && p.Location == to, "Transfer result mismatch.");
            accepted++; transfers++; Check();
        }
        void Remove(string id, ProductLocation from)
        {
            Require(flow.TryRemove(id, from, out var p, out var e), e); Require(p.Location == ProductLocation.Removed, "Removal result mismatch.");
            accepted++; removals++; Check();
        }
        void FailFeed(bool expectedWait = false)
        {
            string before = Signature(); bool ok = flow.TryFeedNext(out var p, out var e); Failure(ok, p, e, before, "feed", expectedWait);
        }
        void FailTransfer(string id, ProductLocation from, ProductLocation to, bool expectedWait = false)
        {
            string before = Signature(); bool ok = flow.TryTransfer(id, from, to, out var p, out var e); Failure(ok, p, e, before, id + ": " + from + " -> " + to, expectedWait);
        }
        void FailRemove(string id, ProductLocation from)
        {
            string before = Signature(); bool ok = flow.TryRemove(id, from, out var p, out var e); Failure(ok, p, e, before, "remove " + id);
        }
        void FailEnqueue(string id, ProductDestination destination)
        {
            string before = Signature(); bool ok = flow.TryEnqueue(id, destination, out var p, out var e); Failure(ok, p, e, before, "enqueue " + id);
        }

        FailFeed();
        Enqueue("A", ProductDestination.A); Enqueue("B", ProductDestination.B);
        Enqueue("C", ProductDestination.A); Enqueue("D", ProductDestination.B);
        FailEnqueue("A", ProductDestination.B); FailEnqueue(null, ProductDestination.A); FailEnqueue("BAD", (ProductDestination)99);
        FailTransfer("A", ProductLocation.Waiting, ProductLocation.Input); FailTransfer("B", ProductLocation.Waiting, ProductLocation.Input);
        Feed("A"); FailFeed(true);
        FailTransfer("C", ProductLocation.Input, ProductLocation.Scara);
        Transfer("A", ProductLocation.Input, ProductLocation.Scara); Feed("B");
        FailTransfer("B", ProductLocation.Input, ProductLocation.Scara, true);
        Transfer("A", ProductLocation.Scara, ProductLocation.Handoff);
        Transfer("B", ProductLocation.Input, ProductLocation.Scara); Feed("C"); FailFeed(true);
        FailTransfer("B", ProductLocation.Scara, ProductLocation.Handoff, true);
        FailTransfer("C", ProductLocation.Input, ProductLocation.Scara, true);
        Enqueue("E", ProductDestination.A); Enqueue("F", ProductDestination.A);
        var queueBeforeRemoval = flow.WaitingSnapshot(); Remove("E", ProductLocation.Waiting);
        Require(queueBeforeRemoval.SequenceEqual(new[] { "D", "E", "F" }) && flow.WaitingSnapshot().SequenceEqual(new[] { "D", "F" }), "Middle removal changed remaining order or old snapshot.");
        FailEnqueue("E", ProductDestination.B); FailTransfer("E", ProductLocation.Removed, ProductLocation.Input); FailRemove("E", ProductLocation.Waiting);
        Transfer("A", ProductLocation.Handoff, ProductLocation.Fanuc);
        Transfer("B", ProductLocation.Scara, ProductLocation.Handoff);
        Transfer("C", ProductLocation.Input, ProductLocation.Scara); Feed("D");
        FailTransfer("B", ProductLocation.Handoff, ProductLocation.Fanuc, true);
        FailTransfer("C", ProductLocation.Scara, ProductLocation.Handoff, true);
        FailTransfer("D", ProductLocation.Input, ProductLocation.Scara, true); FailFeed(true);
        FailRemove("A", ProductLocation.Fanuc); FailTransfer("A", ProductLocation.Fanuc, ProductLocation.OutputB);
        Transfer("A", ProductLocation.Fanuc, ProductLocation.OutputA);
        FailTransfer("A", ProductLocation.OutputA, ProductLocation.Input); FailRemove("A", ProductLocation.OutputA);
        Transfer("B", ProductLocation.Handoff, ProductLocation.Fanuc);
        Transfer("C", ProductLocation.Scara, ProductLocation.Handoff);
        Transfer("D", ProductLocation.Input, ProductLocation.Scara); Feed("F"); FailFeed(true);
        FailTransfer("B", ProductLocation.Fanuc, ProductLocation.OutputA);
        Transfer("B", ProductLocation.Fanuc, ProductLocation.OutputB);
        Transfer("C", ProductLocation.Handoff, ProductLocation.Fanuc); FailRemove("D", ProductLocation.Scara);
        Transfer("C", ProductLocation.Fanuc, ProductLocation.OutputA);
        Transfer("D", ProductLocation.Scara, ProductLocation.Handoff);
        Transfer("F", ProductLocation.Input, ProductLocation.Scara);
        Transfer("D", ProductLocation.Handoff, ProductLocation.Fanuc);
        Transfer("F", ProductLocation.Scara, ProductLocation.Handoff);
        Transfer("D", ProductLocation.Fanuc, ProductLocation.OutputB);
        Transfer("F", ProductLocation.Handoff, ProductLocation.Fanuc);
        Transfer("F", ProductLocation.Fanuc, ProductLocation.OutputA);
        FailFeed(); FailEnqueue("A", ProductDestination.A); FailTransfer("F", ProductLocation.Handoff, ProductLocation.Fanuc);
        Enqueue("G", ProductDestination.A); Enqueue("H", ProductDestination.B);
        Feed("G"); Remove("G", ProductLocation.Input); Require(flow.InputProductId == null, "Input removal did not free slot.");
        Feed("H"); Transfer("H", ProductLocation.Input, ProductLocation.Scara); Transfer("H", ProductLocation.Scara, ProductLocation.Handoff);
        FailTransfer("H", ProductLocation.Handoff, ProductLocation.Removed);
        Remove("H", ProductLocation.Handoff); Require(flow.HandoffProductId == null, "Handoff removal did not free slot.");
        FailRemove("H", ProductLocation.Handoff); FailEnqueue("G", ProductDestination.B);
        FailTransfer("missing", ProductLocation.Scara, ProductLocation.Handoff); FailRemove("missing", ProductLocation.Waiting);
        FailTransfer("H", (ProductLocation)99, ProductLocation.Fanuc); FailTransfer("H", ProductLocation.Removed, (ProductLocation)99);
        FailRemove("H", ProductLocation.Removed); FailRemove("G", ProductLocation.Input);
        Require(!flow.TryGet(null, out _) && !flow.TryGet("missing", out _), "Missing lookup accepted.");
        Require(fed.SequenceEqual(new[] { "A", "B", "C", "D", "F", "G", "H" }), "FIFO feed trace mismatch.");
        var final = flow.Snapshot(); int completedA = final.Count(p => p.Location == ProductLocation.OutputA), completedB = final.Count(p => p.Location == ProductLocation.OutputB);
        int removed = final.Count(p => p.Location == ProductLocation.Removed);
        Require(flow.Count == 8 && completedA == 3 && completedB == 2 && removed == 3 && flow.Count == completedA + completedB + removed &&
            flow.WaitingCount == 0 && flow.InputProductId == null && flow.ScaraProductId == null && flow.HandoffProductId == null && flow.FanucProductId == null, "Conservation or final occupancy failed.");
        Require(hashes.All(p => Hash(p.Key) == p.Value), "Sources changed during pure-data verification.");
        var report = new { passed = true, registeredProducts = flow.Count, acceptedRequests = accepted, successfulFeeds = fed.Count,
            successfulTransfers = transfers, successfulRemovals = removals, rejectedRequestsPreserved = rejected, normalCapacityWaitsPreserved = blocked,
            invariantChecks, completedA, completedB, removedProducts = removed, fifoFeedOrder = fed,
            middleQueueRemovalPreservesOrder = true, allSlotsEmpty = true, conservationPassed = true, sourceAssetsUnchanged = true,
            hashes, rows, actualRobotMoves = 0, directButtonClicks = 0,
            scope = "LW-10 process 2: pure logical FIFO, one item per input/SCARA/handoff/FANUC slot, backpressure and removal conservation; no physical grip/scene or coordinated robot operation." };
        Directory.CreateDirectory("Logs/LW10"); File.WriteAllText("Logs/LW10/product-flow-verification.json", JsonConvert.SerializeObject(report, Formatting.Indented));
        return JsonConvert.SerializeObject(new { report.passed, report.registeredProducts, report.acceptedRequests, report.successfulFeeds, report.successfulTransfers,
            report.successfulRemovals, report.rejectedRequestsPreserved, report.normalCapacityWaitsPreserved, report.invariantChecks,
            report.completedA, report.completedB, report.removedProducts, report.conservationPassed, report.sourceAssetsUnchanged });
    }

    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))); }
    private static void Require(bool value, string message) { if (!value) throw new InvalidOperationException(message); }
}
