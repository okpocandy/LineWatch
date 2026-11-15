using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading;
using System.Threading.Tasks;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;

public static class VerifyFanucForwardKinematicsFull
{
    private const string PrefabPath = "Assets/03.Prefabs/FanucRobot_Work.prefab";
    private const float PositionTolerance = .00001f;
    private const float RotationTolerance = .005f;
    private const int Seed = 20261114;
    private static readonly string[] SourcePaths = { PrefabPath, "Assets/01.Scenes/RobotLab.unity",
        "Assets/03.Prefabs/ScaraRobot_Work.prefab", "ProjectSettings/ProjectSettings.asset" };

    public static object Static()
    {
        Require(!EditorApplication.isPlaying, "Run isolated comparisons in Edit Mode.");
        var hashes = SourcePaths.ToDictionary(p => p, Hash);
        var root = PrefabUtility.LoadPrefabContents(PrefabPath);
        try
        {
            var lab = root.GetComponent<FanucJointLab>();
            Require(lab != null && lab.RobotFrame == root.transform && lab.Tcp.parent == lab.Joints[5].transform,
                "Expected saved RobotFrame and direct A6->TCP.");
            var homes = lab.Joints.Select(j => j.transform.localRotation).ToArray();
            var originalGeometry = lab.Geometry;
            var poses = Poses(lab);
            var placements = new[] {
                new { name = "Authored", position = root.transform.position, rotation = root.transform.rotation },
                new { name = "Translated and yawed", position = new Vector3(2.1f,1.2f,-3f), rotation = Quaternion.Euler(0,37,0) },
                new { name = "Translated and tilted", position = new Vector3(-4.2f,.8f,2.3f), rotation = Quaternion.Euler(20,47,-15) }
            };
            var tools = new[] {
                new { name = "Original", position = originalGeometry.ToolPositionMetres, rotation = originalGeometry.ToolRotation },
                new { name = "3D position offset", position = originalGeometry.ToolPositionMetres + new Vector3(-.02f,.015f,.025f), rotation = originalGeometry.ToolRotation },
                new { name = "3D rotation offset", position = originalGeometry.ToolPositionMetres, rotation = originalGeometry.ToolRotation * Quaternion.Euler(25,-35,15) },
                new { name = "Combined offset", position = originalGeometry.ToolPositionMetres + new Vector3(-.03f,-.02f,.035f), rotation = originalGeometry.ToolRotation * Quaternion.Euler(-20,30,40) }
            };
            void Apply(float[] q)
            {
                for (int i = 0; i < 6; i++)
                    lab.Joints[i].transform.localRotation = homes[i] * Quaternion.AngleAxis(q[i],lab.Joints[i].LocalRotationAxis);
            }
            int comparisons = 0, failures = 0;
            var details = new List<object>();
            var groups = new List<object>();
            float maxLocal = 0, maxWorld = 0, maxLocalRotation = 0, maxWorldRotation = 0, maxInvariant = 0, maxInvariantRotation = 0;
            foreach (var tool in tools)
            {
                lab.Tcp.SetLocalPositionAndRotation(tool.position,tool.rotation);
                var geometry = new FanucKinematicGeometry(originalGeometry.Joints.ToArray(),tool.position,tool.rotation);
                var baselinePosition = new Vector3[poses.Count];
                var baselineRotation = new Quaternion[poses.Count];
                for (int placementIndex = 0; placementIndex < placements.Length; placementIndex++)
                {
                    var placement = placements[placementIndex];
                    root.transform.SetPositionAndRotation(placement.position,placement.rotation);
                    float groupLocal = 0, groupWorld = 0, groupLocalRotation = 0, groupWorldRotation = 0, groupInvariant = 0, groupInvariantRotation = 0;
                    for (int poseIndex = 0; poseIndex < poses.Count; poseIndex++)
                    {
                        var q = poses[poseIndex];
                        Apply(q);
                        var calculated = FanucForwardKinematics.Calculate(q,geometry);
                        var actualLocal = lab.RobotFrame.InverseTransformPoint(lab.Tcp.position);
                        var actualRotation = Quaternion.Inverse(lab.RobotFrame.rotation) * lab.Tcp.rotation;
                        float local = Vector3.Distance(calculated.PositionMetres,actualLocal);
                        float world = Vector3.Distance(lab.RobotFrame.TransformPoint(calculated.PositionMetres),lab.Tcp.position);
                        float localRotation = Angle(calculated.Rotation,actualRotation);
                        float worldRotation = Angle(lab.RobotFrame.rotation * calculated.Rotation,lab.Tcp.rotation);
                        float invariant = 0, invariantRotation = 0;
                        if (placementIndex == 0) { baselinePosition[poseIndex] = actualLocal; baselineRotation[poseIndex] = actualRotation; }
                        else
                        {
                            invariant = Vector3.Distance(baselinePosition[poseIndex],actualLocal);
                            invariantRotation = Angle(baselineRotation[poseIndex],actualRotation);
                        }
                        groupLocal = Mathf.Max(groupLocal,local); groupWorld = Mathf.Max(groupWorld,world);
                        groupLocalRotation = Mathf.Max(groupLocalRotation,localRotation); groupWorldRotation = Mathf.Max(groupWorldRotation,worldRotation);
                        groupInvariant = Mathf.Max(groupInvariant,invariant); groupInvariantRotation = Mathf.Max(groupInvariantRotation,invariantRotation);
                        comparisons++;
                        if (local > PositionTolerance || world > PositionTolerance || invariant > PositionTolerance ||
                            localRotation > RotationTolerance || worldRotation > RotationTolerance || invariantRotation > RotationTolerance)
                        {
                            failures++;
                            if (details.Count < 8) details.Add(new { tool=tool.name, placement=placement.name, poseIndex,
                                q, local, world, localRotation, worldRotation, invariant, invariantRotation });
                        }
                    }
                    maxLocal = Mathf.Max(maxLocal,groupLocal); maxWorld = Mathf.Max(maxWorld,groupWorld);
                    maxLocalRotation = Mathf.Max(maxLocalRotation,groupLocalRotation); maxWorldRotation = Mathf.Max(maxWorldRotation,groupWorldRotation);
                    maxInvariant = Mathf.Max(maxInvariant,groupInvariant); maxInvariantRotation = Mathf.Max(maxInvariantRotation,groupInvariantRotation);
                    groups.Add(new { tool=tool.name, placement=placement.name, comparisonCount=poses.Count,
                        toolPositionMetres=V(tool.position), toolRotationXyzw=Q(tool.rotation),
                        maxLocalPositionErrorMetres=groupLocal, maxWorldPositionErrorMetres=groupWorld,
                        maxLocalRotationErrorDegrees=groupLocalRotation, maxWorldRotationErrorDegrees=groupWorldRotation,
                        maxPlacementInvariantErrorMetres=groupInvariant, maxPlacementInvariantRotationDegrees=groupInvariantRotation });
                }
            }
            // Leave the changed TCP in place but deliberately calculate with its old calibration.
            var changed = tools[3];
            root.transform.SetPositionAndRotation(placements[0].position,placements[0].rotation);
            Apply(poses[19]);
            var old = FanucForwardKinematics.Calculate(poses[19],originalGeometry);
            float mismatchPosition = Vector3.Distance(old.PositionMetres,lab.RobotFrame.InverseTransformPoint(lab.Tcp.position));
            float mismatchRotation = Angle(old.Rotation,Quaternion.Inverse(lab.RobotFrame.rotation)*lab.Tcp.rotation);
            bool mismatchDetected = Mathf.Abs(mismatchPosition-Vector3.Distance(changed.position,originalGeometry.ToolPositionMetres)) < PositionTolerance
                && Mathf.Abs(mismatchRotation-Angle(changed.rotation,originalGeometry.ToolRotation)) < RotationTolerance
                && mismatchPosition > PositionTolerance && mismatchRotation > RotationTolerance;
            bool unchanged = hashes.All(p => Hash(p.Key)==p.Value);
            var report = new { passed=failures==0 && mismatchDetected && unchanged, utc=DateTime.UtcNow.ToString("o"), seed=Seed,
                distinctJointPoses=poses.Count, placements=placements.Length, toolVariants=tools.Length, staticPoseComparisons=comparisons,
                positionToleranceMetres=PositionTolerance, rotationToleranceDegrees=RotationTolerance, failures,
                maxLocalPositionErrorMetres=maxLocal, maxWorldPositionErrorMetres=maxWorld,
                maxLocalRotationErrorDegrees=maxLocalRotation, maxWorldRotationErrorDegrees=maxWorldRotation,
                maxPlacementInvariantErrorMetres=maxInvariant, maxPlacementInvariantRotationDegrees=maxInvariantRotation,
                staleToolPositionErrorMetres=mismatchPosition, staleToolRotationErrorDegrees=mismatchRotation,
                staleToolMismatchDetected=mismatchDetected, sourceAssetsUnchanged=unchanged, sourceHashes=hashes,
                scope="1200 isolated FK/Transform comparisons; no motor cycles, IK, collisions or physical robot accuracy",
                jointLimits=lab.Joints.Select(j=>new{minimum=j.MinimumDegrees,maximum=j.MaximumDegrees}).ToArray(),
                jointPoses=poses, groups, failureDetails=details };
            Save("full-verification.json",report);
            Require(report.passed,"Full static FK validation failed; inspect Logs/LW07B/full-verification.json.");
            return new { report.passed, report.distinctJointPoses, report.staticPoseComparisons, report.failures,
                report.maxLocalPositionErrorMetres, report.maxWorldPositionErrorMetres, report.maxLocalRotationErrorDegrees,
                report.maxWorldRotationErrorDegrees, report.maxPlacementInvariantErrorMetres, report.maxPlacementInvariantRotationDegrees,
                report.staleToolPositionErrorMetres, report.staleToolRotationErrorDegrees, report.staleToolMismatchDetected, report.sourceAssetsUnchanged };
        }
        finally { PrefabUtility.UnloadPrefabContents(root); }
    }

    public static async Task<object> Moving()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused,"Run RobotLab in unpaused Play Mode.");
        var lab=UnityEngine.Object.FindFirstObjectByType<FanucJointLab>();
        Require(lab!=null && lab.IsReady,"FANUC is not ready.");
        var token=lab.destroyCancellationToken;
        var hashes=SourcePaths.ToDictionary(p=>p,Hash);
        var targets=new[] { new[]{20f,-20f,15f,-15f,10f,-10f}, new[]{-20f,15f,-10f,20f,-15f,15f},
            new[]{30f,-25f,20f,-30f,25f,-20f}, new float[6] };
        int samples=0, arrivals=0;
        float maxPosition=0,maxRotation=0;
        FanucJointLab.FkDiagnostics Read()
        {
            Require(lab.TryReadFkDiagnostics(out var d,out var error),"Diagnostic failure: "+error);
            Require(d.PositionErrorMetres < PositionTolerance && d.RotationErrorDegrees < RotationTolerance,"Moving FK/actual TCP disagreement.");
            maxPosition=Mathf.Max(maxPosition,d.PositionErrorMetres); maxRotation=Mathf.Max(maxRotation,d.RotationErrorDegrees);
            samples++;
            return d;
        }
        void Move(float[] q) { for(int i=0;i<6;i++) Require(lab.MoveJoint(i,q[i]),"Valid joint target was rejected."); }
        try
        {
            lab.HomeAll();
            await Until(()=>At(lab,new float[6]),token);
            Move(targets[0]);
            await Until(()=>lab.Joints[0].CurrentPosition>=3f,token);
            lab.StopAll();
            var held=lab.Joints.Select(j=>j.CurrentPosition).ToArray();
            var before=Read();
            float fixedStart=Time.fixedTime;
            await Until(()=>Time.fixedTime-fixedStart>=Time.fixedDeltaTime*4,token);
            var after=Read();
            bool stopHeld=lab.Joints.Select((j,i)=>!j.IsMoving && j.CurrentPosition==held[i]).All(v=>v)
                && Vector3.Distance(before.ActualPositionMetres,after.ActualPositionMetres)<PositionTolerance
                && Angle(before.ActualRotation,after.ActualRotation)<RotationTolerance;
            Require(stopHeld,"Stopped joints or diagnostic pose drifted.");
            foreach(var target in targets)
            {
                Move(target);
                var watch=Stopwatch.StartNew();
                while(!At(lab,target))
                {
                    token.ThrowIfCancellationRequested();
                    Require(!EditorApplication.isPaused && watch.Elapsed.TotalSeconds<10,"Moving diagnostic check timed out or paused.");
                    Read();
                    await Task.Delay(40,token);
                }
                Read(); arrivals++;
            }
            bool unchanged=hashes.All(p=>Hash(p.Key)==p.Value);
            Require(unchanged,"A source asset changed during moving verification.");
            var report=new { passed=true,utc=DateTime.UtcNow.ToString("o"),actualTargetsVerified=arrivals,diagnosticSamples=samples,
                maxPositionErrorMetres=maxPosition,maxRotationErrorDegrees=maxRotation,stopHeld,
                sourceAssetsUnchanged=unchanged,sourceHashes=hashes,manualClicksPerformed=false,
                scope="Four API-driven targets including home; separate from the 1200 static comparisons, not 100 motor cycles" };
            Save("full-play-verification.json",report);
            return new {report.passed,report.actualTargetsVerified,report.diagnosticSamples,report.maxPositionErrorMetres,
                report.maxRotationErrorDegrees,report.stopHeld,report.sourceAssetsUnchanged,report.manualClicksPerformed};
        }
        finally { if(lab!=null && Application.isPlaying) lab.StopAll(); }
    }

    private static List<float[]> Poses(FanucJointLab lab)
    {
        var poses=new List<float[]> { new float[6] };
        for(int i=0;i<6;i++)
            foreach(float value in new[]{lab.Joints[i].MinimumDegrees,lab.Joints[i].MaximumDegrees})
            { var q=new float[6]; q[i]=value; poses.Add(q); }
        poses.Add(lab.Joints.Select(j=>j.MinimumDegrees).ToArray());
        poses.Add(lab.Joints.Select(j=>j.MaximumDegrees).ToArray());
        poses.Add(lab.Joints.Select((j,i)=>i%2==0?j.MinimumDegrees:j.MaximumDegrees).ToArray());
        poses.Add(lab.Joints.Select((j,i)=>i%2==0?j.MaximumDegrees:j.MinimumDegrees).ToArray());
        poses.Add(Enumerable.Repeat(.0001f,6).ToArray());
        poses.Add(Enumerable.Repeat(-.0001f,6).ToArray());
        poses.Add(new[]{25f,-30f,20f,-35f,15f,40f});
        poses.Add(new[]{-40f,15f,-25f,30f,-35f,-20f});
        var random=new System.Random(Seed);
        while(poses.Count<100)
            poses.Add(lab.Joints.Select(j=>j.MinimumDegrees+(j.MaximumDegrees-j.MinimumDegrees)*(float)random.NextDouble()).ToArray());
        Require(poses.All(q=>q.Select((v,i)=>v>=lab.Joints[i].MinimumDegrees && v<=lab.Joints[i].MaximumDegrees).All(v=>v)),"Pose outside simulation limits.");
        Require(poses.Select(q=>JsonConvert.SerializeObject(q)).Distinct().Count()==100,"Expected 100 distinct poses.");
        return poses;
    }
    private static bool At(FanucJointLab lab,float[] q)=>lab.Joints.Select((j,i)=>!j.IsMoving && Mathf.Abs(j.CurrentPosition-q[i])<.001f).All(v=>v);
    private static async Task Until(Func<bool> condition,CancellationToken token)
    {
        var watch=Stopwatch.StartNew();
        while(!condition())
        {
            token.ThrowIfCancellationRequested();
            Require(!EditorApplication.isPaused && watch.Elapsed.TotalSeconds<10,"Wait timed out or paused.");
            await Task.Delay(40,token);
        }
    }
    private static float Angle(Quaternion a,Quaternion b)
    {
        var d=Quaternion.Inverse(a)*b;
        return (float)(2*Math.Atan2(Math.Sqrt((double)d.x*d.x+(double)d.y*d.y+(double)d.z*d.z),Math.Abs(d.w))*180/Math.PI);
    }
    private static float[] V(Vector3 p)=>new[]{p.x,p.y,p.z};
    private static float[] Q(Quaternion q)=>new[]{q.x,q.y,q.z,q.w};
    private static string Hash(string path) { using(var sha=SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))); }
    private static void Save(string name,object report) { Directory.CreateDirectory("Logs/LW07B"); File.WriteAllText("Logs/LW07B/"+name,JsonConvert.SerializeObject(report,Formatting.Indented)); }
    private static void Require(bool condition,string message) { if(!condition) throw new InvalidOperationException(message); }
}
