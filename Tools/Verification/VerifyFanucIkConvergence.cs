using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading.Tasks;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using IK = LineWatch.FanucInverseKinematics;

public static class VerifyFanucIkConvergence
{
    private const int RandomSeed = 20261115;
    public static async Task<object> Run(string label = "baseline")
    {
        Require(!EditorApplication.isPlaying,"Run pure convergence checks in Edit Mode.");
        Require(label=="baseline" || label=="final","Use baseline or final label.");
        const string prefabPath="Assets/03.Prefabs/FanucRobot_Work.prefab";
        var paths=new[]{prefabPath,"Assets/01.Scenes/RobotLab.unity","Assets/03.Prefabs/ScaraRobot_Work.prefab",
            "ProjectSettings/ProjectSettings.asset","Assets/02.Scripts/LineWatch/FanucInverseKinematics.cs"};
        var hashes=paths.ToDictionary(p=>p,Hash);
        var root=PrefabUtility.LoadPrefabContents(prefabPath);
        try
        {
            var lab=root.GetComponent<FanucJointLab>(); var original=lab.Geometry;
            var lower=lab.Joints.Select(j=>j.MinimumDegrees).ToArray(); var upper=lab.Joints.Select(j=>j.MaximumDegrees).ToArray();
            var homes=lab.Joints.Select(j=>j.transform.localRotation).ToArray();
            var poses=Poses(lower,upper);
            var seeds=new[]{new float[6],new[]{10f,-15f,25f,15f,20f,-10f},new[]{-10f,15f,-25f,-15f,-20f,10f}};
            var budget=new IK.Budget(240,50);
            string dataset=JsonConvert.SerializeObject(new{poses,seeds,lower,upper,budget});
            var placements=new[]{new{name="Original",p=Vector3.zero,r=Quaternion.identity},
                new{name="Translated/yawed",p=new Vector3(2.1f,1.2f,-3),r=Quaternion.Euler(0,37,0)},
                new{name="Translated/tilted",p=new Vector3(-4.2f,.8f,2.3f),r=Quaternion.Euler(20,47,-15)}};
            var tools=new[]{new{name="Original",p=original.ToolPositionMetres,r=original.ToolRotation},
                new{name="3D position",p=original.ToolPositionMetres+new Vector3(-.02f,.015f,.025f),r=original.ToolRotation},
                new{name="3D rotation",p=original.ToolPositionMetres,r=original.ToolRotation*Quaternion.Euler(25,-35,15)},
                new{name="Combined",p=original.ToolPositionMetres+new Vector3(-.03f,-.02f,.035f),r=original.ToolRotation*Quaternion.Euler(-20,30,40)}};
            var rows=new List<Row>();
            float maxPosition=0,maxRotation=0,maxWorldPosition=0,maxWorldRotation=0;
            void Check(IK.Result result,Vector3 target,Quaternion rotation,Vector3 worldTarget,Quaternion worldRotation,FanucKinematicGeometry geometry)
            {
                Require(result.Iterations<=240,"Shared iteration budget exceeded.");
                if(!result.Succeeded)
                {
                    Require(result.JointDegrees.Count==0 && result.Reason!=IK.Failure.OutsideReachBound,"Known feasible goal was falsely called outside reach or exposed a failed solution.");
                    return;
                }
                Require(result.JointDegrees.Count==6 && result.JointDegrees.Select((q,i)=>q>=lower[i] && q<=upper[i]).All(v=>v),"Successful solution violated limits.");
                var fk=FanucForwardKinematics.Calculate(result.JointDegrees,geometry);
                float pe=Vector3.Distance(fk.PositionMetres,target),re=Angle(fk.Rotation,rotation);
                Require(pe<=IK.PositionToleranceMetres+.000001f && re<=IK.RotationToleranceDegrees+.0001f,"False successful FK residual.");
                for(int i=0;i<6;i++)lab.Joints[i].transform.localRotation=homes[i]*Quaternion.AngleAxis(result.JointDegrees[i],lab.Joints[i].LocalRotationAxis);
                float mp=Vector3.Distance(target,lab.RobotFrame.InverseTransformPoint(lab.Tcp.position));
                float mr=Angle(rotation,Quaternion.Inverse(lab.RobotFrame.rotation)*lab.Tcp.rotation);
                float wp=Vector3.Distance(worldTarget,lab.Tcp.position),wr=Angle(worldRotation,lab.Tcp.rotation);
                Require(mp<=IK.PositionToleranceMetres+.000002f && wp<=IK.PositionToleranceMetres+.000002f &&
                    mr<=IK.RotationToleranceDegrees+.0001f && wr<=IK.RotationToleranceDegrees+.0001f,"Successful solution disagrees with isolated model.");
                maxPosition=Mathf.Max(maxPosition,mp);maxRotation=Mathf.Max(maxRotation,mr);
                maxWorldPosition=Mathf.Max(maxWorldPosition,wp);maxWorldRotation=Mathf.Max(maxWorldRotation,wr);
            }
            for(int ti=0;ti<tools.Length;ti++)
            {
                var tool=tools[ti];lab.Tcp.SetLocalPositionAndRotation(tool.p,tool.r);
                var geometry=new FanucKinematicGeometry(original.Joints.ToArray(),tool.p,tool.r);
                for(int pi=0;pi<placements.Length;pi++)
                {
                    var placement=placements[pi];root.transform.SetPositionAndRotation(placement.p,placement.r);
                    for(int gi=0;gi<poses.Count;gi++)
                    {
                        var goal=FanucForwardKinematics.Calculate(poses[gi],geometry);
                        var wp=lab.RobotFrame.TransformPoint(goal.PositionMetres);var wr=lab.RobotFrame.rotation*goal.Rotation;
                        var local=lab.RobotFrame.InverseTransformPoint(wp);var lr=Quaternion.Inverse(lab.RobotFrame.rotation)*wr;
                        if(ti==0 && pi==0)
                            for(int si=0;si<seeds.Length;si++)
                            {
                                var single=IK.Solve(local,lr,geometry,lower,upper,new[]{seeds[si]},budget);
                                Check(single,local,lr,wp,wr,geometry);
                                rows.Add(new Row("single",gi,ti,pi,si,single));
                            }
                        var result=IK.Solve(local,lr,geometry,lower,upper,seeds,budget);
                        Check(result,local,lr,wp,wr,geometry);
                        rows.Add(new Row("batch",gi,ti,pi,-1,result));
                        if(gi%10==9){await Task.Delay(1);Require(!EditorApplication.isPlaying,"Play interrupted isolated validation.");}
                    }
                }
            }
            Require(dataset==JsonConvert.SerializeObject(new{poses,seeds,lower,upper,budget}),"Inputs were mutated.");
            bool unchanged=hashes.All(p=>Hash(p.Key)==p.Value);Require(unchanged,"Source asset changed.");
            var singles=rows.Where(r=>r.mode=="single").ToArray();var batches=rows.Where(r=>r.mode=="batch").ToArray();
            var primary=batches.Where(r=>r.toolIndex==0 && r.placementIndex==0).ToArray();
            var report=new{validationContractPassed=true,label,utc=DateTime.UtcNow.ToString("o"),randomSeed=RandomSeed,
                distinctJointPoses=poses.Count,placements=placements.Length,tools=tools.Length,
                singleSeedAttempts=singles.Length,singleSeedSuccesses=singles.Count(r=>r.succeeded),
                primaryBatchSuccesses=primary.Count(r=>r.succeeded),primaryBatchTargets=primary.Length,
                batchAttempts=batches.Length,batchSuccesses=batches.Count(r=>r.succeeded),allBatchGoalsSolved=batches.All(r=>r.succeeded),
                primaryFallbacks=primary.Count(r=>r.seedsTried>1),batchFallbacks=batches.Count(r=>r.seedsTried>1),
                alreadyWithinTolerancePrimaryIndices=primary.Where(r=>r.succeeded && r.iterations==0).Select(r=>r.goalIndex).ToArray(),
                nonControlSingleAttempts=singles.Count(r=>r.goalIndex!=0 && r.goalIndex!=17 && r.goalIndex!=18),
                nonControlSingleSuccesses=singles.Count(r=>r.succeeded && r.goalIndex!=0 && r.goalIndex!=17 && r.goalIndex!=18),
                maxModelPositionErrorMetres=maxPosition,maxModelRotationErrorDegrees=maxRotation,
                maxWorldPositionErrorMetres=maxWorldPosition,maxWorldRotationErrorDegrees=maxWorldRotation,
                timing=Stats(rows),batchTiming=Stats(batches),
                singleFailures=singles.Where(r=>!r.succeeded).GroupBy(r=>r.reason).Select(g=>new{reason=g.Key,count=g.Count()}).ToArray(),
                seedStats=singles.GroupBy(r=>r.initialSeedIndex).Select(g=>new{initialSeedIndex=g.Key,attempts=g.Count(),successes=g.Count(r=>r.succeeded),timing=Stats(g)}).ToArray(),
                groups=batches.GroupBy(r=>new{r.toolIndex,r.placementIndex}).Select(g=>new{g.Key.toolIndex,g.Key.placementIndex,
                    attempts=g.Count(),successes=g.Count(r=>r.succeeded),failures=g.Where(r=>!r.succeeded).GroupBy(r=>r.reason).Select(f=>new{reason=f.Key,count=f.Count()}).ToArray(),timing=Stats(g)}).ToArray(),
                lower,upper,initialSeeds=seeds,jointPoses=poses,maximumIterations=240,maximumMilliseconds=50,
                datasetHash=HashText(dataset),sourceHashes=hashes,sourceAssetsUnchanged=unchanged,rows,
                scope="100 fixed known-feasible targets; independent fixed seeds, no oracle seeds, no motion/UI or real robot accuracy claim"};
            Directory.CreateDirectory("Logs/LW07C");File.WriteAllText("Logs/LW07C/full-convergence-"+label+".json",JsonConvert.SerializeObject(report,Formatting.Indented));
            if(label=="final")Require(report.allBatchGoalsSolved && report.primaryBatchSuccesses==100,"Final acceptance requires all fixed batch goals to solve.");
            return new{report.validationContractPassed,report.singleSeedAttempts,report.singleSeedSuccesses,
                report.primaryBatchSuccesses,report.primaryBatchTargets,report.batchAttempts,report.batchSuccesses,
                report.allBatchGoalsSolved,report.maxModelPositionErrorMetres,report.maxModelRotationErrorDegrees,
                report.batchTiming,report.primaryFallbacks,report.batchFallbacks,report.singleFailures,
                report.alreadyWithinTolerancePrimaryIndices,report.nonControlSingleAttempts,report.nonControlSingleSuccesses,
                report.seedStats,failedPrimary=primary.Where(r=>!r.succeeded).Select(r=>new{r.goalIndex,r.reason,r.iterations,r.positionErrorMetres,r.rotationErrorDegrees}).ToArray(),
                report.datasetHash,report.sourceAssetsUnchanged};
        }
        finally{PrefabUtility.UnloadPrefabContents(root);}
    }
    private sealed class Row
    {
        public string mode;public int goalIndex,toolIndex,placementIndex,initialSeedIndex,selectedSeedIndex,iterations,seedsTried;
        public bool succeeded;public string reason;public double milliseconds;public float? positionErrorMetres,rotationErrorDegrees;
        public float[] solutionDegrees;
        public Row(string mode,int goal,int tool,int placement,int seed,IK.Result result)
        {this.mode=mode;goalIndex=goal;toolIndex=tool;placementIndex=placement;initialSeedIndex=seed;
            succeeded=result.Succeeded;reason=result.Reason.ToString();selectedSeedIndex=result.SeedIndex;iterations=result.Iterations;
            seedsTried=result.SeedsTried;milliseconds=result.ElapsedMilliseconds;positionErrorMetres=result.PositionErrorMetres;rotationErrorDegrees=result.RotationErrorDegrees;
            solutionDegrees=result.JointDegrees.ToArray();}
    }
    private static object Stats(IEnumerable<Row> source)
    {var rows=source.ToArray();var times=rows.Select(r=>r.milliseconds).OrderBy(v=>v).ToArray();return new{meanMilliseconds=times.Average(),
        p95Milliseconds=times[(int)Math.Ceiling(times.Length*.95)-1],maximumMilliseconds=times.Last(),
        meanIterations=rows.Average(r=>r.iterations),maximumIterations=rows.Max(r=>r.iterations)};}
    private static List<float[]> Poses(float[] lower,float[] upper)
    {
        var result=new List<float[]>{new float[6]};
        for(int i=0;i<6;i++)foreach(float value in new[]{lower[i],upper[i]}){var q=new float[6];q[i]=value;result.Add(q);}
        result.Add((float[])lower.Clone());result.Add((float[])upper.Clone());
        result.Add(Enumerable.Range(0,6).Select(i=>i%2==0?lower[i]:upper[i]).ToArray());
        result.Add(Enumerable.Range(0,6).Select(i=>i%2==0?upper[i]:lower[i]).ToArray());
        result.Add(Enumerable.Repeat(.0001f,6).ToArray());result.Add(Enumerable.Repeat(-.0001f,6).ToArray());
        result.Add(new[]{25f,-30f,20f,-35f,15f,40f});result.Add(new[]{-40f,15f,-25f,30f,-35f,-20f});
        var random=new System.Random(RandomSeed);
        while(result.Count<100)result.Add(Enumerable.Range(0,6).Select(i=>lower[i]+(upper[i]-lower[i])*(float)random.NextDouble()).ToArray());
        Require(result.Select(q=>JsonConvert.SerializeObject(q)).Distinct().Count()==100,"Duplicate target poses.");
        Require(result.All(q=>q.Select((v,i)=>v>=lower[i] && v<=upper[i]).All(v=>v)),"Known feasible poses must stay within the saved limits.");return result;
    }
    private static float Angle(Quaternion a,Quaternion b){var d=Quaternion.Inverse(a)*b;return(float)(2*Math.Atan2(Math.Sqrt((double)d.x*d.x+(double)d.y*d.y+(double)d.z*d.z),Math.Abs(d.w))*180/Math.PI);}
    private static string Hash(string path){using(var sha=SHA256.Create())return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path)));}
    private static string HashText(string text){using(var sha=SHA256.Create())return BitConverter.ToString(sha.ComputeHash(System.Text.Encoding.UTF8.GetBytes(text)));}
    private static void Require(bool condition,string message){if(!condition)throw new InvalidOperationException(message);}
}
