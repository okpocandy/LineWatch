using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using IK = LineWatch.FanucInverseKinematics;

public static class VerifyFanucInverseKinematics
{
    public static object Run()
    {
        Require(!EditorApplication.isPlaying,"Run pure IK checks in Edit Mode.");
        const string prefabPath="Assets/03.Prefabs/FanucRobot_Work.prefab";
        var paths=new[]{prefabPath,"Assets/01.Scenes/RobotLab.unity","Assets/03.Prefabs/ScaraRobot_Work.prefab","ProjectSettings/ProjectSettings.asset"};
        var hashes=paths.ToDictionary(p=>p,Hash);
        var root=PrefabUtility.LoadPrefabContents(prefabPath);
        try
        {
            var lab=root.GetComponent<FanucJointLab>();
            var geometry=lab.Geometry;
            var lower=lab.Joints.Select(j=>j.MinimumDegrees).ToArray(); var upper=lab.Joints.Select(j=>j.MaximumDegrees).ToArray();
            var homes=lab.Joints.Select(j=>j.transform.localRotation).ToArray();
            var known=new[]{new float[6],new[]{1f,-1f,2f,-1f,1f,2f},new[]{15f,-20f,25f,-15f,20f,10f},
                new[]{-20f,15f,-25f,20f,-15f,-10f},new[]{44f,-40f,35f,-30f,25f,-40f},new[]{30f,-10f,20f,35f,-30f,15f}};
            var seeds=new[]{new float[6],new[]{10f,-15f,25f,15f,20f,-10f},new[]{-10f,15f,-25f,-15f,-20f,10f}};
            string inputs=JsonConvert.SerializeObject(new{known,seeds,lower,upper,geometry=JsonUtility.ToJson(geometry)});
            var singles=new List<object>(); var batches=new List<object>(); var results=new List<IK.Result>();
            int singleSuccess=0,batchSuccess=0;
            float maxFkPosition=0,maxFkRotation=0,maxModelPosition=0,maxModelRotation=0;
            void Check(IK.Result result,FanucForwardKinematics.TcpPose goal)
            {
                Require(result.Iterations<=240,"Global iteration budget exceeded.");
                if(!result.Succeeded) {Require(result.JointDegrees.Count==0,"Failure exposed an executable joint solution."); return;}
                Require(result.JointDegrees.Count==6 && result.JointDegrees.Select((q,i)=>q>=lower[i] && q<=upper[i]).All(v=>v),"Successful solution violates limits.");
                var fk=FanucForwardKinematics.Calculate(result.JointDegrees,geometry);
                float pe=Vector3.Distance(fk.PositionMetres,goal.PositionMetres),re=Angle(fk.Rotation,goal.Rotation);
                Require(pe<=IK.PositionToleranceMetres+.000001f && re<=IK.RotationToleranceDegrees+.0001f,"Successful IK residual is invalid.");
                Require(Mathf.Abs(pe-result.PositionErrorMetres.Value)<.000001f && Mathf.Abs(re-result.RotationErrorDegrees.Value)<.0001f,"Reported residual disagrees with returned joints.");
                for(int i=0;i<6;i++) lab.Joints[i].transform.localRotation=homes[i]*Quaternion.AngleAxis(result.JointDegrees[i],lab.Joints[i].LocalRotationAxis);
                float mp=Vector3.Distance(goal.PositionMetres,lab.RobotFrame.InverseTransformPoint(lab.Tcp.position));
                float mr=Angle(goal.Rotation,Quaternion.Inverse(lab.RobotFrame.rotation)*lab.Tcp.rotation);
                Require(mp<=IK.PositionToleranceMetres+.000002f && mr<=IK.RotationToleranceDegrees+.0001f,"Successful IK disagrees with isolated model.");
                maxFkPosition=Mathf.Max(maxFkPosition,pe); maxFkRotation=Mathf.Max(maxFkRotation,re);
                maxModelPosition=Mathf.Max(maxModelPosition,mp); maxModelRotation=Mathf.Max(maxModelRotation,mr);
            }
            for(int goalIndex=0;goalIndex<known.Length;goalIndex++)
            {
                var goal=FanucForwardKinematics.Calculate(known[goalIndex],geometry);
                for(int seedIndex=0;seedIndex<seeds.Length;seedIndex++)
                {
                    var result=IK.Solve(goal.PositionMetres,goal.Rotation,geometry,lower,upper,new[]{seeds[seedIndex]});
                    Check(result,goal); results.Add(result); if(result.Succeeded)singleSuccess++;
                    singles.Add(Row(goalIndex,seedIndex,result));
                }
                var batch=IK.Solve(goal.PositionMetres,goal.Rotation,geometry,lower,upper,seeds);
                Check(batch,goal); results.Add(batch); batches.Add(Row(goalIndex,-1,batch)); if(batch.Succeeded)batchSuccess++;
            }
            var ordinary=FanucForwardKinematics.Calculate(known[2],geometry);
            var neg=new Quaternion(-ordinary.Rotation.x,-ordinary.Rotation.y,-ordinary.Rotation.z,-ordinary.Rotation.w);
            var signEquivalent=IK.Solve(ordinary.PositionMetres,neg,geometry,lower,upper,seeds);
            Check(signEquivalent,ordinary);
            Require(signEquivalent.Succeeded,"Equivalent quaternion sign failed.");
            var negativeGoal=FanucForwardKinematics.Calculate(known[3],geometry);
            var actualFallback=IK.Solve(negativeGoal.PositionMetres,negativeGoal.Rotation,geometry,lower,upper,new[]{seeds[1],seeds[0],seeds[2]});
            Check(actualFallback,negativeGoal); results.Add(actualFallback);
            Require(actualFallback.Succeeded && actualFallback.SeedIndex==1 && actualFallback.SeedsTried==2,
                "The poor primary seed should recover with the alternate home seed within one shared budget.");
            int failureChecks=0;
            var failures=new List<object>();
            void F(string name,IK.Result result,IK.Failure expected)
            {
                Require(!result.Succeeded && result.Reason==expected && result.JointDegrees.Count==0,"Unexpected failure contract: "+name+" -> "+result.Reason);
                failureChecks++; failures.Add(new{name,reason=result.Reason.ToString(),result.Iterations,result.SeedsTried,result.ElapsedMilliseconds});
            }
            F("NaN position",IK.Solve(new Vector3(float.NaN,0,0),ordinary.Rotation,geometry,lower,upper,seeds),IK.Failure.InvalidInput);
            F("zero rotation",IK.Solve(ordinary.PositionMetres,default,geometry,lower,upper,seeds),IK.Failure.InvalidInput);
            F("nonunit rotation",IK.Solve(ordinary.PositionMetres,new Quaternion(0,0,0,2),geometry,lower,upper,seeds),IK.Failure.InvalidInput);
            F("infinite rotation",IK.Solve(ordinary.PositionMetres,new Quaternion(0,float.PositiveInfinity,0,1),geometry,lower,upper,seeds),IK.Failure.InvalidInput);
            F("null geometry",IK.Solve(ordinary.PositionMetres,ordinary.Rotation,null,lower,upper,seeds),IK.Failure.InvalidGeometry);
            F("missing homes",IK.Solve(ordinary.PositionMetres,ordinary.Rotation,new FanucKinematicGeometry(),lower,upper,seeds),IK.Failure.InvalidGeometry);
            F("missing seeds",IK.Solve(ordinary.PositionMetres,ordinary.Rotation,geometry,lower,upper,Array.Empty<float[]>()),IK.Failure.InvalidInput);
            F("short seed",IK.Solve(ordinary.PositionMetres,ordinary.Rotation,geometry,lower,upper,new[]{new float[5]}),IK.Failure.InvalidSeed);
            var badSeed=new float[6]; badSeed[0]=float.NaN;
            F("NaN seed",IK.Solve(ordinary.PositionMetres,ordinary.Rotation,geometry,lower,upper,new[]{badSeed}),IK.Failure.InvalidSeed);
            badSeed=new float[6]; badSeed[0]=46;
            F("seed outside limits",IK.Solve(ordinary.PositionMetres,ordinary.Rotation,geometry,lower,upper,new[]{badSeed}),IK.Failure.InvalidSeed);
            F("short limits",IK.Solve(ordinary.PositionMetres,ordinary.Rotation,geometry,new float[5],upper,seeds),IK.Failure.InvalidInput);
            var badLower=(float[])lower.Clone(); badLower[0]=50;
            F("inverted limits",IK.Solve(ordinary.PositionMetres,ordinary.Rotation,geometry,badLower,upper,seeds),IK.Failure.InvalidInput);
            F("invalid budget",IK.Solve(ordinary.PositionMetres,ordinary.Rotation,geometry,lower,upper,seeds,new IK.Budget(-1,50)),IK.Failure.InvalidInput);
            F("invalid deadline",IK.Solve(ordinary.PositionMetres,ordinary.Rotation,geometry,lower,upper,seeds,new IK.Budget(240,0)),IK.Failure.InvalidInput);
            F("outside proven bound",IK.Solve(new Vector3(3,0,0),ordinary.Rotation,geometry,lower,upper,seeds),IK.Failure.OutsideReachBound);
            F("zero iteration budget",IK.Solve(ordinary.PositionMetres,ordinary.Rotation,geometry,lower,upper,new[]{seeds[0]},new IK.Budget(0,50)),IK.Failure.IterationLimit);
            var boundaryGoal=FanucForwardKinematics.Calculate(known[4],geometry);
            var sharedBudget=IK.Solve(boundaryGoal.PositionMetres,boundaryGoal.Rotation,geometry,lower,upper,seeds,new IK.Budget(3,50));
            F("shared iteration budget",sharedBudget,IK.Failure.IterationLimit);
            Require(sharedBudget.Iterations==3 && sharedBudget.SeedsTried==3,"Seed attempts did not share the global iteration budget.");
            F("deadline",IK.Solve(ordinary.PositionMetres,ordinary.Rotation,geometry,lower,upper,seeds,new IK.Budget(240,1e-9)),IK.Failure.TimeLimit);
            F("fixed joint constraints",IK.Solve(ordinary.PositionMetres,ordinary.Rotation,geometry,new float[6],new float[6],new[]{new float[6]}),IK.Failure.JointLimitStall);
            var home=FanucForwardKinematics.Calculate(new float[6],geometry);
            var exact=IK.Solve(home.PositionMetres,home.Rotation,geometry,lower,upper,new[]{new float[6]},new IK.Budget(0,50));
            Require(exact.Succeeded && exact.Iterations==0,"An already-valid singular pose should not require an iteration.");
            var wrap=WrapCases();
            bool inputsUnchanged=inputs==JsonConvert.SerializeObject(new{known,seeds,lower,upper,geometry=JsonUtility.ToJson(geometry)});
            bool unchanged=hashes.All(p=>Hash(p.Key)==p.Value);
            Require(inputsUnchanged && unchanged,"IK mutated an input or source asset.");
            var report=new{passed=batchSuccess==6,utc=DateTime.UtcNow.ToString("o"),knownTargets=known.Length,
                singleSeedAttempts=singles.Count,singleSeedSuccesses=singleSuccess,batchSuccesses=batchSuccess,
                maximumIterations=240,maximumMilliseconds=50,positionToleranceMetres=IK.PositionToleranceMetres,
                rotationToleranceDegrees=IK.RotationToleranceDegrees,maxFkPositionErrorMetres=maxFkPosition,maxFkRotationErrorDegrees=maxFkRotation,
                maxModelPositionErrorMetres=maxModelPosition,maxModelRotationErrorDegrees=maxModelRotation,
                baselineSolveCount=results.Count,maximumBaselineSolveMilliseconds=results.Max(r=>r.ElapsedMilliseconds),failureChecks,
                fallback=Row(3,1,actualFallback),sharedIterationBudgetPassed=true,
                quaternionSignPassed=signEquivalent.Succeeded,exactSingularGoalPassed=exact.Succeeded,wrap,
                inputsUnchanged,sourceAssetsUnchanged=unchanged,sourceHashes=hashes,singles,batches,failures,
                scope="LW-07C process 2: six baseline targets/three independent seeds, pure solver and isolated model; no broad convergence campaign or motion/UI integration"};
            Directory.CreateDirectory("Logs/LW07C");
            File.WriteAllText("Logs/LW07C/inverse-kinematics-verification.json",JsonConvert.SerializeObject(report,Formatting.Indented));
            return new{report.passed,report.batchSuccesses,report.singleSeedAttempts,report.singleSeedSuccesses,report.maxFkPositionErrorMetres,report.maxFkRotationErrorDegrees,
                report.maxModelPositionErrorMetres,report.maxModelRotationErrorDegrees,report.maximumBaselineSolveMilliseconds,report.failureChecks,
                report.fallback,report.sharedIterationBudgetPassed,report.quaternionSignPassed,report.exactSingularGoalPassed,
                report.wrap,report.inputsUnchanged,report.sourceAssetsUnchanged};
        }
        finally{PrefabUtility.UnloadPrefabContents(root);}
    }

    private static object Row(int goalIndex,int seedIndex,IK.Result result)=>new{goalIndex,initialSeedIndex=seedIndex,succeeded=result.Succeeded,
        reason=result.Reason.ToString(),selectedSeedIndex=result.SeedIndex,result.SeedsTried,result.Iterations,result.ElapsedMilliseconds,
        result.PositionErrorMetres,result.RotationErrorDegrees,jointDegrees=result.JointDegrees};
    private static object WrapCases()
    {
        var homes=Enumerable.Range(0,6).Select(_=>new FanucKinematicGeometry.JointHome(Vector3.zero,Quaternion.identity,Vector3.up)).ToArray();
        var g=new FanucKinematicGeometry(homes,new Vector3(0,0,.25f),Quaternion.identity);
        var lower=new[]{-180f,0,0,0,0,0}; var upper=new[]{180f,0,0,0,0,0};
        var known=new[]{179f,0,0,0,0,0}; var goal=FanucForwardKinematics.Calculate(known,g);
        var seeds=new[]{new[]{-179f,0,0,0,0,0},new float[6]};
        var result=IK.Solve(goal.PositionMetres,goal.Rotation,g,lower,upper,seeds);
        Require(result.Succeeded && result.SeedIndex==1,"Wrapped target should recover via alternate seed without crossing a joint limit.");
        void Check(IK.Result solved,FanucForwardKinematics.TcpPose target)
        {
            Require(solved.Succeeded && solved.JointDegrees.Select((q,i)=>q>=lower[i] && q<=upper[i]).All(v=>v),"Synthetic solution violated its limits.");
            var actual=FanucForwardKinematics.Calculate(solved.JointDegrees,g);
            Require(Vector3.Distance(actual.PositionMetres,target.PositionMetres)<=IK.PositionToleranceMetres+.000001f &&
                Angle(actual.Rotation,target.Rotation)<=IK.RotationToleranceDegrees+.0001f,"Synthetic rotation solution failed independent FK validation.");
        }
        Check(result,goal);
        var half=FanucForwardKinematics.Calculate(new[]{180f,0,0,0,0,0},g);
        var halfResult=IK.Solve(half.PositionMetres,half.Rotation,g,lower,upper,new[]{new float[6]});
        Require(halfResult.Succeeded,"Half-turn rotation failed.");
        Check(halfResult,half);
        return new{fallbackPassed=result.Succeeded,usedSeedIndex=result.SeedIndex,
            primaryToSolutionJointChangeDegrees=Mathf.Abs(result.JointDegrees[0]-seeds[0][0]),halfTurnPassed=halfResult.Succeeded};
    }
    private static float Angle(Quaternion a,Quaternion b)
    {var d=Quaternion.Inverse(a)*b; return(float)(2*Math.Atan2(Math.Sqrt((double)d.x*d.x+(double)d.y*d.y+(double)d.z*d.z),Math.Abs(d.w))*180/Math.PI);}
    private static string Hash(string path){using(var sha=SHA256.Create())return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path)));}
    private static void Require(bool condition,string message){if(!condition)throw new InvalidOperationException(message);}
}
