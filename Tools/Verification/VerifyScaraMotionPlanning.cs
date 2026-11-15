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
using IK = LineWatch.ScaraInverseKinematics;
using Planner = LineWatch.ScaraMotionPlanner;

public static class VerifyScaraMotionPlanning
{
    private static IK.JointLimits Limits => new IK.JointLimits(J(-170f,-150f,0f,-180f),J(170f,150f,.15f,180f));
    private static Vector4 Speeds => new Vector4(30f,30f,.05f,30f);
    private static ScaraForwardKinematics.Geometry Geometry => new ScaraForwardKinematics.Geometry(.325f,.375f,.016783f,-.097783f,new Vector3(0f,-.034177f,0f));

    public static object Pure()
    {
        Require(!EditorApplication.isPlayingOrWillChangePlaymode,"Edit Mode required");
        int goals = 0, oldRejected = 0, jointRejected = 0, linearRejected = 0, denseSamples = 0;
        float maxLineError = 0f, maxYawError = 0f;
        var rows = new List<object>();

        void CheckPath(string name,IK.JointValues start,IK.JointValues[] targets,bool boundary)
        {
            int rejects = 0;
            var origin = Fk(start,Geometry);
            foreach (var nominal in targets)
            {
                var goal = Fk(nominal,Geometry);
                var solved = IK.Solve(goal.PositionMetres,goal.YawDegrees,Geometry,Limits);
                Require(solved.Success,"Known feasible endpoint rejected: "+name);
                var chosen = solved.Solutions.OrderBy(s => Cost(start,s.Joints)).First();
                bool local = Vector3.Distance(origin.PositionMetres,goal.PositionMetres) <= .005f && Mathf.Abs(Mathf.DeltaAngle(origin.YawDegrees,goal.YawDegrees)) <= 1f;
                if (local && (MaxRotation(start,chosen.Joints) > 15f || Mathf.Abs(start.Q2Degrees)>1f && Mathf.Abs(chosen.Joints.Q2Degrees)>1f && Mathf.Sign(start.Q2Degrees)!=Mathf.Sign(chosen.Joints.Q2Degrees))) oldRejected++;
                bool jointOk = Planner.Joint(start,chosen,Limits,Speeds,out var joint,out _);
                if (!jointOk) jointRejected++;
                Require(jointOk && joint.Nodes.Count == 2,"Joint-space path must accept bounded endpoints");
                CheckTiming(joint);
                bool linearOk = Planner.Linear(start,goal.PositionMetres,goal.YawDegrees,Geometry,Limits,Speeds,0,out var linear,out _);
                if (!linearOk) { linearRejected++; rejects++; }
                else CheckLine(linear,goal,Geometry,ref denseSamples,ref maxLineError,ref maxYawError);
                goals++;
            }
            Require(boundary ? rejects>0 : rejects==0,"Unexpected linear path rejection: "+name);
            rows.Add(new { name, goals = targets.Length, jointRejected = 0, linearRejected = rejects });
        }

        CheckPath("Ordinary small changes",J(20f,60f,.05f,-80f),Enumerable.Range(0,41).Select(i=>J(20f+i*.01f,60f-i*.01f,.05f+i*.000001f,-80f)).ToArray(),false);
        CheckPath("Positive extension",J(0f,2f,.05f,-2f),Enumerable.Range(0,41).Select(i=>J(0f,2f-i*.05f,.05f,-2f+i*.05f)).ToArray(),false);
        CheckPath("Negative extension",J(0f,-2f,.05f,2f),Enumerable.Range(0,41).Select(i=>J(0f,-2f+i*.05f,.05f,2f-i*.05f)).ToArray(),false);
        CheckPath("Yaw representation wrap",J(20f,60f,.05f,99.8f),Enumerable.Range(0,21).Select(i=>J(20f,60f,.05f,99.8f+i*.02f)).ToArray(),false);
        CheckPath("Wrist physical turn",J(0f,90f,.05f,179.9f),Enumerable.Range(0,21).Select(i=>J(0f,90f,.05f,179.9f+i*.01f)).ToArray(),true);
        CheckPath("Shoulder branch limit",J(169.9f,-60f,.05f,-109.9f),Enumerable.Range(0,21).Select(i=>J(169.9f+i*.01f,-60f,.05f,-109.9f-i*.01f)).ToArray(),true);
        Require(goals == 186 && oldRejected == 20 && jointRejected == 0 && linearRejected == 20,"Endpoint/path comparison changed");

        var start = J(0f,90f,.05f,-90f);
        var longGoal = Fk(J(20f,50f,.08f,-70f),Geometry);
        Require(Planner.Linear(start,longGoal.PositionMetres,longGoal.YawDegrees,Geometry,Limits,Speeds,0,out var longPlan,out _),"Legitimate large rotation rejected");
        Require(MaxRotation(start,longPlan.End.Joints) > 15f && longPlan.Nodes.Count > 2,"Adaptive subdivision not exercised");
        CheckLine(longPlan,longGoal,Geometry,ref denseSamples,ref maxLineError,ref maxYawError);
        var tool = new ScaraForwardKinematics.Geometry(.325f,.375f,.016783f,-.097783f,new Vector3(.02f,-.054177f,.01f),35f);
        var toolGoal = Fk(J(10f,80f,.06f,-90f),tool);
        Require(Planner.Linear(start,toolGoal.PositionMetres,toolGoal.YawDegrees,tool,Limits,Speeds,0,out var toolPlan,out _),"Eccentric tool path rejected");
        CheckLine(toolPlan,toolGoal,tool,ref denseSamples,ref maxLineError,ref maxYawError);

        var holeStart = J(-89.5f,0f,.05f,0f);
        var holeGoal = Fk(J(89.5f,0f,.05f,0f),Geometry);
        Require(IK.Solve(holeGoal.PositionMetres,holeGoal.YawDegrees,Geometry,Limits).Success,"Hole endpoint not reachable");
        Require(!Planner.Linear(holeStart,holeGoal.PositionMetres,holeGoal.YawDegrees,Geometry,Limits,Speeds,0,out _,out var holeFailure) && holeFailure==Planner.Failure.PathUnreachable,"Interior reach violation missed");
        Require(!Planner.Linear(start,longGoal.PositionMetres,longGoal.YawDegrees,Geometry,Limits,Speeds,-1,out _,out var branchFailure) && branchFailure==Planner.Failure.ConfigurationChangeRequired,"Linear motion changed elbow configuration");
        Require(!Planner.Linear(start,new Vector3(float.NaN,0f,0f),0f,Geometry,Limits,Speeds,0,out _,out _),"NaN accepted");
        Require(!Planner.Linear(start,longGoal.PositionMetres,0f,Geometry,Limits,Vector4.zero,0,out _,out _),"Invalid speed accepted");
        var report = new { passed = true, knownFeasibleEndpoints = goals, oldThresholdRejected = oldRejected,
            jointModeRejected = jointRejected, linearModeRejected = linearRejected, denseSamples,
            maxSampledLineErrorMetres = maxLineError, maxSampledYawErrorDegrees = maxYawError,
            largeRotationLinearPathAccepted = true, largeRotationPathNodes = longPlan.Nodes.Count,
            interiorUnreachableDetected = true, toolOffsetPathVerified = true,
            scope = "Joint feasibility is bounded joint-space interpolation, without collision claims. Comparison is this targeted dataset, not a production rejection rate.", rows };
        Write("planning-verification.json",report);
        return new { report.passed, report.knownFeasibleEndpoints, report.oldThresholdRejected, report.jointModeRejected,
            report.linearModeRejected, report.denseSamples, report.maxSampledLineErrorMetres, report.maxSampledYawErrorDegrees,
            report.largeRotationPathNodes, report.interiorUnreachableDetected };
    }

    public static async Task<object> Play()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused,"Unpaused RobotLab Play Mode required");
        var lab = UnityEngine.Object.FindFirstObjectByType<ScaraJointLab>();
        Require(lab!=null && lab.IsReady,"SCARA references required");
        var token = lab.destroyCancellationToken;
        float originalScale = Time.timeScale;
        bool enabled4 = lab.Axis4.enabled;
        string[] paths = { "Assets/01.Scenes/RobotLab.unity", "Assets/03.Prefabs/ScaraRobot_Work.prefab", "ProjectSettings/ProjectSettings.asset" };
        var hashes = paths.ToDictionary(path=>path,Hash);
        int arrivals = 0, rejected = 0, motionSamples = 0;
        float maxLine = 0f, maxRotarySpeed = 0f, maxSlideSpeed = 0f, maxJointProgressSpread = 0f;
        var rows = new List<object>();

        async Task SetPose(IK.JointValues j)
        {
            Require(lab.MoveJoint(0,j.Q1Degrees) && lab.MoveJoint(1,j.Q2Degrees) && lab.MoveJoint(2,j.D3Metres) && lab.MoveJoint(3,j.Q4Degrees),"Baseline setup failed");
            await Until(()=>AtRest(lab),token);
        }
        void Refuse(string name,Vector3 target,float yaw,ScaraJointLab.MotionMode mode)
        {
            var before = Targets(lab); var current = Current(lab); var moving = Flags(lab); bool active = lab.IsTrajectoryMoving;
            Require(!lab.TryMoveTcp(target,yaw,ScaraJointLab.IkSelection.Nearest,mode,out _,out var error) && !string.IsNullOrEmpty(error),name+": accepted");
            Require(before.Equals(Targets(lab)) && current.Equals(Current(lab)) && moving.SequenceEqual(Flags(lab)) && active==lab.IsTrajectoryMoving,name+": mutated existing trajectory");
            rows.Add(new { name, error, passed = true }); rejected++;
        }
        async Task Move(string name,ScaraForwardKinematics.TcpPose goal,ScaraJointLab.MotionMode mode,bool invalidDuring = false)
        {
            Vector3 origin = lab.ActualTcpPosition;
            Require(lab.TryMoveTcp(goal.PositionMetres,goal.YawDegrees,ScaraJointLab.IkSelection.Nearest,mode,out _,out var error),name+": "+error);
            var startingJoints = Current(lab);
            var endingJoints = Targets(lab);
            if(invalidDuring) Refuse("Invalid replacement preserves active plan",new Vector3(.8f,-.2f,0f),0f,ScaraJointLab.MotionMode.Linear);
            var previous = Current(lab);
            float fixedBefore = Time.fixedTime;
            var watch = Stopwatch.StartNew();
            while(lab.IsTrajectoryMoving)
            {
                token.ThrowIfCancellationRequested();
                Require(!EditorApplication.isPaused,"Verification interrupted by user pause");
                if(watch.Elapsed.TotalSeconds>15) throw new TimeoutException(name+": arrival timeout");
                await Task.Delay(12,token);
                var current = Current(lab); float now = Time.fixedTime, dt = now-fixedBefore;
                if(dt>0f)
                {
                    float rotation = Mathf.Max(Mathf.Abs(current.x-previous.x),Mathf.Abs(current.y-previous.y),Mathf.Abs(current.w-previous.w));
                    float slide = Mathf.Abs(current.z-previous.z);
                    Require(rotation<=30f*dt+.005f && slide<=.05f*dt+.000002f,"Actual configured speed exceeded");
                    maxRotarySpeed = Mathf.Max(maxRotarySpeed,rotation/dt); maxSlideSpeed = Mathf.Max(maxSlideSpeed,slide/dt);
                }
                if(mode==ScaraJointLab.MotionMode.Linear)
                {
                    var delta = goal.PositionMetres-origin;
                    float u = delta.sqrMagnitude>0f ? Mathf.Clamp01(Vector3.Dot(lab.ActualTcpPosition-origin,delta)/delta.sqrMagnitude) : 0f;
                    float deviation = Vector3.Distance(lab.ActualTcpPosition,Vector3.Lerp(origin,goal.PositionMetres,u));
                    maxLine = Mathf.Max(maxLine,deviation);
                    Require(deviation<=.00012f,"Actual execution departed from validated Cartesian line");
                }
                else
                {
                    float minProgress=1f,maxProgress=0f;
                    for(int axis=0;axis<4;axis++)
                        if(Mathf.Abs(endingJoints[axis]-startingJoints[axis])>0.000001f)
                        {
                            float progress=(current[axis]-startingJoints[axis])/(endingJoints[axis]-startingJoints[axis]);
                            minProgress=Mathf.Min(minProgress,progress); maxProgress=Mathf.Max(maxProgress,progress);
                        }
                    maxJointProgressSpread=Mathf.Max(maxJointProgressSpread,maxProgress-minProgress);
                    Require(maxProgress-minProgress<.0001f,"Joint axes did not share synchronized progress");
                }
                previous=current; fixedBefore=now; motionSamples++;
            }
            Require(AtRest(lab) && Vector3.Distance(lab.ActualTcpPosition,goal.PositionMetres)<.000002f &&
                Mathf.Abs(Mathf.DeltaAngle(lab.ActualTcpYawDegrees,goal.YawDegrees))<.001f,name+": actual arrival mismatch");
            arrivals++;
            rows.Add(new { name, passed = true, mode = mode.ToString(), positionErrorMetres = Vector3.Distance(lab.ActualTcpPosition,goal.PositionMetres) });
        }

        try
        {
            Time.timeScale=10f;
            await SetPose(J(0f,90f,.05f,-90f));
            await Move("Linear path with 40 degree elbow change",Fk(J(20f,50f,.08f,-70f),Geometry),ScaraJointLab.MotionMode.Linear,true);
            await Move("Nearby linear replacement",Fk(J(20.1f,49.9f,.0801f,-70f),Geometry),ScaraJointLab.MotionMode.Linear);

            await SetPose(J(169.9f,-60f,.05f,-109.9f));
            var boundary = Fk(J(170.05f,-60f,.05f,-110.05f),Geometry);
            Refuse("Linear shoulder configuration blocked",boundary.PositionMetres,boundary.YawDegrees,ScaraJointLab.MotionMode.Linear);
            var window=EditorWindow.GetWindow<ScaraJointLabWindow>(); window.RefreshLab(); var ui=window.rootVisualElement;
            Require(ui.Q<DropdownField>("scara-ik-motion").choices.Count==2,"Movement mode UI missing");
            ui.Q<FloatField>("scara-ik-x").value=boundary.PositionMetres.x*1000f;
            ui.Q<FloatField>("scara-ik-y").value=boundary.PositionMetres.y*1000f;
            ui.Q<FloatField>("scara-ik-z").value=boundary.PositionMetres.z*1000f;
            ui.Q<FloatField>("scara-ik-yaw").value=boundary.YawDegrees;
            ui.Q<DropdownField>("scara-ik-motion").value="직선 작업 이동";
            await Task.Delay(160,token);
            Require(!ui.Q<Button>("scara-ik-move").enabledInHierarchy && ui.Q<HelpBox>("scara-ik-feedback").text.Contains("직선"),"Linear rejection hidden by UI");
            ui.Q<DropdownField>("scara-ik-motion").value="관절 이동 (동기 도착)";
            await Task.Delay(120,token);
            Require(ui.Q<Button>("scara-ik-move").enabledInHierarchy,"Feasible joint reconfiguration still blocked");
            window.RefreshLab();
            await Move("Large bounded joint reconfiguration",boundary,ScaraJointLab.MotionMode.Joint);

            await SetPose(J(-89.5f,0f,.05f,0f));
            var hole=Fk(J(89.5f,0f,.05f,0f),Geometry);
            Require(IK.Solve(hole.PositionMetres,hole.YawDegrees,Geometry,Limits).Success,"Interior test endpoint invalid");
            Refuse("Reachable endpoint but unreachable line interior",hole.PositionMetres,hole.YawDegrees,ScaraJointLab.MotionMode.Linear);
            await Move("Joint route around unreachable line interior",hole,ScaraJointLab.MotionMode.Joint);

            await SetPose(J(20f,60f,.05f,-80f));
            var goal=Fk(J(40f,40f,.08f,-80f),Geometry);
            Require(lab.TryMoveTcp(goal.PositionMetres,goal.YawDegrees,ScaraJointLab.IkSelection.Nearest,ScaraJointLab.MotionMode.Linear,out _,out _),"Stop test request rejected");
            await Task.Delay(35,token); Require(lab.IsTrajectoryMoving,"Stop test not moving");
            lab.StopAll(); var held=Current(lab);
            await Task.Delay(140,token);
            Require(!lab.IsTrajectoryMoving && !Flags(lab).Any(v=>v) && held.Equals(Current(lab)),"Stop did not hold actual pose");

            await SetPose(J(0f,90f,.05f,-90f));
            var original=Fk(J(20f,50f,.08f,-70f),Geometry);
            Require(lab.TryMoveTcp(original.PositionMetres,original.YawDegrees,ScaraJointLab.IkSelection.Nearest,ScaraJointLab.MotionMode.Linear,out _,out _),"Replacement test start failed");
            await Task.Delay(25,token);
            await Move("Replan from actual in-flight joint state",Fk(J(20.05f,49.95f,.08001f,-70f),Geometry),ScaraJointLab.MotionMode.Linear);

            Require(lab.TryMoveTcp(new Vector3(.4f,-.2f,-.3f),0f,ScaraJointLab.IkSelection.Nearest,out _,out _),"Manual override setup failed");
            Require(lab.MoveJoint(0,5f) && !lab.IsTrajectoryMoving,"Manual command did not cancel coordinated plan");
            await Until(()=>!lab.Axis1.IsMoving,token);
            Require(!lab.Axis2.IsMoving && !lab.Axis3.IsMoving && !lab.Axis4.IsMoving,"Other axes resumed an obsolete plan");
            lab.HomeAll(); await Until(()=>AtRest(lab),token);
            Require(lab.TryMoveTcp(new Vector3(.4f,-.2f,-.3f),0f,ScaraJointLab.IkSelection.Nearest,out _,out _),"Disabled-axis setup failed");
            lab.Axis4.enabled=false;
            await Until(()=>!lab.IsTrajectoryMoving,token);
            Require(!Flags(lab).Any(v=>v) && !string.IsNullOrEmpty(lab.LastMotionError),"Disabled axis did not cancel the plan");
            lab.Axis4.enabled=enabled4;
            lab.HomeAll(); await Until(()=>AtRest(lab),token);
            bool unchanged=paths.All(path=>hashes[path]==Hash(path)); Require(unchanged,"Source assets changed");
            var report=new { passed=true, actualArrivals=arrivals, rejectedRequestsPreserved=rejected, motionSamples,
                maxActualLineDeviationMetres=maxLine, maxObservedRotarySpeedDegreesPerSecond=maxRotarySpeed,
                maxObservedSlideSpeedMetresPerSecond=maxSlideSpeed, maxJointProgressSpread, stopHeld=true, invalidPlanPreserved=true,
                manualAndDisabledAxisCancelPlan=true, motionModeUiVerified=true, sourceAssetsUnchanged=unchanged,
                directButtonClicks=0, scope="Configured velocity bounds and sampled kinematic line accuracy; no acceleration, collision or industrial safety validation", rows };
            Write("motion-verification.json",report);
            return new { report.passed, report.actualArrivals, report.rejectedRequestsPreserved, report.motionSamples,
                report.maxActualLineDeviationMetres, report.maxObservedRotarySpeedDegreesPerSecond, report.maxObservedSlideSpeedMetresPerSecond, report.maxJointProgressSpread,
                report.stopHeld, report.manualAndDisabledAxisCancelPlan, report.sourceAssetsUnchanged };
        }
        finally { Time.timeScale=originalScale; if(lab!=null) { lab.Axis4.enabled=enabled4; lab.StopAll(); } }
    }

    private static bool AtRest(ScaraJointLab l)=>!l.IsTrajectoryMoving && !Flags(l).Any(v=>v) && l.Axis1.IsAtTarget && l.Axis2.IsAtTarget && l.Axis3.IsAtTarget && l.Axis4.IsAtTarget;
    private static Vector4 Current(ScaraJointLab l)=>new Vector4(l.Axis1.CurrentPosition,l.Axis2.CurrentPosition,l.Axis3.CurrentPosition,l.Axis4.CurrentPosition);
    private static Vector4 Targets(ScaraJointLab l)=>new Vector4(l.Axis1.TargetPosition,l.Axis2.TargetPosition,l.Axis3.TargetPosition,l.Axis4.TargetPosition);
    private static bool[] Flags(ScaraJointLab l)=>new[]{l.Axis1.IsMoving,l.Axis2.IsMoving,l.Axis3.IsMoving,l.Axis4.IsMoving};
    private static async Task Until(Func<bool> condition,CancellationToken token)
    {
        var watch=Stopwatch.StartNew();
        while(!condition()) { token.ThrowIfCancellationRequested(); Require(!EditorApplication.isPaused,"Verification interrupted by pause"); if(watch.Elapsed.TotalSeconds>12) throw new TimeoutException("Motion verification timeout"); await Task.Delay(20,token); }
    }
    private static string Hash(string path) { using(var sha=SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))).Replace("-",""); }

    private static void CheckLine(Planner.Plan plan,ScaraForwardKinematics.TcpPose goal,ScaraForwardKinematics.Geometry g,ref int samples,ref float maxPosition,ref float maxYaw)
    {
        CheckTiming(plan);
        var origin = Fk(plan.Nodes[0].Joints,g);
        float yawDelta = Mathf.DeltaAngle(origin.YawDegrees,goal.YawDegrees);
        for (int i=1;i<plan.Nodes.Count;i++)
        for (int k=0;k<=32;k++)
        {
            float t = k/32f;
            var a = plan.Nodes[i-1]; var b = plan.Nodes[i];
            float fraction = Mathf.Lerp(a.Fraction,b.Fraction,t);
            var actual = Fk(Planner.Lerp(a.Joints,b.Joints,t),g);
            float ep = Vector3.Distance(actual.PositionMetres,Vector3.Lerp(origin.PositionMetres,goal.PositionMetres,fraction));
            float ey = Mathf.Abs(Mathf.DeltaAngle(actual.YawDegrees,origin.YawDegrees+yawDelta*fraction));
            Require(ep<=.000101f && ey<=.001f,"Dense path check failed");
            maxPosition = Mathf.Max(maxPosition,ep); maxYaw = Mathf.Max(maxYaw,ey); samples++;
        }
    }
    private static void CheckTiming(Planner.Plan plan)
    {
        for(int i=1;i<plan.Nodes.Count;i++)
        {
            var a = plan.Nodes[i-1].Joints; var b = plan.Nodes[i].Joints; float dt = plan.Nodes[i].SecondsFromPrevious;
            Require(dt>=0f && MaxRotation(a,b)<=30f*dt+.00001f && Mathf.Abs(a.D3Metres-b.D3Metres)<=.05f*dt+.00000001f,"Segment speed bound failed");
        }
    }
    private static IK.JointValues J(float a,float b,float c,float d) => new IK.JointValues(a,b,c,d);
    private static ScaraForwardKinematics.TcpPose Fk(IK.JointValues j,ScaraForwardKinematics.Geometry g) => ScaraForwardKinematics.Calculate(j.Q1Degrees,j.Q2Degrees,j.D3Metres,j.Q4Degrees,g);
    private static float MaxRotation(IK.JointValues a,IK.JointValues b) => Mathf.Max(Mathf.Abs(a.Q1Degrees-b.Q1Degrees),Mathf.Abs(a.Q2Degrees-b.Q2Degrees),Mathf.Abs(a.Q4Degrees-b.Q4Degrees));
    private static double Cost(IK.JointValues a,IK.JointValues b) { double x=(double)a.Q1Degrees-b.Q1Degrees,y=(double)a.Q2Degrees-b.Q2Degrees,z=(double)a.Q4Degrees-b.Q4Degrees; return x*x+y*y+z*z; }
    private static void Write(string name,object report) { Directory.CreateDirectory("Logs/LW07Path"); File.WriteAllText("Logs/LW07Path/"+name,JsonConvert.SerializeObject(report,Formatting.Indented)); }
    private static void Require(bool value,string message) { if(!value) throw new InvalidOperationException(message); }
}
