using System;
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
using Policy = LineWatch.FanucJointLab.IkSeedPolicy;

public static class VerifyFanucTcpMotion
{
    public static async Task<object> Run()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused,"Use unpaused Play Mode.");
        var lab=UnityEngine.Object.FindFirstObjectByType<FanucJointLab>();Require(lab!=null && lab.IsReady,"FANUC not ready.");
        var axes=lab.Joints.ToArray();var speeds=axes.Select(a=>a.SpeedDegreesPerSecond).ToArray();
        var lower=axes.Select(j=>j.MinimumDegrees).ToArray();var upper=axes.Select(j=>j.MaximumDegrees).ToArray();
        var directions=axes.Select(a=>a.LocalRotationAxis).ToArray();var enabled=axes.Select(a=>a.enabled).ToArray();
        var tool=lab.Geometry.ToolPositionMetres;var tcpPosition=lab.Tcp.localPosition;
        float timeScale=Time.timeScale;var token=lab.destroyCancellationToken;
        var paths=new[]{"Assets/01.Scenes/RobotLab.unity","Assets/03.Prefabs/FanucRobot_Work.prefab","Assets/03.Prefabs/ScaraRobot_Work.prefab","ProjectSettings/ProjectSettings.asset"};
        var hashes=paths.ToDictionary(p=>p,Hash);
        var a=FanucForwardKinematics.Calculate(new[]{15f,-20f,25f,-15f,20f,10f},lab.Geometry);
        var b=FanucForwardKinematics.Calculate(new[]{-20f,15f,-25f,20f,-15f,-10f},lab.Geometry);
        var c=FanucForwardKinematics.Calculate(new[]{30f,-25f,20f,-30f,25f,-20f},lab.Geometry);
        Require(EditorApplication.ExecuteMenuItem("LineWatch/FANUC 조작 테스트"),"Menu missing.");
        var root=EditorWindow.GetWindow<FanucJointLabWindow>().rootVisualElement;
        var move=root.Q<Button>("fanuc-ik-move");var stop=root.Q<Button>("fanuc-ik-stop");
        var state=root.Q<Label>("fanuc-motion-state");var position=root.Q<Vector3Field>("fanuc-ik-position");
        var rotation=root.Q<Vector4Field>("fanuc-ik-rotation");
        Require(move!=null && stop!=null && state!=null && position!=null && rotation!=null,"Motion UI incomplete.");
        int arrivals=0,rejections=0,samples=0,faults=0;float maxPosition=0,maxRotation=0,maxSpeed=0,maxSpread=0;
        float[] Current()=>axes.Select(j=>j.CurrentPosition).ToArray();
        void Reject(Func<bool> request,string name)
        {string before=State(lab,axes);Require(!request(),"Invalid request accepted: "+name);Require(before==State(lab,axes),"Rejected request changed active commands: "+name);rejections++;}
        FanucJointLab.IkPreview Start(FanucForwardKinematics.TcpPose goal)
        {
            var before=Current();
            Require(lab.TryMoveTcp(goal.PositionMetres,goal.Rotation,Policy.CurrentThenFixed,out var p,out var error),"Move rejected: "+error);
            Require(before.SequenceEqual(Current()),"Accepting a plan teleported a joint.");
            return p;
        }
        async Task Trace(float[] start,FanucJointLab.IkPreview accepted,FanucForwardKinematics.TcpPose goal)
        {
            var end=accepted.Result.JointDegrees.ToArray();var last=Current();float lastTime=Time.fixedTime;
            var watch=Stopwatch.StartNew();
            while(lab.IsTcpMoving)
            {
                token.ThrowIfCancellationRequested();Require(!EditorApplication.isPaused && watch.Elapsed.TotalSeconds<12,"Motion timed out.");
                var q=Current();float now=Time.fixedTime;float min=1,max=0;int changed=0;
                for(int i=0;i<6;i++)
                {
                    Require(q[i]>=axes[i].MinimumDegrees && q[i]<=axes[i].MaximumDegrees,"Intermediate angle outside limits.");
                    if(Mathf.Abs(end[i]-start[i])>.01f)
                    {float progress=(q[i]-start[i])/(end[i]-start[i]);min=Mathf.Min(min,progress);max=Mathf.Max(max,progress);changed++;}
                    if(now>lastTime)maxSpeed=Mathf.Max(maxSpeed,Mathf.Abs(q[i]-last[i])/(now-lastTime));
                }
                if(changed>1){maxSpread=Mathf.Max(maxSpread,max-min);Require(max-min<.00001f,"Joint progress is not synchronized.");}
                Require(lab.TryReadFkDiagnostics(out var d,out var e) && d.PositionErrorMetres<.00001f && d.RotationErrorDegrees<.005f,"In-flight FK/model mismatch: "+e);
                samples++;last=q;lastTime=now;await Task.Delay(20,token);
            }
            Require(axes.All(j=>!j.IsMoving && j.IsAtTarget),"Plan did not finish all axes.");
            Require(lab.TryReadActualTcp(out var actual,out var orientation,out var reason),reason);
            float pe=Vector3.Distance(actual,goal.PositionMetres),re=Angle(orientation,goal.Rotation);
            maxPosition=Mathf.Max(maxPosition,pe);maxRotation=Mathf.Max(maxRotation,re);
            Require(pe<=.002f && re<=.5f,"Actual TCP arrival missed integration tolerance.");
            arrivals++;
        }
        async Task Home(){lab.HomeAll();await Until(()=>axes.All(j=>!j.IsMoving && Mathf.Abs(j.CurrentPosition)<.001f),token);}
        async Task BeginA(){Start(a);await Until(()=>axes[0].CurrentPosition>=3f && lab.IsTcpMoving,token);}
        async Task Hold()
        {
            var q=Current();float before=Time.fixedTime;await Until(()=>Time.fixedTime-before>=4*Time.fixedDeltaTime,token);
            Require(q.SequenceEqual(Current()) && axes.All(j=>!j.IsMoving),"Stopped axes drifted.");
        }
        async Task FaultStopped()
        {
            await Until(()=>!lab.IsTcpMoving,token);
            Require(!string.IsNullOrEmpty(lab.LastMotionError) && axes.All(j=>!j.IsMoving),"Fault did not stop the whole owned group.");
            await Hold();faults++;
        }
        try
        {
            await Home();position.value=a.PositionMetres*1000;rotation.value=V4(a.Rotation);
            await Until(()=>move.enabledSelf && stop.enabledSelf,token);
            var start=Current();var accepted=Start(a);
            await Until(()=>state.text.Contains("이동 중"),token);
            await Trace(start,accepted,a);
            Require(maxSpeed<speeds.Max()+.02f,"Observed motion exceeded configured speed (including sample-time tolerance).");

            var beforeB=Current();Start(b);await Until(()=>Mathf.Abs(axes[0].CurrentPosition-beforeB[0])>.5f && lab.IsTcpMoving,token);
            Reject(()=>lab.TryMoveTcp(new Vector3(3,0,0),b.Rotation,Policy.CurrentThenFixed,out _,out _),"outside bound");
            Reject(()=>lab.TryMoveTcp(c.PositionMetres,default,Policy.CurrentThenFixed,out _,out _),"zero quaternion");
            Reject(()=>lab.TryMoveTcp(c.PositionMetres,c.Rotation,(Policy)999,out _,out _),"invalid policy");
            Require(lab.TryPreviewTcp(c.PositionMetres,c.Rotation,Policy.CurrentThenFixed,out _,out _),"Expected a valid preview before stale-input test.");
            SetTool(lab,tool+new Vector3(0,.01f,0));
            Reject(()=>lab.TryMoveTcp(c.PositionMetres,c.Rotation,Policy.CurrentThenFixed,out _,out _),"geometry changed after preview");
            SetTool(lab,tool);
            SetFloat(axes[5],"speedDegreesPerSecond",0);
            Reject(()=>lab.TryMoveTcp(c.PositionMetres,c.Rotation,Policy.CurrentThenFixed,out _,out _),"last axis invalid speed");
            SetFloat(axes[5],"speedDegreesPerSecond",speeds[5]);
            var oldTargets=axes.Select(j=>j.TargetPosition).ToArray();string beforeInput=State(lab,axes);
            position.value=new Vector3(3000,0,0);
            Require(!move.enabledSelf && State(lab,axes)==beforeInput,"Input edit did not invalidate display-only preview cleanly.");
            await Until(()=>!move.enabledSelf && root.Q<HelpBox>("fanuc-ik-failure").text.Contains("외곽"),token);
            Require(lab.IsTcpMoving && oldTargets.SequenceEqual(axes.Select(j=>j.TargetPosition)),"Editing preview retargeted the active plan.");
            position.value=c.PositionMetres*1000;rotation.value=V4(c.Rotation);
            await Until(()=>move.enabledSelf,token);
            EditorApplication.isPaused=true;
            Reject(()=>lab.TryMoveTcp(c.PositionMetres,c.Rotation,Policy.CurrentThenFixed,out _,out _),"editor pause");
            await Until(()=>!move.enabledSelf && stop.enabledSelf,token);
            EditorApplication.isPaused=false;
            Time.timeScale=0;
            Reject(()=>lab.TryMoveTcp(c.PositionMetres,c.Rotation,Policy.CurrentThenFixed,out _,out _),"time scale zero");
            Time.timeScale=timeScale;
            var replacementStart=Current();var replacement=Start(c);
            Require(replacement.CurrentJointDegrees.SequenceEqual(replacementStart),"Replacement did not use actual current joints.");
            await Until(()=>Mathf.Abs(axes[0].CurrentPosition-replacementStart[0])>.5f,token);
            lab.StopJoint(2);Require(!lab.IsTcpMoving && axes.All(j=>!j.IsMoving),"Single-axis stop did not cancel the group.");await Hold();
            var resumeStart=Current();var resumed=Start(c);await Trace(resumeStart,resumed,c);

            Require(lab.TryReadActualTcp(out var currentPosition,out var currentRotation,out _),"Cannot read current pose.");
            var noOpStart=Current();
            Require(lab.TryMoveTcp(currentPosition,currentRotation,Policy.CurrentOnly,out _,out _)
                && !lab.IsTcpMoving && axes.All(j=>!j.IsMoving) && noOpStart.SequenceEqual(Current()),"Already-arrived goal did not complete without movement.");

            Start(a);
            Reject(()=>lab.MoveJoint(0,99),"invalid manual override");
            Require(lab.MoveJoint(0,5) && !lab.IsTcpMoving && axes.Skip(1).All(j=>!j.IsMoving),"Manual override did not cancel the planned group.");
            await Until(()=>!axes[0].IsMoving,token);await Home();
            await BeginA();lab.HomeAll();
            Require(!lab.IsTcpMoving && axes.All(j=>j.TargetPosition==0),"Home utility did not cancel the plan.");
            await Home();

            SetJoint(lab,5,axes[0]);
            Reject(()=>lab.TryMoveTcp(a.PositionMetres,a.Rotation,Policy.CurrentThenFixed,out _,out _),"duplicate/wrong-order binding at home");
            RestoreJoints(lab,axes);
            SetVector(axes[5],"localRotationAxis",Vector3.forward);
            Reject(()=>lab.TryMoveTcp(a.PositionMetres,a.Rotation,Policy.CurrentThenFixed,out _,out _),"axis direction changed at zero pose");
            SetVector(axes[5],"localRotationAxis",directions[5]);

            await BeginA();SetFloat(axes[5],"speedDegreesPerSecond",speeds[5]+1);await FaultStopped();
            SetFloat(axes[5],"speedDegreesPerSecond",speeds[5]);await Home();
            await BeginA();SetFloat(axes[5],"maximumDegrees",upper[5]-1);await FaultStopped();
            SetFloat(axes[5],"maximumDegrees",upper[5]);await Home();
            await BeginA();SetTool(lab,tool+new Vector3(0,.01f,0));lab.Tcp.localPosition=tcpPosition+new Vector3(0,.01f,0);await FaultStopped();
            SetTool(lab,tool);lab.Tcp.localPosition=tcpPosition;await Home();
            await BeginA();SetJoint(lab,5,axes[0]);await FaultStopped();
            RestoreJoints(lab,axes);await Home();
            await BeginA();axes[2].enabled=false;await FaultStopped();axes[2].enabled=true;await Home();
            await Until(()=>state.text.Contains("대기") && string.IsNullOrEmpty(lab.LastMotionError),token);
            Require(hashes.All(p=>Hash(p.Key)==p.Value),"A source asset changed.");
            var report=new{passed=true,utc=DateTime.UtcNow.ToString("o"),actualArrivals=arrivals,rejectedRequestsPreserved=rejections,
                movingSamples=samples,maxActualPositionErrorMetres=maxPosition,maxActualRotationErrorDegrees=maxRotation,
                maxObservedJointSpeedDegreesPerSecond=maxSpeed,maxJointProgressSpread=maxSpread,
                activeReplacementPassed=true,groupStopHeld=true,resumeFromActualPassed=true,alreadyArrivedGoalPassed=true,
                manualAndHomeOverridePassed=true,executionFaultGroupsStopped=faults,
                stalePreviewRecalculated=true,inputEditingDidNotRetarget=true,motionUiStateRead=true,
                sourceAssetsUnchanged=true,sourceHashes=hashes,manualClicksPerformed=false,
                scope="LW-07D process 2: two actual TCP arrivals plus replacement/stop/manual/fault probes; not the 100-arrival campaign, no acceleration/collision validation"};
            Directory.CreateDirectory("Logs/LW07D");
            File.WriteAllText("Logs/LW07D/motion-verification.json",JsonConvert.SerializeObject(report,Formatting.Indented));
            return report;
        }
        finally
        {
            EditorApplication.isPaused=false;Time.timeScale=timeScale;
            if(lab!=null && Application.isPlaying)
            {
                RestoreJoints(lab,axes);SetTool(lab,tool);lab.Tcp.localPosition=tcpPosition;
                for(int i=0;i<6;i++){axes[i].enabled=enabled[i];SetFloat(axes[i],"speedDegreesPerSecond",speeds[i]);
                    SetFloat(axes[i],"minimumDegrees",lower[i]);SetFloat(axes[i],"maximumDegrees",upper[i]);SetVector(axes[i],"localRotationAxis",directions[i]);}
                lab.StopAll();
            }
        }
    }

    private static string State(FanucJointLab lab,RotaryAxis[] axes)=>JsonConvert.SerializeObject(new{lab.IsTcpMoving,lab.LastMotionError,
        axes=axes.Select(j=>new{j.CurrentPosition,j.TargetPosition,j.IsMoving,r=Q(j.transform.localRotation)})});
    private static void SetTool(FanucJointLab lab,Vector3 value){var s=new SerializedObject(lab);s.FindProperty("geometry").FindPropertyRelative("toolPositionMetres").vector3Value=value;s.ApplyModifiedPropertiesWithoutUndo();}
    private static void SetFloat(UnityEngine.Object o,string name,float value){var s=new SerializedObject(o);s.FindProperty(name).floatValue=value;s.ApplyModifiedPropertiesWithoutUndo();}
    private static void SetVector(UnityEngine.Object o,string name,Vector3 value){var s=new SerializedObject(o);s.FindProperty(name).vector3Value=value;s.ApplyModifiedPropertiesWithoutUndo();}
    private static void SetJoint(FanucJointLab lab,int index,RotaryAxis axis){var s=new SerializedObject(lab);s.FindProperty("joints").GetArrayElementAtIndex(index).objectReferenceValue=axis;s.ApplyModifiedPropertiesWithoutUndo();}
    private static void RestoreJoints(FanucJointLab lab,RotaryAxis[] axes){for(int i=0;i<6;i++)SetJoint(lab,i,axes[i]);}
    private static Vector4 V4(Quaternion q)=>new Vector4(q.x,q.y,q.z,q.w);
    private static float[] Q(Quaternion q)=>new[]{q.x,q.y,q.z,q.w};
    private static async Task Until(Func<bool> condition,CancellationToken token){var watch=Stopwatch.StartNew();while(!condition())
        {token.ThrowIfCancellationRequested();Require(watch.Elapsed.TotalSeconds<12,"Verification wait timed out.");await Task.Delay(20,token);}}
    private static float Angle(Quaternion a,Quaternion b){var d=Quaternion.Inverse(a)*b;return(float)(2*Math.Atan2(Math.Sqrt((double)d.x*d.x+(double)d.y*d.y+(double)d.z*d.z),Math.Abs(d.w))*180/Math.PI);}
    private static string Hash(string path){using(var sha=SHA256.Create())return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path)));}
    private static void Require(bool condition,string message){if(!condition)throw new InvalidOperationException(message);}
}
