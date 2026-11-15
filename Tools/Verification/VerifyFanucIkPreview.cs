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

public static class VerifyFanucIkPreview
{
    private static readonly string[] Paths={"Assets/01.Scenes/RobotLab.unity","Assets/03.Prefabs/FanucRobot_Work.prefab",
        "Assets/03.Prefabs/ScaraRobot_Work.prefab","ProjectSettings/ProjectSettings.asset"};

    public static object Edit()
    {
        Require(!EditorApplication.isPlaying,"Use Edit Mode.");
        var lab=UnityEngine.Object.FindFirstObjectByType<FanucJointLab>();
        Require(EditorApplication.ExecuteMenuItem("LineWatch/FANUC 조작 테스트"),"Menu not found.");
        var root=EditorWindow.GetWindow<FanucJointLabWindow>().rootVisualElement;
        var position=root.Q<Vector3Field>("fanuc-ik-position");var rotation=root.Q<Vector4Field>("fanuc-ik-rotation");
        Require(position!=null && rotation!=null && !position.enabledSelf && !rotation.enabledSelf,"Edit inputs must wait for Play.");
        Require(root.Q<Label>("fanuc-ik-result").text.Contains("Play"),"Missing edit-state display.");
        Require(!lab.TryPreviewTcp(Vector3.zero,Quaternion.identity,Policy.CurrentThenFixed,out var preview,out var error)
            && preview.Result==null && !string.IsNullOrEmpty(error),"Edit preview should reject current-joint sampling.");
        var report=new{passed=true,utc=DateTime.UtcNow.ToString("o"),editStateShown=true,inputsLocked=true,manualClicksPerformed=false};
        Save("preview-edit-verification.json",report);return report;
    }

    public static async Task<object> Play()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused,"Use unpaused Play Mode.");
        var lab=UnityEngine.Object.FindFirstObjectByType<FanucJointLab>();Require(lab!=null && lab.IsReady,"FANUC not ready.");
        var token=lab.destroyCancellationToken;var hashes=Paths.ToDictionary(p=>p,Hash);
        var originalTool=lab.Geometry.ToolPositionMetres;var originalFrame=lab.RobotFrame;
        Require(EditorApplication.ExecuteMenuItem("LineWatch/FANUC 조작 테스트"),"Menu not found.");
        var root=EditorWindow.GetWindow<FanucJointLabWindow>().rootVisualElement;
        var position=root.Q<Vector3Field>("fanuc-ik-position");var rotation=root.Q<Vector4Field>("fanuc-ik-rotation");
        var policy=root.Q<DropdownField>("fanuc-ik-policy");var state=root.Q<Label>("fanuc-ik-result");
        var joints=root.Q<Label>("fanuc-ik-joints");var info=root.Q<Label>("fanuc-ik-snapshot");
        var failure=root.Q<HelpBox>("fanuc-ik-failure");
        Require(position!=null && rotation!=null && policy!=null && state!=null && joints!=null && info!=null && failure!=null,"Preview display incomplete.");
        var goal=FanucForwardKinematics.Calculate(new[]{15f,-20f,25f,-15f,20f,10f},lab.Geometry);
        var negative=FanucForwardKinematics.Calculate(new[]{-20f,15f,-25f,20f,-15f,-10f},lab.Geometry);
        void Inputs(FanucForwardKinematics.TcpPose p,int index)
        {position.value=p.PositionMetres*1000;rotation.value=V4(p.Rotation);policy.index=index;}
        FanucJointLab.IkPreview Read(FanucForwardKinematics.TcpPose p,Policy choice)
        {
            string before=State(lab);
            Require(lab.TryPreviewTcp(p.PositionMetres,p.Rotation,choice,out var preview,out var error),"Preview failed: "+error);
            Require(State(lab)==before,"Preview changed joint state/commands.");
            Check(preview,p,lab);return preview;
        }
        try
        {
            lab.HomeAll();await Until(()=>At(lab,new float[6]),token);
            var homePreview=Read(goal,Policy.CurrentThenFixed);
            Require(homePreview.AvailableSeeds==3 && homePreview.SeedName=="현재 자세","Duplicate home seed was not removed.");
            Inputs(goal,0);await Until(()=>state.text.Contains("계산 성공") && joints.text.Contains("A6") && info.text.Contains("현재 자세"),token);
            Inputs(goal,2);await Until(()=>state.text.Contains("계산 성공") && info.text.Contains("홈"),token);
            var fixedPreview=Read(goal,Policy.Fixed);Require(fixedPreview.AvailableSeeds==3,"Fixed policy changed.");

            var bad=new[]{10f,-15f,25f,15f,20f,-10f};Move(lab,bad);
            await Until(()=>lab.Joints[0].CurrentPosition>=3,token);
            Read(goal,Policy.CurrentThenFixed);
            await Until(()=>At(lab,bad),token);
            string stationary=State(lab);
            Require(!lab.TryPreviewTcp(negative.PositionMetres,negative.Rotation,Policy.CurrentOnly,out var single,out var singleError)
                && single.Result.Reason==FanucInverseKinematics.Failure.IterationLimit && single.Result.JointDegrees.Count==0,
                "Current-only poor seed should expose a nonconvergence failure.");
            Require(State(lab)==stationary && !singleError.Contains("외곽"),"Failure changed commands or used the wrong message.");
            Inputs(negative,1);
            await Until(()=>state.text.Contains("계산 실패") && joints.text=="관절 해: 없음" && failure.text.Contains("반복 예산"),token);
            Inputs(negative,0);var recovered=Read(negative,Policy.CurrentThenFixed);
            Require(recovered.SeedName=="홈" && recovered.Result.SeedsTried==2 && recovered.AvailableSeeds==3,"Fallback/dedup used wrong seed policy.");
            await Until(()=>state.text.Contains("계산 성공") && info.text.Contains("홈") && failure.style.display.value==DisplayStyle.None,token);

            var arbitrary=new[]{5f,-8f,12f,6f,9f,-4f};Move(lab,arbitrary);await Until(()=>At(lab,arbitrary),token);
            string originalState=State(lab);int successes=0;
            foreach(var q in Poses())
            {
                var target=FanucForwardKinematics.Calculate(q,lab.Geometry);
                if(lab.TryPreviewTcp(target.PositionMetres,target.Rotation,Policy.CurrentThenFixed,out var p,out _))
                {Check(p,target,lab);Require(p.AvailableSeeds==4,"Arbitrary current seed should be combined with fixed three.");successes++;}
                else Require(p.Result!=null && p.Result.JointDegrees.Count==0,"Failed preview exposed stale solution.");
            }
            Require(State(lab)==originalState,"Numerical preview sweep moved the robot.");

            position.value=new Vector3(3000,0,0);rotation.value=V4(goal.Rotation);policy.index=0;
            Require(!lab.TryPreviewTcp(new Vector3(3,0,0),goal.Rotation,Policy.CurrentThenFixed,out var outside,out _)
                && outside.Result.Reason==FanucInverseKinematics.Failure.OutsideReachBound,"Missing proven outer-bound failure.");
            await Until(()=>failure.text.Contains("외곽"),token);
            Require(joints.text=="관절 해: 없음","Old solution remained visible after failure.");
            Inputs(goal,0);await Until(()=>state.text.Contains("계산 성공"),token);
            string beforeInvalidPolicy=State(lab);
            Require(!lab.TryPreviewTcp(goal.PositionMetres,goal.Rotation,(Policy)99,out var invalidPolicy,out var policyError)
                && invalidPolicy.Result==null && policyError.Contains("정책") && State(lab)==beforeInvalidPolicy,"Invalid policy did not preserve commands.");
            rotation.value=Vector4.zero;await Until(()=>failure.text.Contains("Quaternion") && joints.text=="관절 해: 없음",token);
            Inputs(goal,0);await Until(()=>state.text.Contains("계산 성공"),token);

            SetTool(lab,originalTool+new Vector3(0,.01f,0));
            Require(!lab.TryPreviewTcp(goal.PositionMetres,goal.Rotation,Policy.CurrentThenFixed,out var mismatch,out var mismatchError)
                && mismatch.Result==null && mismatchError.Contains("일치하지"),"Uncalibrated model accepted a preview.");
            await Until(()=>failure.text.Contains("일치하지") && joints.text=="관절 해: 없음",token);
            SetTool(lab,originalTool);await Until(()=>state.text.Contains("계산 성공"),token);
            SetFrame(lab,null);await Until(()=>state.text.Contains("계산 실패") && !position.enabledSelf,token);
            SetFrame(lab,originalFrame);await Until(()=>state.text.Contains("계산 성공") && position.enabledSelf,token);
            EditorApplication.isPaused=true;Read(goal,Policy.CurrentThenFixed);EditorApplication.isPaused=false;
            Require(State(lab)==originalState,"Readonly failures/recovery changed commands.");
            lab.HomeAll();await Until(()=>At(lab,new float[6]),token);
            var hashesUnchanged=hashes.All(p=>Hash(p.Key)==p.Value);Require(hashesUnchanged,"A source asset changed.");
            var report=new{passed=successes==100,utc=DateTime.UtcNow.ToString("o"),arbitraryCurrentPreviewTargets=100,
                arbitraryCurrentPreviewSuccesses=successes,currentSeedDeduplicated=true,fixedPolicyPassed=true,
                currentOnlyFailureReason=single.Result.Reason.ToString(),fallbackSeed=recovered.SeedName,
                fallbackIterations=recovered.Result.Iterations,previewPreservedMovingAndStationaryCommands=true,
                outsideBoundShown=true,invalidRotationShown=true,invalidPolicyRejected=true,geometryMismatchRejected=true,missingFrameShown=true,
                pausedReadonlyPreviewPassed=true,recoveryPassed=true,sourceAssetsUnchanged=hashesUnchanged,sourceHashes=hashes,
                manualClicksPerformed=false,scope="LW-07D first process: readonly preview API/UI; old manual APIs prepare poses, no new TCP move command or 100 target arrivals"};
            Save("preview-play-verification.json",report);
            Require(report.passed,"Some fixed preview policy sweep goals failed; inspect report.");return report;
        }
        finally
        {
            EditorApplication.isPaused=false;
            if(lab!=null && Application.isPlaying){SetTool(lab,originalTool);SetFrame(lab,originalFrame);lab.StopAll();}
        }
    }

    private static void Check(FanucJointLab.IkPreview preview,FanucForwardKinematics.TcpPose goal,FanucJointLab lab)
    {
        var r=preview.Result;Require(r!=null && r.Succeeded && r.JointDegrees.Count==6,"Invalid successful preview.");
        Require(r.JointDegrees.Select((q,i)=>q>=lab.Joints[i].MinimumDegrees && q<=lab.Joints[i].MaximumDegrees).All(v=>v),"Preview outside limits.");
        var fk=FanucForwardKinematics.Calculate(r.JointDegrees,lab.Geometry);
        Require(Vector3.Distance(fk.PositionMetres,goal.PositionMetres)<.000101f && Angle(fk.Rotation,goal.Rotation)<.0501f,"Preview failed FK validation.");
    }
    private static System.Collections.Generic.List<float[]> Poses()
    {
        var result=new System.Collections.Generic.List<float[]>{new float[6]};
        for(int i=0;i<6;i++)foreach(float value in new[]{-45f,45f}){var q=new float[6];q[i]=value;result.Add(q);}
        result.Add(Enumerable.Repeat(-45f,6).ToArray());result.Add(Enumerable.Repeat(45f,6).ToArray());
        result.Add(Enumerable.Range(0,6).Select(i=>i%2==0?-45f:45f).ToArray());
        result.Add(Enumerable.Range(0,6).Select(i=>i%2==0?45f:-45f).ToArray());
        result.Add(Enumerable.Repeat(.0001f,6).ToArray());result.Add(Enumerable.Repeat(-.0001f,6).ToArray());
        result.Add(new[]{25f,-30f,20f,-35f,15f,40f});result.Add(new[]{-40f,15f,-25f,30f,-35f,-20f});
        var random=new System.Random(20261115);
        while(result.Count<100)result.Add(Enumerable.Range(0,6).Select(_=>-45+90*(float)random.NextDouble()).ToArray());
        return result;
    }
    private static void Move(FanucJointLab lab,float[] q){for(int i=0;i<6;i++)Require(lab.MoveJoint(i,q[i]),"Old manual setup command rejected.");}
    private static bool At(FanucJointLab lab,float[] q)=>lab.Joints.Select((j,i)=>!j.IsMoving && Mathf.Abs(j.CurrentPosition-q[i])<.001f).All(v=>v);
    private static string State(FanucJointLab lab)=>JsonConvert.SerializeObject(lab.Joints.Select(j=>new{j.CurrentPosition,j.TargetPosition,j.IsMoving}));
    private static Vector4 V4(Quaternion q)=>new Vector4(q.x,q.y,q.z,q.w);
    private static void SetTool(FanucJointLab lab,Vector3 p){var s=new SerializedObject(lab);s.FindProperty("geometry").FindPropertyRelative("toolPositionMetres").vector3Value=p;s.ApplyModifiedPropertiesWithoutUndo();}
    private static void SetFrame(FanucJointLab lab,Transform p){var s=new SerializedObject(lab);s.FindProperty("robotFrame").objectReferenceValue=p;s.ApplyModifiedPropertiesWithoutUndo();}
    private static async Task Until(Func<bool> condition,CancellationToken token){var watch=Stopwatch.StartNew();while(!condition())
        {token.ThrowIfCancellationRequested();Require(watch.Elapsed.TotalSeconds<10,"Preview/UI wait timed out.");await Task.Delay(40,token);}}
    private static float Angle(Quaternion a,Quaternion b){var d=Quaternion.Inverse(a)*b;return(float)(2*Math.Atan2(Math.Sqrt((double)d.x*d.x+(double)d.y*d.y+(double)d.z*d.z),Math.Abs(d.w))*180/Math.PI);}
    private static string Hash(string path){using(var sha=SHA256.Create())return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path)));}
    private static void Save(string name,object report){Directory.CreateDirectory("Logs/LW07D");File.WriteAllText("Logs/LW07D/"+name,JsonConvert.SerializeObject(report,Formatting.Indented));}
    private static void Require(bool condition,string message){if(!condition)throw new InvalidOperationException(message);}
}
