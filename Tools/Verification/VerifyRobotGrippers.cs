using System;
using System.IO;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Security.Cryptography;
using System.Threading;
using System.Threading.Tasks;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

public static class VerifyRobotGrippers
{
    private static readonly string[] RobotPrefabs = { "Assets/03.Prefabs/ScaraRobot_Work.prefab", "Assets/03.Prefabs/FanucRobot_Work.prefab" };
    private static readonly string[] PartPrefabs = { "Assets/03.Prefabs/GripTrialPart.prefab" };

    public static object Inspect()
    {
        Require(!EditorApplication.isPlaying, "Inspect in Edit Mode.");
        var s = UnityEngine.Object.FindFirstObjectByType<ScaraJointLab>();
        var f = UnityEngine.Object.FindFirstObjectByType<FanucJointLab>();
        object Describe(Transform tcp, Transform meshRoot) => new { tcp = tcp.name,
            meshes = meshRoot.GetComponentsInChildren<MeshFilter>().Select(m => {
                var b = m.sharedMesh.bounds; var corners = new Vector3[8];
                for(int i=0;i<8;i++) corners[i] = tcp.InverseTransformPoint(m.transform.TransformPoint(b.center +
                    Vector3.Scale(b.extents,new Vector3((i&1)==0?-1:1,(i&2)==0?-1:1,(i&4)==0?-1:1))));
                var bounds = new Bounds(corners[0],Vector3.zero); foreach(var c in corners) bounds.Encapsulate(c);
                return new { name=m.name, center=V(bounds.center), size=V(bounds.size), min=V(bounds.min), max=V(bounds.max) };
            }).ToArray() };
        var report = new { scara=Describe(s.Tcp,s.Tcp.parent), fanuc=Describe(f.Tcp,f.Tcp) };
        Directory.CreateDirectory("Logs/LW08"); File.WriteAllText("Logs/LW08/gripper-model-datums.json",JsonConvert.SerializeObject(report,Formatting.Indented));
        return report;
    }

    public static object Wire()
    {
        Require(!EditorApplication.isPlaying, "Wire in Edit Mode.");
        const string materialPath="Assets/09.Materials/GripTrial.mat";
        var material=AssetDatabase.LoadAssetAtPath<Material>(materialPath);
        if(material==null)
        {
            var shader=Shader.Find("Universal Render Pipeline/Lit");Require(shader!=null,"URP Lit shader required.");
            material=new Material(shader); material.color=new Color(.12f,.85f,.3f,1);
            AssetDatabase.CreateAsset(material,materialPath);
        }
        var size=new Vector3(.024f,.012f,.024f);
        for(int robot=0;robot<2;robot++)
        {
            if(AssetDatabase.LoadAssetAtPath<GameObject>(PartPrefabs[0])==null)
            {
                var root=new GameObject("GripTrialPart");
                try
                {
                    var body=root.AddComponent<Rigidbody>();body.isKinematic=true;body.useGravity=false;body.interpolation=RigidbodyInterpolation.None;
                    root.AddComponent<BoxCollider>().size=size;
                    var part=root.AddComponent<GrippableItem>();
                    var point=new GameObject("GripPoint").transform;point.SetParent(root.transform,false);
                    point.localPosition=Vector3.zero;
                    var visual=GameObject.CreatePrimitive(PrimitiveType.Cube);visual.name="Visual";visual.transform.SetParent(root.transform,false);
                    visual.transform.localScale=size;UnityEngine.Object.DestroyImmediate(visual.GetComponent<Collider>());
                    visual.GetComponent<Renderer>().sharedMaterial=material;
                    var s=new SerializedObject(part);s.FindProperty("gripPoint").objectReferenceValue=point;s.ApplyModifiedPropertiesWithoutUndo();
                    PrefabUtility.SaveAsPrefabAsset(root,PartPrefabs[0]);
                }
                finally{UnityEngine.Object.DestroyImmediate(root);}
            }
            var robotRoot=PrefabUtility.LoadPrefabContents(RobotPrefabs[robot]);
            try
            {
                var scara=robotRoot.GetComponent<ScaraJointLab>();var fanuc=robotRoot.GetComponent<FanucJointLab>();
                var tcp=robot==0?scara.Tcp:fanuc.Tcp;
                var contact=tcp.Find("GripContact");
                if(contact==null){contact=new GameObject("GripContact").transform;contact.SetParent(tcp,false);}
                contact.localPosition=robot==0?new Vector3(0,-.006f,0):new Vector3(0,0,.16f);contact.localRotation=Quaternion.identity;contact.localScale=Vector3.one;
                var grip=robotRoot.GetComponent<RobotGripper>()??robotRoot.AddComponent<RobotGripper>();
                var s=new SerializedObject(grip);s.FindProperty("scara").objectReferenceValue=scara;s.FindProperty("fanuc").objectReferenceValue=fanuc;
                s.FindProperty("contact").objectReferenceValue=contact;
                var descendants=tcp.GetComponentsInChildren<Transform>();
                s.FindProperty("leftFinger").objectReferenceValue=robot==0?null:descendants.Single(t=>t.name=="LeftFinger");
                s.FindProperty("rightFinger").objectReferenceValue=robot==0?null:descendants.Single(t=>t.name=="RightFinger");
                s.FindProperty("fingerTravelMetres").floatValue=robot==0?0f:.0025f;s.ApplyModifiedPropertiesWithoutUndo();
                var lab=robotRoot.GetComponent<RobotGripperLab>()??robotRoot.AddComponent<RobotGripperLab>();
                s=new SerializedObject(lab);s.FindProperty("gripper").objectReferenceValue=grip;
                s.FindProperty("waypoints").objectReferenceValue=AssetDatabase.LoadAssetAtPath<RobotWaypointSet>(robot==0?
                    "Assets/04.ScriptableObject/ScaraLabWaypoints.asset":"Assets/04.ScriptableObject/FanucLabWaypoints.asset");
                s.FindProperty("trialPrefab").objectReferenceValue=AssetDatabase.LoadAssetAtPath<GameObject>(PartPrefabs[0]).GetComponent<GrippableItem>();
                s.ApplyModifiedPropertiesWithoutUndo();PrefabUtility.SaveAsPrefabAsset(robotRoot,RobotPrefabs[robot]);
            }
            finally{PrefabUtility.UnloadPrefabContents(robotRoot);}
        }
        AssetDatabase.SaveAssets();
        return Edit();
    }

    public static object Edit()
    {
        Require(!EditorApplication.isPlaying,"Use Edit Mode.");
        var labs=UnityEngine.Object.FindObjectsByType<RobotGripperLab>(FindObjectsSortMode.None);
        Require(labs.Length==2 && labs.All(l=>l.IsReady && l.TrialItem==null),"Expected two ready grip fixtures without spawned parts.");
        foreach(var lab in labs)
        {
            Require(!lab.TryPrepare(out _) && !lab.TryMoveToWork(out _) && !lab.Gripper.TryGrip(null,out _) && !lab.Gripper.TryRelease(out _),"Edit request accepted.");
        }
        foreach(string menu in new[]{"LineWatch/SCARA 4축 조작 테스트","LineWatch/FANUC 조작 테스트"})Require(EditorApplication.ExecuteMenuItem(menu),"Menu missing.");
        var roots=new[]{EditorWindow.GetWindow<LineWatch.Editor.ScaraJointLabWindow>().rootVisualElement,EditorWindow.GetWindow<LineWatch.Editor.FanucJointLabWindow>().rootVisualElement};
        foreach(var root in roots)Require(!root.Q<UnityEngine.UIElements.Button>("gripper-prepare").enabledSelf && !root.Q<UnityEngine.UIElements.Button>("gripper-grip").enabledSelf,"Edit panel not locked.");
        var report=new{passed=true,robots=2,editCommandsRejected=true,uiLocked=true,directButtonClicks=0};
        Directory.CreateDirectory("Logs/LW08");File.WriteAllText("Logs/LW08/gripper-edit-verification.json",JsonConvert.SerializeObject(report,Formatting.Indented));
        return report;
    }

    public static object ProbeBodyPose()
    {
        Require(EditorApplication.isPlaying,"Use Play Mode after a prepared trial.");
        var lab=UnityEngine.Object.FindObjectsByType<RobotGripperLab>(FindObjectsSortMode.None).First(l=>l.Gripper.Scara!=null);
        var item=lab.TrialItem;Require(item!=null && item.Owner==null,"Unowned trial required.");
        var original=item.Body.position;
        try
        {
            item.Body.position=original+Vector3.right*.01f;
            var visual=item.GripPoint.position;
            var physical=item.Body.position+item.Body.rotation*item.transform.InverseTransformPoint(item.GripPoint.position);
            var report=new{bodyPosition=V(item.Body.position),transformPosition=V(item.transform.position),visualGripPoint=V(visual),
                physicalGripPoint=V(physical),differenceMetres=Vector3.Distance(visual,physical)};
            Directory.CreateDirectory("Logs/LW08");File.WriteAllText("Logs/LW08/gripper-body-pose-probe.json",JsonConvert.SerializeObject(report,Formatting.Indented));
            return report;
        }
        finally{item.Body.position=original;}
    }

    public static async Task<object> Play()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused,"Use unpaused Play Mode.");
        var labs=UnityEngine.Object.FindObjectsByType<RobotGripperLab>(FindObjectsSortMode.None).OrderBy(l=>l.Gripper.Fanuc!=null).ToArray();
        Require(labs.Length==2 && labs.All(l=>l.IsReady && l.TrialItem==null && l.Gripper.HeldItem==null),"Fresh Play session required.");
        var token=labs[0].destroyCancellationToken; float originalScale=Time.timeScale;
        var paths=RobotPrefabs.Concat(PartPrefabs).Concat(new[]{"Assets/01.Scenes/RobotLab.unity","ProjectSettings/ProjectSettings.asset",
            "Assets/04.ScriptableObject/ScaraLabWaypoints.asset","Assets/04.ScriptableObject/FanucLabWaypoints.asset","Assets/09.Materials/GripTrial.mat",
            "Assets/02.Scripts/LineWatch/RobotGripper.cs","Assets/02.Scripts/LineWatch/GrippableItem.cs","Assets/02.Scripts/LineWatch/RobotGripperLab.cs"}).ToArray();
        var hashes=paths.ToDictionary(p=>p,Hash);
        int carryRuns=0,rejections=0,cleanups=0,samples=0,releaseHolds=0,heldStops=0;
        float maxFollowPosition=0,maxFollowRotation=0,maxReleaseDrift=0;
        var rejected=new List<object>();
        bool Stopped(RobotGripper g)=>g.IsStopped;
        bool RobotAtRest(RobotGripper g)=>g.Scara!=null?!g.Scara.IsTrajectoryMoving && !g.Scara.Axis1.IsMoving && !g.Scara.Axis2.IsMoving && !g.Scara.Axis3.IsMoving && !g.Scara.Axis4.IsMoving:
            !g.Fanuc.IsTcpMoving && g.Fanuc.Joints.All(j=>!j.IsMoving);
        string State(RobotGripperLab lab)=>JsonConvert.SerializeObject(new{held=lab.Gripper.HeldItem!=null?lab.Gripper.HeldItem.GetInstanceID():0,
            item=lab.TrialItem!=null?new{owner=lab.TrialItem.Owner!=null?lab.TrialItem.Owner.GetInstanceID():0,p=V(lab.TrialItem.Body.position),q=Q(lab.TrialItem.Body.rotation)}:null,
            version=lab.Gripper.Scara!=null?lab.Gripper.Scara.CommandVersion:lab.Gripper.Fanuc.CommandVersion});
        void Reject(RobotGripperLab lab,string name,Func<bool> request)
        {
            string before=State(lab);Require(!request() && before==State(lab),"Rejected request changed ownership/pose/commands: "+name);
            rejections++;rejected.Add(new{robot=lab.name,name});
        }
        async Task Ticks(int count)
        {double before=Time.fixedTimeAsDouble;await Until(()=>Time.fixedTimeAsDouble-before>=count*Time.fixedDeltaTime,token);}
        async Task Home(RobotGripper g)
        {if(g.Scara!=null)g.Scara.HomeAll();else g.Fanuc.HomeAll();await Until(()=>Stopped(g),token);await Ticks(2);}
        async Task Follow(RobotGripperLab lab,Vector3 localPosition,Quaternion localRotation)
        {
            var g=lab.Gripper;var item=lab.TrialItem;var watch=Stopwatch.StartNew();
            do
            {
                await Task.Delay(1,token);Require(watch.Elapsed.TotalSeconds<15,"Carry timed out.");
                Require(g.HeldItem==item && item.Owner==g,"Carry lost its unique owner.");
                float pe=Vector3.Distance(item.Body.position,g.Tcp.TransformPoint(localPosition));
                float re=RobotGripper.RotationError(item.Body.rotation,g.Tcp.rotation*localRotation);
                maxFollowPosition=Mathf.Max(maxFollowPosition,pe);maxFollowRotation=Mathf.Max(maxFollowRotation,re);
                Require(pe<=.0001f && re<=.005f,"Held body did not follow the measured TCP pose.");samples++;
            }while(!Stopped(g));
            await Ticks(2);carryRuns++;
        }
        try
        {
            Time.timeScale=10;
            for(int robot=0;robot<2;robot++)
            {
                var lab=labs[robot];var g=lab.Gripper;await Home(g);
                Reject(lab,"empty release",()=>g.TryRelease(out _));Reject(lab,"missing candidate",()=>g.TryGrip(null,out _));
                Require(lab.TryPrepare(out var error),error);await Ticks(2);
                var item=lab.TrialItem;
                Reject(lab,"far from contact",()=>g.TryGrip(item,out _));
                Require(lab.TryMoveToWork(out error),error);
                Reject(lab,"grip during robot motion",()=>g.TryGrip(item,out _));
                await Until(()=>Stopped(g),token);await Ticks(2);
                Vector3 position=item.Body.position;Quaternion rotation=item.Body.rotation;
                item.Body.position=position+Vector3.right*.01f;
                Reject(lab,"10mm contact error",()=>g.TryGrip(item,out _));item.Body.position=position;
                item.Body.rotation=Quaternion.AngleAxis(3,Vector3.up)*rotation;
                Reject(lab,"3 degree contact error",()=>g.TryGrip(item,out _));item.Body.rotation=rotation;
                item.Body.isKinematic=false;Reject(lab,"dynamic body unsupported",()=>g.TryGrip(item,out _));item.Body.isKinematic=true;
                var shape=item.GetComponent<BoxCollider>();shape.enabled=false;
                Reject(lab,"disabled collider",()=>g.TryGrip(item,out _));shape.enabled=true;
                var serialized=new SerializedObject(item);var point=item.GripPoint;
                serialized.FindProperty("gripPoint").objectReferenceValue=null;serialized.ApplyModifiedPropertiesWithoutUndo();
                Reject(lab,"missing grip point",()=>g.TryGrip(item,out _));
                serialized.FindProperty("gripPoint").objectReferenceValue=point;serialized.ApplyModifiedPropertiesWithoutUndo();
                EditorApplication.isPaused=true;Reject(lab,"editor paused grip",()=>g.TryGrip(item,out _));EditorApplication.isPaused=false;
                Time.timeScale=0;Reject(lab,"zero time scale grip",()=>g.TryGrip(item,out _));Time.timeScale=10;
                // Keep a real 1mm placement offset to prove acquisition does not snap the part.
                item.Body.position=position+g.Contact.right*.001f;await Ticks(2);
                var before=item.Body.position;var beforeRotation=item.Body.rotation;var parent=item.transform.parent;
                var fingers=g.Fanuc!=null?g.Tcp.GetComponentsInChildren<Transform>().Where(t=>t.name=="LeftFinger"||t.name=="RightFinger").OrderBy(t=>t.name).ToArray():new Transform[0];
                var fingerHomes=fingers.Select(t=>t.localPosition).ToArray();var fingerBefore=fingers.Select(t=>g.Tcp.InverseTransformPoint(t.position)).ToArray();
                Require(g.TryGrip(item,out error),error);
                Require(g.HeldItem==item && item.Owner==g && before.Equals(item.Body.position) && beforeRotation.Equals(item.Body.rotation) && item.transform.parent==parent,"Grip snapped/reparented the part.");
                if(fingers.Length==2)
                {
                    Require(Vector3.Distance(g.Tcp.InverseTransformPoint(fingers[0].position)-fingerBefore[0],new Vector3(0,-.0025f,0))<.000001f &&
                        Vector3.Distance(g.Tcp.InverseTransformPoint(fingers[1].position)-fingerBefore[1],new Vector3(0,.0025f,0))<.000001f,"FANUC finger travel mismatch.");
                }
                Reject(lab,"already holding",()=>g.TryGrip(item,out _));Reject(lab,"reset while held",()=>lab.TryPrepare(out _));
                string own=State(lab);var other=labs[1-robot].Gripper;
                Require(!other.TryGrip(item,out error) && error.Contains("다른 그리퍼") && own==State(lab) && other.HeldItem==null,"Second gripper acquired an owned part.");
                rejections++;rejected.Add(new{robot=lab.name,name="other gripper claim"});
                var relative=g.Tcp.InverseTransformPoint(item.Body.position);var relativeRotation=Quaternion.Inverse(g.Tcp.rotation)*item.Body.rotation;
                Require(lab.TryLift(out error),error);Reject(lab,"release during carry",()=>g.TryRelease(out _));
                await Ticks(2);g.Stop();await Ticks(2);var heldAtStop=item.Body.position;
                await Ticks(4);Require(g.IsStopped && item.Owner==g && g.HeldItem==item && Vector3.Distance(heldAtStop,item.Body.position)<.000001f,"Stop lost or moved the held part.");heldStops++;
                Require(lab.TryLift(out error),error);
                await Follow(lab,relative,relativeRotation);
                var releasedPosition=item.Body.position;var releasedRotation=item.Body.rotation;
                Require(g.TryRelease(out error) && g.HeldItem==null && item.Owner==null,error);
                Require(releasedPosition.Equals(item.Body.position) && releasedRotation.Equals(item.Body.rotation),"Release moved the part.");
                Require(fingers.Select((t,i)=>t.localPosition.Equals(fingerHomes[i])).All(v=>v),"Finger opening did not restore the authored pose.");
                Require(lab.TryMoveToWork(out error),error);await Until(()=>Stopped(g),token);await Ticks(4);
                float drift=Vector3.Distance(releasedPosition,item.Body.position);maxReleaseDrift=Mathf.Max(maxReleaseDrift,drift);
                Require(drift<.000001f && RobotGripper.RotationError(releasedRotation,item.Body.rotation)<.001f && item.Body.isKinematic && !item.Body.useGravity,"Released part followed/fell after robot motion.");releaseHolds++;
                Reject(lab,"released part remains at lift",()=>g.TryGrip(item,out _));
                Require(lab.TryPrepare(out error) && g.TryGrip(item,out error),error);
                Require(lab.TryLift(out error),error);await Ticks(2);Require(!RobotAtRest(g),"Disable fixture did not move.");
                var heldPosition=item.Body.position;g.enabled=false;
                Require(item.Owner==null && g.HeldItem==null && heldPosition.Equals(item.Body.position) && RobotAtRest(g),"Disabling gripper leaked ownership or motion.");
                g.enabled=true;await Ticks(2);Require(g.HeldItem==null,"Gripper auto-acquired on enable.");cleanups++;
                Require(g.TryGrip(item,out error),error);item.gameObject.SetActive(false);
                Require(g.HeldItem==null && item.Owner==null && !string.IsNullOrEmpty(g.LastError),"Disabled item leaked ownership.");
                item.gameObject.SetActive(true);await Ticks(2);cleanups++;
                Require(g.TryGrip(item,out error),error);item.Body.isKinematic=false;
                await Until(()=>g.HeldItem==null,token);Require(item.Owner==null && !string.IsNullOrEmpty(g.LastError),"Changed body profile kept ownership.");
                item.Body.isKinematic=true;await Ticks(2);cleanups++;
                Require(g.TryGrip(item,out error),error);Require(lab.TryLift(out error),error);await Ticks(2);
                Require(!RobotAtRest(g),"Loss fixture did not move.");UnityEngine.Object.Destroy(item.gameObject);
                await Until(()=>g.HeldItem==null,token);Require(!string.IsNullOrEmpty(g.LastError) && RobotAtRest(g),"Destroyed item did not stop/report loss.");cleanups++;
                Require(lab.TryPrepare(out error),error);await Ticks(2);
            }
            foreach(string menu in new[]{"LineWatch/SCARA 4축 조작 테스트","LineWatch/FANUC 조작 테스트"})Require(EditorApplication.ExecuteMenuItem(menu),"Menu missing.");
            await Task.Delay(150,token);
            var roots=new[]{EditorWindow.GetWindow<LineWatch.Editor.ScaraJointLabWindow>().rootVisualElement,EditorWindow.GetWindow<LineWatch.Editor.FanucJointLabWindow>().rootVisualElement};
            for(int i=0;i<2;i++)Require(roots[i].Q<Button>("gripper-prepare").enabledSelf && roots[i].Q<Button>("gripper-grip").enabledSelf &&
                !roots[i].Q<Button>("gripper-release").enabledSelf && roots[i].Q<UnityEditor.UIElements.ObjectField>("gripper-item").value==labs[i].TrialItem,"Grip panel binding/state incorrect.");
            Require(hashes.All(p=>Hash(p.Key)==p.Value),"Source code/assets changed.");
            var report=new{passed=true,utc=DateTime.UtcNow.ToString("o"),robots=2,carryRuns,rejectedRequestsPreserved=rejections,lifecycleCleanups=cleanups,
                followSamples=samples,maxFollowPositionErrorMetres=maxFollowPosition,maxFollowRotationErrorDegrees=maxFollowRotation,
                heldStops,releasedPartsHeld=releaseHolds,maxReleasedPositionDriftMetres=maxReleaseDrift,uniqueOwnerVerified=true,noSnapAndNoReparentVerified=true,fanucFingerTravelVerified=true,movingLossStopVerified=true,
                codeAndAssetsUnchanged=true,hashes,uiStateRead=true,directButtonClicks=0,
                scope="LW-08 process 3: manual API kinematic grip/contact alignment, one owner, rigidbody TCP following and release-in-place; 10x simulation; no automatic waypoint/grip sequence, contact forces, friction, gravity drop or industrial safety",rejected};
            Directory.CreateDirectory("Logs/LW08");File.WriteAllText("Logs/LW08/gripper-play-verification.json",JsonConvert.SerializeObject(report,Formatting.Indented));
            return new{report.passed,report.robots,report.carryRuns,report.rejectedRequestsPreserved,report.lifecycleCleanups,report.followSamples,
                report.maxFollowPositionErrorMetres,report.maxFollowRotationErrorDegrees,report.heldStops,report.releasedPartsHeld,report.maxReleasedPositionDriftMetres,report.codeAndAssetsUnchanged};
        }
        finally
        {
            EditorApplication.isPaused=false;Time.timeScale=originalScale;
            foreach(var lab in labs)
            {
                var g=lab.Gripper;g.enabled=true;
                if(g.Scara!=null)g.Scara.StopAll();else g.Fanuc.StopAll();
                if(g.HeldItem!=null)g.TryRelease(out _);
            }
        }
    }

    private static async Task Until(Func<bool> condition,CancellationToken token)
    {var watch=Stopwatch.StartNew();while(!condition()){token.ThrowIfCancellationRequested();Require(watch.Elapsed.TotalSeconds<15,"Verification wait timed out.");await Task.Delay(1,token);}}
    private static string Hash(string path){using(var sha=SHA256.Create())return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path)));}
    private static float[] Q(Quaternion q)=>new[]{q.x,q.y,q.z,q.w};
    private static float[] V(Vector3 v)=>new[]{v.x,v.y,v.z};
    private static void Require(bool condition,string message){if(!condition)throw new InvalidOperationException(message);}
}
