using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading.Tasks;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class VerifyCellLayout
{
    private const string ScenePath = "Assets/01.Scenes/LineWatch.unity";
    private const string ScaraPrefab = "Assets/03.Prefabs/ScaraRobot_Work.prefab";
    private const string FanucPrefab = "Assets/03.Prefabs/FanucRobot_Work.prefab";
    private const string PartPrefab = "Assets/03.Prefabs/HandoffTrialPart.prefab";
    private const string ScaraData = "Assets/04.ScriptableObject/ScaraCellHandoff.asset";
    private const string FanucData = "Assets/04.ScriptableObject/FanucCellHandoff.asset";
    private const string ConveyorModel = "Packages/io.realvirtual.starter/Assets/3DPrefabs/3DModels/ConveyorSmall.fbx";
    private static readonly string[] Protected = {
        "ProjectSettings/ProjectSettings.asset", "ProjectSettings/EditorBuildSettings.asset",
        ScaraPrefab, FanucPrefab, PartPrefab, "Assets/03.Prefabs/GripTrialPart.prefab",
        "Assets/04.ScriptableObject/ScaraLabWaypoints.asset", "Assets/04.ScriptableObject/FanucLabWaypoints.asset",
        "Assets/02.Scripts/LineWatch/GrippableItem.cs", "Assets/02.Scripts/LineWatch/RobotGripper.cs",
        "Assets/02.Scripts/LineWatch/RobotGripperLab.cs", "Assets/02.Scripts/LineWatch/RobotWaypointSequence.cs"
    };

    public static object Build()
    {
        Require(!EditorApplication.isPlaying && Enumerable.Range(0, SceneManager.sceneCount).All(i => !SceneManager.GetSceneAt(i).isDirty),
            "Stop Play and preserve unsaved scenes before building.");
        Require(!File.Exists(ScenePath) && !File.Exists(ScaraData) && !File.Exists(FanucData), "Cell already exists; do not overwrite authored work.");
        Preflight();
        var hashes = Protected.ToDictionary(p => p, Hash);
        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Additive);
        SceneManager.SetActiveScene(scene);
        var root = new GameObject("LineWatchCell");
        var layout = root.AddComponent<LineWatchCellLayout>();
        var stations = Child(root.transform, "Stations"); var equipment = Child(root.transform, "Equipment");
        var zones = Child(root.transform, "Zones"); var products = Child(root.transform, "Products");
        var floor = Material("CellFloor", new Color(.13f, .16f, .19f));
        var steel = Material("CellSteel", new Color(.42f, .49f, .54f));
        var belt = Material("CellBelt", new Color(.035f, .055f, .065f));
        var blue = Material("CellBlue", new Color(.16f, .39f, .67f));
        var amber = Material("CellAmber", new Color(.72f, .47f, .13f));
        var green = Material("CellGreen", new Color(.13f, .55f, .39f));
        Box("Floor", equipment, new Vector3(.1f, -.04f, -.2f), new Vector3(3.2f, .08f, 3.1f), floor);
        var sz = Child(zones, "Zone_SCARA"); var fz = Child(zones, "Zone_FANUC"); var hz = Child(zones, "Zone_Handoff");
        Box("SCARA_Footprint", sz, new Vector3(-.37f, .002f, .48f), new Vector3(.7f, .004f, .7f), blue, false);
        Box("FANUC_Footprint", fz, new Vector3(-.15f, .002f, -.85f), new Vector3(.85f, .004f, 1.1f), amber, false);
        Box("Handoff_Footprint", hz, new Vector3(0, .004f, 0), new Vector3(.54f, .004f, .42f), green, false);
        var s = ((GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(ScaraPrefab), scene)).GetComponent<ScaraJointLab>();
        var f = ((GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(FanucPrefab), scene)).GetComponent<FanucJointLab>();
        s.name = "SCARA-01"; f.name = "FANUC-01"; s.transform.SetParent(sz, true); f.transform.SetParent(fz, true);
        var sw = AssetDatabase.LoadAssetAtPath<RobotWaypointSet>(Protected[7]).Work;
        var fw = FanucForwardKinematics.Calculate(new[] { 0f, -20f, 20f, 0f, 20f, 0f }, f.Geometry);
        Place(s.GetComponent<RobotGripper>(), new Pose(sw.PositionMetres, sw.Rotation), Quaternion.identity);
        Place(f.GetComponent<RobotGripper>(), new Pose(fw.PositionMetres, fw.Rotation), Quaternion.Euler(0, 180, 0));
        Pedestal(s.gameObject, equipment, steel); Pedestal(f.gameObject, equipment, steel);
        var inputPose = ScaraForwardKinematics.Calculate(-20, 60, .1f, -40, s.Geometry);
        var sg = s.GetComponent<RobotGripper>(); var fg = f.GetComponent<RobotGripper>();
        var inputWorld = s.RobotFrame.TransformPoint(inputPose.PositionMetres) + s.RobotFrame.rotation * Quaternion.Euler(0, inputPose.YawDegrees, 0) * s.Tcp.InverseTransformPoint(sg.Contact.position);
        var input = Marker(stations, "Input_Stop", inputWorld);
        var handoff = Marker(stations, "Handoff_Point", new Vector3(0, 1.2f, 0));
        var outputA = Marker(stations, "Output_A", new Vector3(.08f, 1.2f, -.04f));
        var outputB = Marker(stations, "Output_B", new Vector3(-.08f, 1.2f, -.04f));
        Infeed(equipment, inputWorld, steel, belt);
        Box("TransferBench", equipment, new Vector3(0, 1.09f, -.02f), new Vector3(.52f, .04f, .38f), steel);
        foreach (float x in new[] { -.22f, .22f }) foreach (float z in new[] { -.17f, .13f })
            Box("BenchLeg", equipment, new Vector3(x, .535f, z), new Vector3(.025f, 1.07f, .025f), steel);
        foreach (var point in new[] { handoff, outputA, outputB })
        {
            var color = point == handoff ? amber : point == outputA ? blue : green;
            Box(point.name + "_Seat", equipment, point.position - Vector3.up * .008f, new Vector3(.008f, .004f, .008f), color);
            Box(point.name + "_Stem", equipment, new Vector3(point.position.x, 1.15f, point.position.z), new Vector3(.006f, .08f, .006f), steel);
        }
        var part = (GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(PartPrefab), scene);
        part.name = "Input_Sample"; part.transform.SetParent(products, false); part.transform.SetPositionAndRotation(input.position, Quaternion.identity);
        var ss = Waypoints(ScaraData, RobotWaypointSet.RobotType.Scara, new Pose(sw.PositionMetres, sw.Rotation), .07f, new Vector3(.02f, 0, .02f));
        var fs = Waypoints(FanucData, RobotWaypointSet.RobotType.Fanuc, new Pose(fw.PositionMetres, fw.Rotation), .06f, new Vector3(.03f, 0, .02f));
        for (int i = 0; i < 2; i++)
        {
            var robot = i == 0 ? s.gameObject : f.gameObject; var data = i == 0 ? ss : fs;
            SetObject(robot.GetComponent<RobotWaypointSequence>(), "waypoints", data);
            SetObject(robot.GetComponent<RobotGripperLab>(), "waypoints", data);
            SetObject(robot.GetComponent<RobotGripperLab>(), "trialPrefab", AssetDatabase.LoadAssetAtPath<GameObject>(PartPrefab).GetComponent<GrippableItem>());
        }
        SetObject(layout, "scara", s); SetObject(layout, "fanuc", f); SetObject(layout, "scaraZone", sz); SetObject(layout, "fanucZone", fz); SetObject(layout, "handoffZone", hz);
        SetObject(layout, "inputStop", input); SetObject(layout, "handoffPoint", handoff); SetObject(layout, "outputA", outputA); SetObject(layout, "outputB", outputB);
        SetObject(layout, "productPrefab", AssetDatabase.LoadAssetAtPath<GameObject>(PartPrefab).GetComponent<GrippableItem>());
        SetObject(layout, "scaraHandoffWaypoints", ss); SetObject(layout, "fanucHandoffWaypoints", fs);
        var camera = Child(root.transform, "Main Camera").gameObject.AddComponent<Camera>(); camera.tag = "MainCamera";
        camera.transform.position = new Vector3(2.5f, 2.3f, 3); camera.transform.LookAt(new Vector3(0, .86f, -.2f));
        camera.orthographic = true; camera.orthographicSize = 1.65f; camera.nearClipPlane = .03f; camera.farClipPlane = 30;
        camera.clearFlags = CameraClearFlags.SolidColor; camera.backgroundColor = new Color(.055f, .075f, .10f); camera.gameObject.AddComponent<AudioListener>();
        var light = Child(root.transform, "Key Light").gameObject.AddComponent<Light>(); light.type = LightType.Directional; light.intensity = 1.8f;
        light.shadows = LightShadows.Soft; light.transform.rotation = Quaternion.Euler(45, -35, 0);
        RenderSettings.skybox = null; RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat; RenderSettings.ambientLight = new Color(.65f, .69f, .74f);
        Label(root.transform, "SCARA-01", s.transform.position + Vector3.up * .7f, camera, blue);
        Label(root.transform, "FANUC-01", f.transform.position + Vector3.up * 1.35f, camera, amber);
        Label(root.transform, "INFEED", input.position + new Vector3(.36f, .20f, 0), camera, steel);
        Label(root.transform, "HANDOFF", handoff.position + new Vector3(0, .18f, 0), camera, amber);
        Label(root.transform, "OUT A", outputA.position + new Vector3(.12f, -.22f, .10f), camera, blue);
        Label(root.transform, "OUT B", outputB.position + new Vector3(-.12f, -.22f, .10f), camera, green);
        Presentation(layout);
        VerifyLayout(layout);
        Require(EditorSceneManager.SaveScene(scene, ScenePath), "Scene save failed.");
        Require(hashes.All(p => Hash(p.Key) == p.Value), "Protected source changed.");
        var viewRotation = camera.transform.rotation;
        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        Selection.activeGameObject = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>().gameObject;
        var view = SceneView.lastActiveSceneView;
        if (view != null) { view.LookAt(new Vector3(0, .85f, -.2f), viewRotation, 2.3f); view.Repaint(); }
        var result = Edit();
        Write("cell-build-verification.json", new { passed = true, protectedSourcesUnchanged = true, hashes, buildSettingsUnchanged = true, scene = ScenePath });
        return result;
    }

    public static object Preflight()
    {
        var s = AssetDatabase.LoadAssetAtPath<GameObject>(ScaraPrefab).GetComponent<ScaraJointLab>();
        var f = AssetDatabase.LoadAssetAtPath<GameObject>(FanucPrefab).GetComponent<FanucJointLab>();
        var sw = AssetDatabase.LoadAssetAtPath<RobotWaypointSet>(Protected[7]).Work;
        var input = ScaraForwardKinematics.Calculate(-20, 60, .1f, -40, s.Geometry);
        var fw = FanucForwardKinematics.Calculate(new[] { 0f, -20f, 20f, 0f, 20f, 0f }, f.Geometry);
        float margin = float.MaxValue;
        foreach (var p in new[] { input.PositionMetres, sw.PositionMetres }) foreach (float h in new[] { 0f, .07f })
            CheckIk(s.GetComponent<RobotGripper>(), p + Vector3.up * h, sw.Rotation, ref margin);
        CheckIk(s.GetComponent<RobotGripper>(), sw.PositionMetres + new Vector3(.02f, .07f, .02f), sw.Rotation, ref margin);
        foreach (var offset in new[] { Vector3.zero, new Vector3(-.08f, 0, .04f), new Vector3(.08f, 0, .04f) }) foreach (float h in new[] { 0f, .06f })
            CheckIk(f.GetComponent<RobotGripper>(), fw.PositionMetres + offset + Vector3.up * h, fw.Rotation, ref margin);
        CheckIk(f.GetComponent<RobotGripper>(), fw.PositionMetres + new Vector3(.03f, .06f, .02f), fw.Rotation, ref margin);
        return JsonConvert.SerializeObject(new { passed = true, targets = 12, minimumFanucEndpointMarginDegrees = margin });
    }

    public static object Edit()
    {
        Require(!EditorApplication.isPlaying, "Use Edit Mode.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(layout != null && layout.gameObject.scene.path == ScenePath, "Open LineWatch scene.");
        var report = VerifyLayout(layout); Write("cell-edit-verification.json", report); return JsonConvert.SerializeObject(report);
    }

    public static object UpdatePresentation()
    {
        Require(!EditorApplication.isPlaying, "Use Edit Mode.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(layout != null && layout.gameObject.scene.path == ScenePath, "Open LineWatch scene.");
        Presentation(layout); EditorSceneManager.MarkSceneDirty(layout.gameObject.scene);
        Require(EditorSceneManager.SaveScene(layout.gameObject.scene), "Scene save failed.");
        return Edit();
    }

    private static void Presentation(LineWatchCellLayout layout)
    {
        foreach (var text in layout.GetComponentsInChildren<TextMesh>()) text.characterSize = .014f;
        var anchor = layout.transform.Find("Equipment/InfeedConveyor");
        if (anchor.Find("InputStopGuide") == null)
            Box("InputStopGuide", anchor, layout.InputStop.position + new Vector3(-.012f, .004f, 0), new Vector3(.008f, .02f, .14f),
                AssetDatabase.LoadAssetAtPath<Material>("Assets/09.Materials/CellAmber.mat"));
    }

    public static async Task<object> PlaySmoke()
    {
        Require(EditorApplication.isPlaying && !EditorApplication.isPaused, "Use unpaused LineWatch Play Mode.");
        var layout = UnityEngine.Object.FindFirstObjectByType<LineWatchCellLayout>();
        Require(layout != null && layout.gameObject.scene.path == ScenePath, "Open LineWatch scene.");
        VerifyLayout(layout);
        var s = layout.Scara; var f = layout.Fanuc;
        var g = new[] { s.GetComponent<RobotGripper>(), f.GetComponent<RobotGripper>() };
        Require(g.All(v => v.IsReady && v.GetComponent<RobotGripperLab>().IsReady), "Robot runtime wiring failed.");
        var sample = layout.GetComponentsInChildren<GrippableItem>().Single();
        Require(sample.IsReady && sample.Owner == null, "Sample product not ready.");
        var position = sample.Body.position; var rotation = sample.Body.rotation; int frame = Time.frameCount;
        await Task.Delay(300);
        Require(!s.IsTrajectoryMoving && !s.Axis1.IsMoving && !s.Axis2.IsMoving && !s.Axis3.IsMoving && !s.Axis4.IsMoving &&
            !f.IsTcpMoving && f.Joints.All(j => !j.IsMoving), "Robot moved automatically on Play.");
        Require(g.All(v => v.HeldItem == null && v.GetComponent<RobotWaypointSequence>().State == RobotWaypointSequence.RunState.Idle &&
            v.GetComponent<RobotGripperLab>().TrialItem == null), "Automatic operation or trial spawn occurred on Play.");
        Require(sample.Body.position.Equals(position) && sample.Body.rotation.Equals(rotation) && Time.frameCount > frame, "Idle sample drifted or game loop stalled.");
        Require(s.TryReadFkDiagnostics(out var sd, out var se), se);
        Require(f.TryReadFkDiagnostics(out var fd, out var fe), fe);
        Require(sd.PositionErrorMetres < .00001f && sd.YawErrorDegrees < .005f && fd.PositionErrorMetres < .00001f && fd.RotationErrorDegrees < .005f,
            "Cell placement changed FK/model calibration.");
        var report = new { passed = true, robotsReady = 2, idleSequences = 2, sampleProducts = 1, observedFrames = Time.frameCount - frame,
            automaticMoves = 0, automaticTrialSpawns = 0, defaultFkCalibrationPassed = true, directButtonClicks = 0,
            scope = "Play-entry idle/wiring smoke only. Movement controls and coordinated handoff remain later verification." };
        Write("cell-play-smoke.json", report); return JsonConvert.SerializeObject(report);
    }

    private static object VerifyLayout(LineWatchCellLayout layout)
    {
        Require(layout.Scara != null && layout.Fanuc != null && layout.ScaraId == "SCARA-01" && layout.FanucId == "FANUC-01", "Robot identities missing.");
        Require(layout.ScaraZone != null && layout.FanucZone != null && layout.HandoffZone != null && layout.ProductPrefab != null, "Zone/product references missing.");
        Require(layout.Scara.transform.IsChildOf(layout.ScaraZone) && layout.Fanuc.transform.IsChildOf(layout.FanucZone), "Robot zone ownership mismatch.");
        var points = new[] { layout.InputStop, layout.HandoffPoint, layout.OutputA, layout.OutputB };
        Require(points.All(p => p != null && p.IsChildOf(layout.transform)) && points.Distinct().Count() == 4, "Station references missing.");
        var components = layout.GetComponentsInChildren<MonoBehaviour>(true);
        Require(components.All(c => c != null && (c.GetType().Namespace == "LineWatch" || c.GetType().Namespace == "UnityEngine.Rendering.Universal")), "Missing or vendor behavior in cell.");
        var targets = new List<object>(); float minFanucMargin = float.MaxValue;
        for (int robot = 0; robot < 2; robot++)
        {
            var g = robot == 0 ? layout.Scara.GetComponent<RobotGripper>() : layout.Fanuc.GetComponent<RobotGripper>();
            var data = robot == 0 ? layout.ScaraHandoffWaypoints : layout.FanucHandoffWaypoints;
            Require(data != null && data.TryValidate(out _), "Invalid cell waypoint asset.");
            Require(new SerializedObject(g.GetComponent<RobotWaypointSequence>()).FindProperty("waypoints").objectReferenceValue == data &&
                new SerializedObject(g.GetComponent<RobotGripperLab>()).FindProperty("waypoints").objectReferenceValue == data &&
                new SerializedObject(g.GetComponent<RobotGripperLab>()).FindProperty("trialPrefab").objectReferenceValue == layout.ProductPrefab,
                "Scene-only waypoint/product override missing.");
            var kind = robot == 0 ? RobotWaypointSet.RobotType.Scara : RobotWaypointSet.RobotType.Fanuc;
            var stations = robot == 0 ? new[] { layout.InputStop, layout.HandoffPoint } : new[] { layout.HandoffPoint, layout.OutputA, layout.OutputB };
            foreach (var station in stations)
            {
                var tcp = ItemTcp(g, layout.ProductPrefab.GetGripPoint(kind), station);
                foreach (float height in new[] { 0f, robot == 0 ? .07f : .06f })
                {
                    var p = tcp.position + Vector3.up * height;
                    CheckIk(g, p, tcp.rotation, ref minFanucMargin);
                    targets.Add(new { robot = robot == 0 ? layout.ScaraId : layout.FanucId, station = station.name, height, localTcp = V(p) });
                }
                if (station == layout.HandoffPoint) Require(Vector3.Distance(tcp.position, data.Work.PositionMetres) < .00001f &&
                    RobotGripper.RotationError(tcp.rotation, data.Work.Rotation) < .005f, "Shared product pose / cell waypoint mismatch.");
            }
            CheckIk(g, data.Retreat.PositionMetres, data.Retreat.Rotation, ref minFanucMargin);
        }
        return new { passed = true, scene = ScenePath, robots = 2, zones = 3, productStations = 4, ikTargets = 12,
            minimumFanucEndpointMarginDegrees = minFanucMargin, missingScripts = 0, vendorBehaviors = 0,
            sampleProductCount = layout.GetComponentsInChildren<GrippableItem>(true).Length, targets,
            scope = "LW-09 process 3: scene layout, station pose IK and references; no conveyor motion, queue, coordinated handoff, collision-free route or force validation" };
    }

    private static void CheckIk(RobotGripper g, Vector3 p, Quaternion rotation, ref float margin)
    {
        if (g.Scara != null)
        {
            var s = g.Scara;
            var limits = new ScaraInverseKinematics.JointLimits(new ScaraInverseKinematics.JointValues(s.Axis1.MinimumDegrees, s.Axis2.MinimumDegrees, s.Axis3.MinimumMetres, s.Axis4.MinimumDegrees),
                new ScaraInverseKinematics.JointValues(s.Axis1.MaximumDegrees, s.Axis2.MaximumDegrees, s.Axis3.MaximumMetres, s.Axis4.MaximumDegrees));
            var result = ScaraInverseKinematics.Solve(p, Vector3.SignedAngle(Vector3.forward, rotation * Vector3.forward, Vector3.up), s.Geometry, limits);
            Require(result.Success && result.Solutions.Any(v => v.ElbowSign == 1), "SCARA station IK failed: " + result.Failure);
        }
        else
        {
            var f = g.Fanuc; var lower = f.Joints.Select(j => j.MinimumDegrees).ToArray(); var upper = f.Joints.Select(j => j.MaximumDegrees).ToArray();
            var result = FanucInverseKinematics.Solve(p, rotation, f.Geometry, lower, upper, new[] { new float[6], new[] { 10f, -15f, 25f, 15f, 20f, -10f }, new[] { -10f, 15f, -25f, -15f, -20f, 10f } });
            Require(result.Succeeded, "FANUC station IK failed at " + p + ": " + result.Reason);
            for (int i = 0; i < 6; i++) margin = Mathf.Min(margin, result.JointDegrees[i] - lower[i], upper[i] - result.JointDegrees[i]);
        }
    }

    private static Pose ItemTcp(RobotGripper g, Transform point, Transform station)
    {
        var worldRotation = station.rotation * point.localRotation * Quaternion.Inverse(Quaternion.Inverse(g.Tcp.rotation) * g.Contact.rotation);
        var worldPosition = station.TransformPoint(point.localPosition) - worldRotation * g.Tcp.InverseTransformPoint(g.Contact.position);
        return new Pose(g.RobotFrame.InverseTransformPoint(worldPosition), Quaternion.Inverse(g.RobotFrame.rotation) * worldRotation);
    }

    private static void Place(RobotGripper g, Pose tcp, Quaternion rotation)
    {
        var root = g.transform; var offset = root.InverseTransformPoint(g.RobotFrame.position);
        var contact = tcp.position + tcp.rotation * g.Tcp.InverseTransformPoint(g.Contact.position);
        root.SetPositionAndRotation(new Vector3(0, 1.2f, 0) - rotation * (offset + contact), rotation);
    }

    private static RobotWaypointSet Waypoints(string path, RobotWaypointSet.RobotType kind, Pose work, float lift, Vector3 retreat)
    {
        var data = ScriptableObject.CreateInstance<RobotWaypointSet>(); var serialized = new SerializedObject(data);
        serialized.FindProperty("robot").enumValueIndex = (int)kind;
        foreach (string name in new[] { "approach", "work", "retreat" })
        {
            var target = serialized.FindProperty(name);
            target.FindPropertyRelative("positionMetres").vector3Value = work.position + (name == "work" ? Vector3.zero : Vector3.up * lift) + (name == "retreat" ? retreat : Vector3.zero);
            target.FindPropertyRelative("rotation").quaternionValue = work.rotation;
        }
        serialized.ApplyModifiedPropertiesWithoutUndo(); Require(data.TryValidate(out var error), error); AssetDatabase.CreateAsset(data, path); return data;
    }

    private static void Pedestal(GameObject robot, Transform parent, Material material)
    {
        var bounds = BoundsInRoot(robot); float top = robot.transform.position.y + bounds.min.y;
        Require(top > 0, "Robot base below floor.");
        Box(robot.name + "_Pedestal", parent, new Vector3(robot.transform.position.x, top * .5f, robot.transform.position.z), new Vector3(.32f, top, .30f), material);
    }

    private static void Infeed(Transform parent, Vector3 point, Material steel, Material belt)
    {
        var anchor = Child(parent, "InfeedConveyor"); var center = point + new Vector3(.35f, -.116f, 0);
        var visual = UnityEngine.Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>(ConveyorModel), anchor);
        visual.name = "Conveyor_ModelOnly";
        foreach (var component in visual.GetComponentsInChildren<Component>(true))
            if (!(component is Transform) && !(component is MeshFilter) && !(component is MeshRenderer)) UnityEngine.Object.DestroyImmediate(component);
        var bounds = BoundsInRoot(visual); var scale = new Vector3(.8f / bounds.size.x, .22f / bounds.size.y, .18f / bounds.size.z);
        visual.transform.localRotation = Quaternion.identity; visual.transform.localScale = scale; visual.transform.localPosition = center - Vector3.Scale(scale, bounds.center);
        foreach (var renderer in visual.GetComponentsInChildren<MeshRenderer>()) renderer.sharedMaterials = renderer.sharedMaterials.Select(_ => steel).ToArray();
        Box("BeltDeck", anchor, point + new Vector3(.35f, -.012f, 0), new Vector3(.8f, .012f, .16f), belt);
        float height = point.y - .226f;
        foreach (float x in new[] { point.x + .03f, point.x + .67f }) foreach (float z in new[] { point.z - .065f, point.z + .065f })
            Box("ConveyorSupport", anchor, new Vector3(x, height * .5f, z), new Vector3(.025f, height, .025f), steel);
    }

    private static Material Material(string name, Color color)
    {
        string path = "Assets/09.Materials/" + name + ".mat";
        var existing = AssetDatabase.LoadAssetAtPath<Material>(path); if (existing != null) return existing;
        var material = new Material(Shader.Find("Universal Render Pipeline/Lit")) { name = name, color = color };
        material.SetFloat("_Smoothness", .25f); AssetDatabase.CreateAsset(material, path); return material;
    }
    private static Transform Child(Transform parent, string name) { var t = new GameObject(name).transform; t.SetParent(parent, false); return t; }
    private static Transform Marker(Transform parent, string name, Vector3 position) { var t = Child(parent, name); t.position = position; return t; }
    private static void Box(string name, Transform parent, Vector3 position, Vector3 size, Material material, bool collider = true)
    {
        var box = GameObject.CreatePrimitive(PrimitiveType.Cube); box.name = name; box.transform.SetParent(parent, false);
        box.transform.position = position; box.transform.localScale = size; box.GetComponent<Renderer>().sharedMaterial = material;
        if (!collider) UnityEngine.Object.DestroyImmediate(box.GetComponent<Collider>());
    }
    private static void Label(Transform parent, string text, Vector3 position, Camera camera, Material color)
    {
        var t = Child(parent, "Label_" + text); t.position = position; t.rotation = camera.transform.rotation;
        var label = t.gameObject.AddComponent<TextMesh>(); label.text = text; label.anchor = TextAnchor.MiddleCenter; label.alignment = TextAlignment.Center;
        label.fontSize = 64; label.characterSize = .014f; label.color = Color.Lerp(color.color, Color.white, .5f);
        label.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf"); label.GetComponent<MeshRenderer>().sharedMaterial = label.font.material;
    }
    private static void SetObject(UnityEngine.Object target, string field, UnityEngine.Object value)
    { var serialized = new SerializedObject(target); serialized.FindProperty(field).objectReferenceValue = value; serialized.ApplyModifiedPropertiesWithoutUndo(); }
    private static string Hash(string path) { using (var sha = SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))); }
    private static void Write(string file, object report) { Directory.CreateDirectory("Logs/LW09"); File.WriteAllText("Logs/LW09/" + file, JsonConvert.SerializeObject(report, Formatting.Indented)); }

    public static object Inspect()
    {
        Require(!EditorApplication.isPlaying, "Use Edit Mode.");
        var assets = AssetDatabase.FindAssets("t:Model").Concat(AssetDatabase.FindAssets("t:Prefab"))
            .Select(AssetDatabase.GUIDToAssetPath).Where(p => p.IndexOf("convey", StringComparison.OrdinalIgnoreCase) >= 0 ||
                p.IndexOf("roller", StringComparison.OrdinalIgnoreCase) >= 0 || p.IndexOf("belt", StringComparison.OrdinalIgnoreCase) >= 0)
            .Distinct().Take(18).ToArray();
        var scenes = Enumerable.Range(0, SceneManager.sceneCount).Select(i => SceneManager.GetSceneAt(i))
            .Select(s => new { s.path, s.isDirty }).ToArray();
        return JsonConvert.SerializeObject(new { scenes, assets, lineWatchSceneExists = File.Exists("Assets/01.Scenes/LineWatch.unity") });
    }

    public static object InspectGeometry()
    {
        var paths = new[] { "Packages/io.realvirtual.starter/Assets/3DPrefabs/3DModels/ConveyorSmall.fbx",
            "Assets/03.Prefabs/ScaraRobot_Work.prefab", "Assets/03.Prefabs/FanucRobot_Work.prefab" };
        return JsonConvert.SerializeObject(paths.Select(path => {
            var root = AssetDatabase.LoadAssetAtPath<GameObject>(path);
            var b = BoundsInRoot(root);
            return new { path, meshes = root.GetComponentsInChildren<MeshFilter>(true).Length,
                min = V(b.min), max = V(b.max), size = V(b.size) };
        }));
    }

    private static Bounds BoundsInRoot(GameObject root)
    {
        var meshes = root.GetComponentsInChildren<MeshFilter>(true);
        Require(meshes.Length > 0, "No model meshes.");
        Bounds? result = null;
        foreach (var mesh in meshes)
        {
            var b = mesh.sharedMesh.bounds;
            for (int i = 0; i < 8; i++)
            {
                var p = root.transform.InverseTransformPoint(mesh.transform.TransformPoint(b.center + Vector3.Scale(b.extents,
                    new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1))));
                if (result == null) result = new Bounds(p, Vector3.zero); else { var value = result.Value; value.Encapsulate(p); result = value; }
            }
        }
        return result.Value;
    }

    private static float[] V(Vector3 value) => new[] { value.x, value.y, value.z };

    private static void Require(bool value, string message) { if (!value) throw new InvalidOperationException(message); }
}
