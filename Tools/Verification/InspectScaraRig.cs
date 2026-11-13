using System;
using System.IO;
using System.Linq;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;

public static class InspectScaraRig
{
    public static object Run()
    {
        const string prefabPath = "Assets/03.Prefabs/ScaraRobot_Work.prefab";
        var root = PrefabUtility.LoadPrefabContents(prefabPath);
        try
        {
            var axis1 = root.transform.Find("Base/Axis1");
            var axis2 = axis1.Find("Axis2");
            var axis3 = axis2.Find("Axis3");
            var gripper = axis3.Find("Axis4/Gripper") ?? axis3.Find("Gripper");
            Vector3 firstLink = root.transform.InverseTransformVector(axis2.position - axis1.position);
            Vector3 secondLink = root.transform.InverseTransformVector(axis3.position - axis2.position);
            var mesh = gripper.GetComponent<MeshFilter>().sharedMesh;
            Vector3 gripperBoundsBottom = mesh.bounds.center;
            gripperBoundsBottom.y = mesh.bounds.min.y;

            var report = new
            {
                utc = DateTime.UtcNow.ToString("o"), prefabPath,
                purpose = "LW-04a authored geometry audit; no transforms or assets modified",
                coordinateNote = "Positions in prefab-root coordinates; pivot offsets measured from authored Axis1 home",
                rootPosition = Values(root.transform.position), rootScale = Values(root.transform.localScale),
                objects = root.GetComponentsInChildren<Transform>(true).Select(t => new
                {
                    path = AnimationUtility.CalculateTransformPath(t, root.transform),
                    localPosition = Values(t.localPosition), localEulerDegrees = Values(t.localEulerAngles),
                    localScale = Values(t.localScale),
                    pivotFromAxis1 = Values(root.transform.InverseTransformVector(t.position - axis1.position)),
                    localYInRoot = Values(root.transform.InverseTransformDirection(t.up)),
                    components = t.GetComponents<Component>().Select(c => c == null ? "MISSING" : c.GetType().FullName).ToArray()
                }).ToArray(),
                firstLink = new { delta = Values(firstLink), planarLength = PlanarLength(firstLink), spatialLength = firstLink.magnitude },
                secondLink = new { delta = Values(secondLink), planarLength = PlanarLength(secondLink), spatialLength = secondLink.magnitude },
                positiveYRotationOfX90Degrees = Values(Quaternion.AngleAxis(90f, Vector3.up) * Vector3.right),
                gripper = new
                {
                    meshPath = AssetDatabase.GetAssetPath(mesh),
                    localMeshBoundsCenter = Values(mesh.bounds.center), localMeshBoundsSize = Values(mesh.bounds.size),
                    localMeshBoundsMin = Values(mesh.bounds.min), localMeshBoundsMax = Values(mesh.bounds.max),
                    boundsBottomFromAxis3 = Values(root.transform.InverseTransformVector(
                        gripper.TransformPoint(gripperBoundsBottom) - axis3.position)),
                    note = "Bounds bottom is a geometry candidate, not a confirmed TCP contact point"
                },
                vendorBehaviours = root.GetComponentsInChildren<MonoBehaviour>(true)
                    .Count(c => c != null && (c.GetType().Namespace ?? "").StartsWith("realvirtual")),
                missingScripts = root.GetComponentsInChildren<Transform>(true)
                    .Sum(t => GameObjectUtility.GetMonoBehavioursWithMissingScriptCount(t.gameObject))
            };
            string directory = Path.GetFullPath("Logs/LW04");
            Directory.CreateDirectory(directory);
            File.WriteAllText(Path.Combine(directory, "rig-audit.json"), JsonConvert.SerializeObject(report, Formatting.Indented));
            return report;
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(root);
        }
    }

    private static float[] Values(Vector3 value) => new[] { value.x, value.y, value.z };
    private static float PlanarLength(Vector3 value) => Mathf.Sqrt(value.x * value.x + value.z * value.z);
}
