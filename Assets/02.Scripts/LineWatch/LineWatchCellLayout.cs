using UnityEngine;

namespace LineWatch
{
    // Authored cell references only. Product flow and coordinated operation are separate steps.
    public sealed class LineWatchCellLayout : MonoBehaviour
    {
        [SerializeField] private string scaraId = "SCARA-01";
        [SerializeField] private string fanucId = "FANUC-01";
        [SerializeField] private ScaraJointLab scara;
        [SerializeField] private FanucJointLab fanuc;
        [SerializeField] private Transform scaraZone;
        [SerializeField] private Transform fanucZone;
        [SerializeField] private Transform handoffZone;
        [SerializeField] private MeshFilter scaraFootprint;
        [SerializeField] private MeshFilter fanucFootprint;
        [SerializeField] private MeshFilter handoffFootprint;
        [SerializeField] private Transform inputStop;
        [SerializeField] private Transform handoffPoint;
        [SerializeField] private Transform handoffEquipment;
        [SerializeField] private Transform outputA;
        [SerializeField] private Transform outputB;
        [SerializeField] private GrippableItem productPrefab;
        [SerializeField] private RobotWaypointSet scaraHandoffWaypoints;
        [SerializeField] private RobotWaypointSet fanucHandoffWaypoints;

        public string ScaraId => scaraId;
        public string FanucId => fanucId;
        public ScaraJointLab Scara => scara;
        public FanucJointLab Fanuc => fanuc;
        public Transform ScaraZone => scaraZone;
        public Transform FanucZone => fanucZone;
        public Transform HandoffZone => handoffZone;
        public MeshFilter ScaraFootprint => scaraFootprint;
        public MeshFilter FanucFootprint => fanucFootprint;
        public MeshFilter HandoffFootprint => handoffFootprint;
        public Transform InputStop => inputStop;
        public Transform HandoffPoint => handoffPoint;
        public Transform HandoffEquipment => handoffEquipment;
        public Transform OutputA => outputA;
        public Transform OutputB => outputB;
        public GrippableItem ProductPrefab => productPrefab;
        public RobotWaypointSet ScaraHandoffWaypoints => scaraHandoffWaypoints;
        public RobotWaypointSet FanucHandoffWaypoints => fanucHandoffWaypoints;
    }
}
