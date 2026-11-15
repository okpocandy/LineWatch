using System;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using LineWatch;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;

public static class InspectFanucIkReferenceGoals
{
    private const float DifferenceStepRadians = .001f;
    private const double RankRelativeTolerance = 1e-6;
    private const float LengthScaleMetres = .5f;

    public static object Run()
    {
        Require(!EditorApplication.isPlaying,"Inspect IK foundations in Edit Mode.");
        const string prefabPath="Assets/03.Prefabs/FanucRobot_Work.prefab";
        var paths=new[]{prefabPath,"Assets/01.Scenes/RobotLab.unity","Assets/03.Prefabs/ScaraRobot_Work.prefab","ProjectSettings/ProjectSettings.asset"};
        var hashes=paths.ToDictionary(p=>p,Hash);
        var root=PrefabUtility.LoadPrefabContents(prefabPath);
        try
        {
            var lab=root.GetComponent<FanucJointLab>();
            Require(lab!=null && lab.RobotFrame==root.transform,"Missing saved FANUC geometry/frame.");
            var geometry=lab.Geometry;
            var homes=lab.Joints.Select(j=>j.transform.localRotation).ToArray();
            var known=new[]{new float[6],new[]{1f,-1f,2f,-1f,1f,2f},new[]{15f,-20f,25f,-15f,20f,10f},
                new[]{-20f,15f,-25f,20f,-15f,-10f},new[]{44f,-40f,35f,-30f,25f,-40f},new[]{30f,-10f,20f,35f,-30f,15f}};
            var names=new[]{"Home exact control","Near home","Ordinary positive","Ordinary negative","Near simulation bounds","Mixed wrist"};
            var seeds=new[]{new float[6],new[]{10f,-15f,25f,15f,20f,-10f},new[]{-10f,15f,-25f,-15f,-20f,10f}};
            Require(known.Concat(seeds).All(q=>q.Select((v,i)=>v>=lab.Joints[i].MinimumDegrees && v<=lab.Joints[i].MaximumDegrees).All(v=>v)),"Fixture outside simulation limits.");
            float maxLinearDifference=0,maxAngularDifference=0,maxModelPosition=0,maxModelRotation=0;
            var goals=known.Select((q,index)=>
            {
                var pose=FanucForwardKinematics.Calculate(q,geometry);
                for(int i=0;i<6;i++) lab.Joints[i].transform.localRotation=homes[i]*Quaternion.AngleAxis(q[i],lab.Joints[i].LocalRotationAxis);
                float modelPosition=Vector3.Distance(pose.PositionMetres,lab.RobotFrame.InverseTransformPoint(lab.Tcp.position));
                float modelRotation=RotationVector(pose.Rotation,Quaternion.Inverse(lab.RobotFrame.rotation)*lab.Tcp.rotation).magnitude*Mathf.Rad2Deg;
                Require(modelPosition<.00001f && modelRotation<.005f,"Known feasible target disagrees with isolated model.");
                maxModelPosition=Mathf.Max(maxModelPosition,modelPosition); maxModelRotation=Mathf.Max(maxModelRotation,modelRotation);
                var jacobian=Jacobian(q,geometry);
                for(int column=0;column<6;column++)
                {
                    var plus=(float[])q.Clone(); var minus=(float[])q.Clone();
                    plus[column]+=DifferenceStepRadians*Mathf.Rad2Deg; minus[column]-=DifferenceStepRadians*Mathf.Rad2Deg;
                    var a=FanucForwardKinematics.Calculate(plus,geometry); var b=FanucForwardKinematics.Calculate(minus,geometry);
                    var dp=(a.PositionMetres-b.PositionMetres)/(2*DifferenceStepRadians);
                    var dw=RotationVector(a.Rotation,b.Rotation)/(2*DifferenceStepRadians);
                    for(int row=0;row<3;row++)
                    {
                        maxLinearDifference=Mathf.Max(maxLinearDifference,Mathf.Abs((float)jacobian[row,column]-dp[row]));
                        maxAngularDifference=Mathf.Max(maxAngularDifference,Mathf.Abs((float)jacobian[row+3,column]-dw[row]));
                    }
                }
                return new {name=names[index],targetPositionMetres=V(pose.PositionMetres),targetRotationXyzw=Q(pose.Rotation),
                    verificationOnlyKnownJointDegrees=q,estimatedJacobianRank=Rank(jacobian),
                    modelPositionErrorMetres=modelPosition,modelRotationErrorDegrees=modelRotation};
            }).ToArray();
            Require(maxLinearDifference<.001f && maxAngularDifference<.001f,"Geometric Jacobian disagrees with central FK differences.");
            Require(goals[0].estimatedJacobianRank==5 && goals.Any(g=>g.estimatedJacobianRank==6),"Unexpected home or ordinary-pose Jacobian rank.");
            int residualChecks=CheckRotationResiduals();
            double reachBound=geometry.Joints.Sum(j=>(double)j.PositionMetres.magnitude)+geometry.ToolPositionMetres.magnitude;
            var far=new Vector3(3,0,0);
            Require(far.magnitude>reachBound+.00001,"Far target did not exceed conservative outer reach bound.");
            bool unchanged=hashes.All(p=>Hash(p.Key)==p.Value);
            Require(unchanged,"A source asset changed during IK reference inspection.");
            var report=new {passed=true,utc=DateTime.UtcNow.ToString("o"),knownFeasibleTargetCount=goals.Length,
                jacobianColumnsChecked=goals.Length*6,finiteDifferenceStepRadians=DifferenceStepRadians,
                maxLinearJacobianDifferenceMetresPerRadian=maxLinearDifference,maxAngularJacobianDifference=maxAngularDifference,
                maxModelPositionErrorMetres=maxModelPosition,maxModelRotationErrorDegrees=maxModelRotation,
                homeJacobianRank=goals[0].estimatedJacobianRank,rankRelativeTolerance=RankRelativeTolerance,
                rankPositionScaleMetres=LengthScaleMetres,rotationResidualChecks=residualChecks,
                conservativeOuterReachBoundMetres=reachBound,provenOutsideBoundTargetMetres=V(far),
                seeds=seeds.Select((q,i)=>new{name=i==0?"Home":"Fixed alternative "+i,jointDegrees=q,estimatedJacobianRank=Rank(Jacobian(q,geometry))}).ToArray(),
                goals,homeJacobian=Jacobian(known[0],geometry),sourceAssetsUnchanged=true,sourceHashes=hashes,
                ikSolverImplemented=false,ikSolvesPerformed=0,
                scope="IK contract/reference/Jacobian investigation only; known joint values are verification oracles, not solver seeds"};
            Directory.CreateDirectory("Logs/LW07C");
            File.WriteAllText("Logs/LW07C/reference-goals.json",JsonConvert.SerializeObject(report,Formatting.Indented));
            return new {report.passed,report.knownFeasibleTargetCount,report.jacobianColumnsChecked,
                report.maxLinearJacobianDifferenceMetresPerRadian,report.maxAngularJacobianDifference,
                report.homeJacobianRank,ordinaryRanks=goals.Select(g=>g.estimatedJacobianRank).ToArray(),
                report.rotationResidualChecks,report.conservativeOuterReachBoundMetres,
                report.sourceAssetsUnchanged,report.ikSolverImplemented,report.ikSolvesPerformed};
        }
        finally {PrefabUtility.UnloadPrefabContents(root);}
    }

    // RobotFrame linear/angular velocity Jacobian; joint increments are radians.
    private static double[,] Jacobian(float[] degrees,FanucKinematicGeometry geometry)
    {
        var position=Vector3.zero; var rotation=Quaternion.identity;
        var pivots=new Vector3[6]; var axes=new Vector3[6];
        for(int i=0;i<6;i++)
        {
            var home=geometry.Joints[i];
            position+=rotation*home.PositionMetres;
            var preJoint=rotation*home.Rotation;
            pivots[i]=position; axes[i]=(preJoint*home.LocalRotationAxis.normalized).normalized;
            rotation=Quaternion.Normalize(preJoint*Quaternion.AngleAxis(degrees[i],home.LocalRotationAxis.normalized));
        }
        var tcp=FanucForwardKinematics.Calculate(degrees,geometry).PositionMetres;
        var result=new double[6,6];
        for(int column=0;column<6;column++)
        {
            var linear=Vector3.Cross(axes[column],tcp-pivots[column]);
            for(int row=0;row<3;row++) {result[row,column]=linear[row]; result[row+3,column]=axes[column][row];}
        }
        return result;
    }

    private static Vector3 RotationVector(Quaternion desired,Quaternion actual)
    {
        var d=Quaternion.Normalize(desired*Quaternion.Inverse(actual));
        if(d.w<0) d=new Quaternion(-d.x,-d.y,-d.z,-d.w);
        double size=Math.Sqrt((double)d.x*d.x+(double)d.y*d.y+(double)d.z*d.z);
        double factor=size<1e-10?2:2*Math.Atan2(size,d.w)/size;
        return new Vector3((float)(factor*d.x),(float)(factor*d.y),(float)(factor*d.z));
    }
    private static int CheckRotationResiduals()
    {
        var q=Quaternion.Euler(35,-20,15);
        Require(RotationVector(q,new Quaternion(-q.x,-q.y,-q.z,-q.w)).magnitude<.000001f,"q/-q should have zero residual.");
        float small=RotationVector(Quaternion.AngleAxis(.0001f,Vector3.right),Quaternion.identity).magnitude;
        Require(Mathf.Abs(small-.0001f*Mathf.Deg2Rad)<1e-9f,"Small-angle residual is wrong.");
        float wrapped=RotationVector(Quaternion.AngleAxis(179,Vector3.up),Quaternion.AngleAxis(-179,Vector3.up)).magnitude;
        Require(Mathf.Abs(wrapped-2*Mathf.Deg2Rad)<.000001f,"Shortest rotation across +/-180 is wrong.");
        Require(Mathf.Abs(RotationVector(Quaternion.AngleAxis(180,Vector3.up),Quaternion.identity).magnitude-Mathf.PI)<.000001f,"Half-turn residual is wrong.");
        var expected=new Vector3(1,2,3).normalized*(20*Mathf.Deg2Rad);
        var mixed=RotationVector(Quaternion.AngleAxis(20,new Vector3(1,2,3).normalized)*q,q);
        Require(Vector3.Distance(mixed,expected)<.000001f,"Rotation residual is not in RobotFrame.");
        return 5;
    }
    private static int Rank(double[,] source)
    {
        var a=(double[,])source.Clone(); double max=0;
        for(int row=0;row<6;row++) for(int col=0;col<6;col++)
        {if(row<3) a[row,col]/=LengthScaleMetres; max=Math.Max(max,Math.Abs(a[row,col]));}
        double tolerance=Math.Max(1,max)*RankRelativeTolerance;
        int rank=0;
        for(int col=0;col<6 && rank<6;col++)
        {
            int pivot=rank;
            for(int row=rank+1;row<6;row++) if(Math.Abs(a[row,col])>Math.Abs(a[pivot,col])) pivot=row;
            if(Math.Abs(a[pivot,col])<=tolerance) continue;
            for(int j=col;j<6;j++) {double temp=a[rank,j]; a[rank,j]=a[pivot,j]; a[pivot,j]=temp;}
            for(int row=rank+1;row<6;row++)
            {double factor=a[row,col]/a[rank,col]; for(int j=col;j<6;j++) a[row,j]-=factor*a[rank,j];}
            rank++;
        }
        return rank;
    }
    private static float[] V(Vector3 p)=>new[]{p.x,p.y,p.z};
    private static float[] Q(Quaternion q)=>new[]{q.x,q.y,q.z,q.w};
    private static string Hash(string path) {using(var sha=SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path)));}
    private static void Require(bool condition,string message) {if(!condition) throw new InvalidOperationException(message);}
}
