using UnityEngine;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;


public class VP6242Controller : MonoBehaviour
{
    private ArticulationBody[] articulationChain;

    private readonly int defaultDynamic = 10;
    private readonly int defaultForceLimit = 1000;

    [Header("Control Parameters")]
    public float speed = 5f;
    public float torque = 100f;
    public float acceleration = 5f;

    void Start()
    {
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        foreach (ArticulationBody joint in articulationChain)
        {
            joint.gameObject.AddComponent<VP6242JointControl>();

            joint.jointFriction = defaultDynamic;
            joint.angularDamping = defaultDynamic;

            ArticulationDrive currentDrive = joint.xDrive;

            currentDrive.forceLimit = defaultForceLimit;
            currentDrive.driveType = ArticulationDriveType.Target;

            joint.xDrive = currentDrive;
        }
    }

    private HeaderMsg InitializeHeaderMsg()
    {
        int currentSeconds = (int) Time.time;
        uint currentNanoseconds = (uint) ((Time.time - currentSeconds) * 1e9);

        return new HeaderMsg
        {
            stamp = new TimeMsg
            {
                sec = currentSeconds,
                nanosec = currentNanoseconds
            },
            frame_id = "vp6242"
        };
    }

    private JointStateMsg InitializeJointStateMsg(int numJoints)
    {
        return new JointStateMsg
        {
            header = InitializeHeaderMsg(),
            name = new string[numJoints],
            position = new double[numJoints],
            velocity = new double[numJoints],
            effort = new double[numJoints]
        };
    }

    public JointStateMsg GetJointStates()
    {
        int numJoints = articulationChain.Length;

        JointStateMsg jointStatesMsg = InitializeJointStateMsg(numJoints);

        for (int i = 0; i < numJoints; i++)
        {
            jointStatesMsg.name[i] = articulationChain[i].name;
            jointStatesMsg.position[i] = GetValidJointFeature(articulationChain[i].jointPosition);
            jointStatesMsg.velocity[i] = GetValidJointFeature(articulationChain[i].jointVelocity);
            jointStatesMsg.effort[i] = GetValidJointFeature(articulationChain[i].driveForce);
        }

        return jointStatesMsg;
    }

    private double GetValidJointFeature(ArticulationReducedSpace articulation)
    {
        return articulation.dofCount > 0 ? articulation[0] : 0;
    }

    public void SetJointState(JointStateMsg jointStateMsg)
    {
        for (int i = 0; i < jointStateMsg.name.Length; i++)
        {
            string jointName = jointStateMsg.name[i];
            ArticulationBody joint = GetJointByName(jointName);

            if (joint != null)
            {
                if (i < jointStateMsg.position.Length)
                {
                        float targetPosition = (float) jointStateMsg.position[i];
                        float targetPositionDegrees = targetPosition * Mathf.Rad2Deg;

                        ArticulationDrive currentDrive = joint.xDrive;

                        currentDrive.driveType = ArticulationDriveType.Target;
                        currentDrive.target = targetPositionDegrees;

                        joint.xDrive = currentDrive;
                }
                else if (i < jointStateMsg.velocity.Length)
                {
                        float targetVelocity = (float) jointStateMsg.velocity[i];
                        float targetVelocityDegrees = targetVelocity * Mathf.Rad2Deg;

                        ArticulationDrive currentVelocityDrive = joint.xDrive;

                        currentVelocityDrive.driveType = ArticulationDriveType.Velocity;
                        currentVelocityDrive.targetVelocity = targetVelocityDegrees;

                        joint.xDrive = currentVelocityDrive;
                }
            }
        }
    }

    private ArticulationBody GetJointByName(string jointName)
    {
        foreach (ArticulationBody joint in articulationChain)
        {
            if (joint.name == jointName)
            {
                return joint;
            }
        }

        return null;
    }
}
