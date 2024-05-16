using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class SubscriberVP6242JointState : MonoBehaviour
{
    private ROSConnection rosConnection;
    
    [Header("VP6242 Controller")]
    public VP6242Controller vp6242Controller;

    [Header("ROS Topics Names")]
    public string topicJointStateName = "/vp6242/joint_command";
    
    void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();

        rosConnection.Subscribe<JointStateMsg>(topicJointStateName, ReceiveJointState);
    }
    
    private void ReceiveJointState(JointStateMsg jointStateMsg)
    {
        if (vp6242Controller)
        {
            vp6242Controller.SetJointState(jointStateMsg);
        }
    }
}
