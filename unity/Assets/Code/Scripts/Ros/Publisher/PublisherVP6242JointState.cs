using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class PublisherVP6242JointState : MonoBehaviour
{
    private ROSConnection rosConnection;
    
    [Header("VP6242 Controller")]
    public VP6242Controller vp6242Controller;

    [Header("ROS Topics Names")]
    public string topicJointStateName = "/vp6242/joint_states";

    [Header("ROS Publish Frequency")]
    public float publishMessageFrequency = 0.5f;

    private float timeElapsed;
    
    void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();

        rosConnection.RegisterPublisher<JointStateMsg>(topicJointStateName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            PublishJointStates();

            timeElapsed = 0;
        }
    }

    private void PublishJointStates()
    {
        if (vp6242Controller)
        {
            JointStateMsg jointStatesMsg = vp6242Controller.GetJointStates();

            rosConnection.Publish(topicJointStateName, jointStatesMsg);
        }
    }
}
