using UnityEngine;
using Unity.Robotics;
using Unity.Robotics.UrdfImporter.Control;

using UrdfControlRobot = Unity.Robotics.UrdfImporter.Control;

public class VP6242JointControl : MonoBehaviour
{
    [Header("Controller")]
    VP6242Controller controller;

    [Header("Control Parameters")]
    public float speed;
    public float torque;
    public float acceleration;

    [Header("Joint")]
    public ArticulationBody joint;

    void Start()
    {
        controller = (VP6242Controller) this.GetComponentInParent(typeof(VP6242Controller));
        
        speed = controller.speed;
        torque = controller.torque;
        acceleration = controller.acceleration;
        
        joint = this.GetComponent<ArticulationBody>();
    }

    void FixedUpdate()
    {
        
    }
}
