using UnityEngine;
using Unity.Robotics;
using Unity.Robotics.UrdfImporter.Control;

using UrdfControlRobot = Unity.Robotics.UrdfImporter.Control;
using UnityEditor.ShaderGraph;

public class DensoVP6242Controller : MonoBehaviour
{
    public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };
    public enum ControlType { PositionControl };

    private ArticulationBody[] articulationChain;
    public int currentArticulationIndex;
    private int previousArticulationIndex;

    public ControlType control = ControlType.PositionControl;

    public float stiffness;
    public float damping;
    public float forceLimit;
    public float speed = 5f;
    public float torque = 100f;
    public float acceleration = 5f;

    void Start()
    {
        currentArticulationIndex = previousArticulationIndex = 1;

        articulationChain = this.GetComponentsInChildren<ArticulationBody>();

        int defDyanmicVal = 10;
        foreach (ArticulationBody joint in articulationChain)
        {
            joint.gameObject.AddComponent<DensoVP6242JointControl>();

            joint.jointFriction = defDyanmicVal;
            joint.angularDamping = defDyanmicVal;

            ArticulationDrive currentDrive = joint.xDrive;

            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
        }
    }

    void Update()
    {
        UpdateDirection(1, RotationDirection.Positive);
    }

    private void UpdateDirection(int jointIndex, RotationDirection direction)
    {
        if (jointIndex < 0 || jointIndex >= articulationChain.Length)
        {
            return;
        }

        DensoVP6242JointControl current = articulationChain[jointIndex].GetComponent<DensoVP6242JointControl>();
        if (previousArticulationIndex != jointIndex)
        {
            DensoVP6242JointControl previous = articulationChain[previousArticulationIndex].GetComponent<DensoVP6242JointControl>();
            previous.direction = RotationDirection.None;
            previousArticulationIndex = jointIndex;
        }

        if (current.controltype != control)
        {
            UpdateControlType(current);
        }

        current.direction = direction;
    }

    public void UpdateControlType(DensoVP6242JointControl joint)
    {
        joint.controltype = control;
        if (control == ControlType.PositionControl)
        {
            ArticulationDrive drive = joint.joint.xDrive;

            drive.stiffness = stiffness;
            drive.damping = damping;
            joint.joint.xDrive = drive;
        }
    }
}
