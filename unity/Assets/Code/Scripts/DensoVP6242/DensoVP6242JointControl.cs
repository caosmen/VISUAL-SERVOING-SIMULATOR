using UnityEngine;
using Unity.Robotics;
using Unity.Robotics.UrdfImporter.Control;

using UrdfControlRobot = Unity.Robotics.UrdfImporter.Control;

public class DensoVP6242JointControl : MonoBehaviour
{
    DensoVP6242Controller controller;

    public DensoVP6242Controller.RotationDirection direction;
    public DensoVP6242Controller.ControlType controltype;

    public float speed;
    public float torque;
    public float acceleration;

    public ArticulationBody joint;

    void Start()
    {
        direction = 0;

        controller = (DensoVP6242Controller)this.GetComponentInParent(typeof(DensoVP6242Controller));

        joint = this.GetComponent<ArticulationBody>();

        controller.UpdateControlType(this);

        speed = controller.speed;
        torque = controller.torque;
        acceleration = controller.acceleration;
    }

    void FixedUpdate()
    {
        speed = controller.speed;
        torque = controller.torque;
        acceleration = controller.acceleration;


        if (joint.jointType != ArticulationJointType.FixedJoint)
        {
            if (controltype == DensoVP6242Controller.ControlType.PositionControl)
            {
                ArticulationDrive currentDrive = joint.xDrive;
                float newTargetDelta = (int)direction * Time.fixedDeltaTime * speed;

                if (joint.jointType == ArticulationJointType.RevoluteJoint)
                {
                    if (joint.twistLock == ArticulationDofLock.LimitedMotion)
                    {
                        if (newTargetDelta + currentDrive.target > currentDrive.upperLimit)
                        {
                            currentDrive.target = currentDrive.upperLimit;
                        }
                        else if (newTargetDelta + currentDrive.target < currentDrive.lowerLimit)
                        {
                            currentDrive.target = currentDrive.lowerLimit;
                        }
                        else
                        {
                            currentDrive.target += newTargetDelta;
                        }
                    }
                    else
                    {
                        currentDrive.target += newTargetDelta;

                    }
                }
                else if (joint.jointType == ArticulationJointType.PrismaticJoint)
                {
                    if (joint.linearLockX == ArticulationDofLock.LimitedMotion)
                    {
                        if (newTargetDelta + currentDrive.target > currentDrive.upperLimit)
                        {
                            currentDrive.target = currentDrive.upperLimit;
                        }
                        else if (newTargetDelta + currentDrive.target < currentDrive.lowerLimit)
                        {
                            currentDrive.target = currentDrive.lowerLimit;
                        }
                        else
                        {
                            currentDrive.target += newTargetDelta;
                        }
                    }
                    else
                    {
                        currentDrive.target += newTargetDelta;

                    }
                }

                joint.xDrive = currentDrive;
            }
        }
    }
}
