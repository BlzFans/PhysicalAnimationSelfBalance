using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class MonsterPhysicalLeg
{
    public List<ConfigurableJoint> joints = new List<ConfigurableJoint>();
    public List<float> link_length = new List<float>();
    public List<Tuple<Vector3, Vector3>> joints_axis = new List<Tuple<Vector3, Vector3>>();
    // Local axis of joints init angle
    public Vector3 base_feet_bias;
    public Vector3 base_root_bias;
    public Vector3 base_pole_bias;
    public float base_root_height_bias;
    public float base_pole_height_bias;
    public float max_extend_length;
    public float min_extend_length;
    public void InitParameters(Transform center_of_all_mass, Vector3 pole_world_position)
    {
        // 
        for (int i = 1; i < joints.Count; i++)
        {
            link_length.Add((GetNowJointPosition(i) - GetNowJointPosition(i-1)).magnitude);
            Vector3 x_axis = joints[i].connectedBody.transform.InverseTransformDirection(joints[i].transform.right);
            Vector3 y_axis = joints[i].connectedBody.transform.InverseTransformDirection(joints[i].transform.up);
            joints_axis.Add(new Tuple<Vector3, Vector3>(x_axis, y_axis));
        }
        // 
        Vector3 feet_joint_position = GetNowJointPosition(0);
        base_feet_bias = center_of_all_mass.InverseTransformPoint(feet_joint_position);
        Vector3 root_joint_position = GetNowJointPosition(joints.Count-1);
        base_root_bias = center_of_all_mass.InverseTransformPoint(root_joint_position);
        base_pole_bias = center_of_all_mass.InverseTransformPoint(pole_world_position);

        base_root_height_bias = root_joint_position.y - feet_joint_position.y;
        base_pole_height_bias = pole_world_position.y;

        // C = A * Quaternion.Inverse(B) -> C = A - B -> B = A;

        //
        //float max_angle = joints[1].highAngularXLimit;
        //max_extend_length = Mathf.Sqrt(Mathf.Pow(link_length[0], 2f) + Mathf.Pow(link_length[1], 2f) - 2f * link_length[0] * link_length[1] * Mathf.Cos(max_angle / Mathf.Rad2Deg));
    }
    public Vector3 GetNowFeetBias(Transform center_of_all_mass)
    {
        Vector3 feet_joint_position = GetNowJointPosition(0);
        return center_of_all_mass.InverseTransformPoint(feet_joint_position);
    }
    public Tuple<Vector3, Vector3, Vector3> GetWorldBiasPosition(Transform center_of_all_mass)
    {   
        Vector3 world_base_feet_bias = center_of_all_mass.TransformPoint(base_feet_bias);
        Vector3 world_base_root_bias = center_of_all_mass.TransformPoint(base_root_bias);
        Vector3 world_base_pole_bias = center_of_all_mass.TransformPoint(base_pole_bias);
        return new Tuple<Vector3, Vector3, Vector3>(world_base_feet_bias, world_base_root_bias, world_base_pole_bias);
    }
    public Vector3 GetNowJointPosition(int i)
    {
        return joints[i].connectedBody.transform.TransformPoint(joints[i].connectedAnchor);
    }

}