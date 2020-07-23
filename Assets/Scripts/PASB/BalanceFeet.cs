using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(GroundContactor))]
public class BalanceFeet : MonoBehaviour
{
    public Transform toes;
    /// <summary>
    /// Is right feet or left feet
    /// </summary>
    public bool is_right = false;
    /// <summary>
    /// Maximal speed joint can rotate to fit the ground surface
    /// </summary>
    public float feet_rotate_speed = 360f;
    [HideInInspector]
    public Transform now_ground;
    [HideInInspector]
    public float garound_surface_height;
    [HideInInspector]
    public float garound_height;
    [HideInInspector]
    public Collider[] feet_colliders;
    [HideInInspector]
    public BalanceArea balance_area;
    [HideInInspector]
    public GroundContactor ground_contactor;
    [HideInInspector]
    public ConfigurableJoint feet_joint;
    [HideInInspector]
    public Quaternion axis_rotation;
    [HideInInspector]
    public Quaternion world_to_joint_space;
    [HideInInspector]
    public Quaternion joint_to_world_space;
    [HideInInspector]
    public Quaternion axis_bias;
    [HideInInspector]
    public float constant_feet_height;
    const float max_ray_length = 1f;
    Vector3 garound_normal;

    public void FeetStart(Vector3 pole_direction)
    {
        feet_joint = GetComponent<ConfigurableJoint>();
        ground_contactor = GetComponent<GroundContactor>();
        feet_colliders = GetComponents<Collider>();

        if (GameObject.Find("Feet_Balance_Area" + name) == null)
        {
            var area = new GameObject("Feet_Balance_Area" + name);
            area.transform.SetParent(GameObject.Find("Character").transform);
            var sphere = area.AddComponent<SphereCollider>();
            sphere.isTrigger = true;
            balance_area = area.AddComponent<BalanceArea>();
            balance_area.feet = this;  
        }

        FeetFixedUpdate();
        constant_feet_height = feet_joint.connectedBody.transform.TransformPoint(feet_joint.connectedAnchor).y - garound_surface_height;
        garound_normal = Vector3.up;

        axis_rotation = transform.localRotation;
        // Calculate the rotation expressed by the joint's axis and secondary axis and get its space transform matrix
        var right = feet_joint.axis;
        var up = feet_joint.secondaryAxis;
        var forward = Vector3.Cross(right, up).normalized;
        world_to_joint_space = Quaternion.LookRotation(forward, up);
        joint_to_world_space = Quaternion.Inverse(world_to_joint_space);

        Vector3 forward_axis = Vector3.Cross(pole_direction, garound_normal);
        Quaternion bias_world_rotation = Quaternion.LookRotation(forward_axis, garound_normal);
        Quaternion bias_local_rotation = Quaternion.Inverse(feet_joint.connectedBody.transform.rotation) * bias_world_rotation;
        axis_bias = Quaternion.Inverse(bias_local_rotation) * transform.localRotation;
        
    }

    public void FeetFixedUpdate()
    {
        // Get info of ground surface that under the feet
        RaycastHit[] allHits;
        allHits = Physics.RaycastAll(transform.position, -Vector3.up, max_ray_length);
        foreach (var hit in allHits)
        {
            if (hit.transform.tag == "ground")
            {
                now_ground = hit.transform;
                garound_surface_height = hit.point.y;
                garound_height = garound_surface_height + constant_feet_height;
                garound_normal = hit.normal;
                Debug.DrawLine(hit.point, hit.point + garound_normal, Color.blue);
                break;                
            }
        }
    }

    public void FeetFitSurface(Vector3 pole_direction)
    {
        Vector3 forward_axis = Vector3.Cross(pole_direction, garound_normal);
        Quaternion target_world_rotation = Quaternion.LookRotation(forward_axis, garound_normal);
        Quaternion target_local_rotation = Quaternion.Inverse(feet_joint.connectedBody.transform.rotation) * target_world_rotation;
        target_local_rotation *= axis_bias;
        // Control the max rotate speed.
        Quaternion now_target_rotation = Quaternion.RotateTowards(feet_joint.transform.localRotation, target_local_rotation, feet_rotate_speed * Time.fixedDeltaTime);
        // Matrix change of basis, Transform different of local rotation back into joint space
        Quaternion joint_target_local_rotation = joint_to_world_space * (Quaternion.Inverse(now_target_rotation) * axis_rotation) * world_to_joint_space;
        // Set target rotation to our newly calculated rotation
        feet_joint.targetRotation = joint_target_local_rotation;
    }

    public Vector3 GetFeetJointPosition()
    {
        return feet_joint.connectedBody.transform.TransformPoint(feet_joint.connectedAnchor);
    }
}
