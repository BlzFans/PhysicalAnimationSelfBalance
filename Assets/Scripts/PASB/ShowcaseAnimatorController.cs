using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShowcaseAnimatorController : MonoBehaviour
{
    public PhysicalAnimationBalance pab;
    [HideInInspector]
    public Rigidbody rg;
    ShowcaseCamera show_camera;
    Animator show_animator;
    public LayerMask layerMask; // Select all layers that foot placement applies to.

    [Range (0, 1f)]
    public float DistanceToGround; // Distance from where the foot transform is to the lowest possible position of the foot.
    public float ref_move_force = 9f;
    public float walk_speed = 1.5f;
    public float run_speed = 3f;
    public float rotate_speed = 360f;
    const float vel_gate = 0.2f;
    const float ray_cast_length = 1f;
    Vector3 plane_velocity;
    Vector3 last_position;
    bool is_aim = false;
    
    void Start()
    {
        show_camera = GameObject.FindObjectOfType<ShowcaseCamera>();
        show_animator = GetComponent<Animator>();
        rg = GetComponent<Rigidbody>();
        last_position = transform.position;
    }

    void FixedUpdate()
    {
        Vector3 direction = Vector3.zero;
        if (Input.GetKey("w"))
        {
            direction += Vector3.ProjectOnPlane(show_camera.transform.forward, Vector3.up);
        }
        if (Input.GetKey("s"))
        {
            direction += Vector3.ProjectOnPlane(-show_camera.transform.forward, Vector3.up);
        }
        if (Input.GetKey("a"))
        {
            direction += Vector3.ProjectOnPlane(-show_camera.transform.right, Vector3.up);
        }
        if (Input.GetKey("d"))
        {
            direction += Vector3.ProjectOnPlane(show_camera.transform.right, Vector3.up);
        }
        direction = direction.normalized;

        if (Input.GetKeyDown("c"))
        {
            is_aim = !is_aim;
            show_animator.SetBool("Is_Aim", is_aim);
        } 

        // Change Linear velocity of self rigidbody 
        float now_max_speed = walk_speed;
        float now_dir_scaler = 2f;
        if (Input.GetKey("left shift") || is_aim || pab.in_self_balance_mode)
        {
            now_max_speed = run_speed;
            now_dir_scaler = 1f;
        }

        float direction_speed = Vector3.Project(plane_velocity, direction).magnitude;

        Vector3 now_add_force = direction * Mathf.Min(now_max_speed - direction_speed, ref_move_force * Time.fixedDeltaTime);
        rg.AddForce(now_add_force, ForceMode.VelocityChange);

        Vector3 look_dir = Vector3.forward;
        if (is_aim && !pab.in_self_balance_mode)
        {
            look_dir = plane_velocity.magnitude < vel_gate ? direction : plane_velocity.normalized;
            rg.freezeRotation = true;
        }
        else
        {
            rg.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
            look_dir = plane_velocity.magnitude < vel_gate ? transform.forward : plane_velocity.normalized;
            if (!is_aim)
            {
                // Change facing rotation of this object according to Linear velocity's direction of self rigidbody
                Quaternion target_rotation = Quaternion.LookRotation(look_dir, Vector3.up);
                transform.rotation = Quaternion.RotateTowards(transform.rotation, target_rotation, rotate_speed * Time.fixedDeltaTime);                    
            }
        }
        // Set animation according to Linear velocity of self rigidbody
        float loc_angle = Vector3.SignedAngle(transform.forward, Vector3.forward, Vector3.up);
        Vector3 loc_direction = Vector3.Lerp(Vector3.zero, look_dir / now_dir_scaler, Mathf.InverseLerp(0f, now_max_speed, plane_velocity.magnitude));
        loc_direction = Quaternion.AngleAxis(loc_angle, Vector3.up) * loc_direction;
        show_animator.SetFloat("Speed_Forward", loc_direction.z);
        show_animator.SetFloat("Speed_Right", loc_direction.x);
        show_animator.SetBool("In_Self_Balance_Mode", pab.in_self_balance_mode);

        // Update  velocity
        plane_velocity = Vector3.ProjectOnPlane(transform.position - last_position, Vector3.up) / Time.fixedDeltaTime;
        last_position = transform.position;
    }

    void OnAnimatorIK(int layerIndex) 
    {

        if (show_animator) 
        { // Only carry out the following code if there is an Animator set.

            // Set the weights of left and right feet to the current value defined by the curve in our animations.
            show_animator.SetIKPositionWeight(AvatarIKGoal.LeftFoot, show_animator.GetFloat("IKLeftFootWeight"));
            show_animator.SetIKRotationWeight(AvatarIKGoal.LeftFoot, show_animator.GetFloat("IKLeftFootWeight"));
            show_animator.SetIKPositionWeight(AvatarIKGoal.RightFoot, show_animator.GetFloat("IKRightFootWeight"));
            show_animator.SetIKRotationWeight(AvatarIKGoal.RightFoot, show_animator.GetFloat("IKRightFootWeight"));

            // Left Foot
            RaycastHit hit;
            // We cast our ray from above the foot in case the current terrain/floor is above the foot position.
            Ray ray = new Ray(show_animator.GetIKPosition(AvatarIKGoal.LeftFoot) + Vector3.up, Vector3.down);
            if (Physics.Raycast(ray, out hit, ray_cast_length + 1f, layerMask)) 
            {

                // We're only concerned with objects that are tagged as "ground"
                if (hit.transform.tag == "ground") 
                {

                    Vector3 footPosition = hit.point; // The target foot position is where the raycast hit a ground object...
                    footPosition.y += DistanceToGround; // ... taking account the distance to the ground we added above.
                    show_animator.SetIKPosition(AvatarIKGoal.LeftFoot, footPosition);
                    show_animator.SetIKRotation(AvatarIKGoal.LeftFoot, Quaternion.LookRotation(transform.forward, hit.normal));

                }
            }

            // Right Foot
            ray = new Ray(show_animator.GetIKPosition(AvatarIKGoal.RightFoot) + Vector3.up, Vector3.down);
            if (Physics.Raycast(ray, out hit, ray_cast_length + 1f, layerMask)) 
            {

                if (hit.transform.tag == "ground") 
                {

                    Vector3 footPosition = hit.point;
                    footPosition.y += DistanceToGround;
                    show_animator.SetIKPosition(AvatarIKGoal.RightFoot, footPosition);
                    show_animator.SetIKRotation(AvatarIKGoal.RightFoot, Quaternion.LookRotation(transform.forward, hit.normal));

                }
            }
        }

    }

}
