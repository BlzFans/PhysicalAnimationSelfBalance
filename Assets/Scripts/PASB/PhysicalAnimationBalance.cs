using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class PhysicalAnimationBalance : MonoBehaviour
{

    [Space(5f)][Header("Self balance leg choose nad move parameters")]
    /// <summary>
    /// Center of all mass transform, we use root body's position as its position and rotation use the character facing direction and world up direction to calculate,
    /// many data like recorded balance pose's transform is stored in local transform of center_of_all_mass;
    /// </summary>
    public Transform center_of_all_mass;
    /// <summary>
    /// Should be child of root body, for calculate the rotation of center_of_all_mass, forward direction determine the character's facing direction
    /// </summary>
    public Transform CoM_rotator;
    public PhysicalLeg[] legs;
    /// <summary>
    /// Index of the balance pose
    /// </summary>
    public int pose_index = 0;
    /// <summary>
    /// Radius of balance area, when feet collider overlap balance area we consider that feet is balanced
    /// </summary>
    public float base_feet_radius = 0.1f;
    /// <summary>
    /// The max radius the feet must inside for generate force to make root rigidbody follow the correct position and rotation
    /// </summary>
    public float max_feet_balance_radius = 1f;
    /// <summary>
    /// When feet inside this radius then max force to make root rigidbody follow the correct position and rotation will apply
    /// </summary>
    public float min_feet_balance_radius = 0.25f;
    /// <summary>
    /// When feet one gait end then feet land one ground, that feet will keep that position, but while keep it the whole body will drag that feet moving, 
    /// then we need keep the feet's position in this radius with the landing position of feet relatively to root body
    /// </summary>
    public float max_feet_keep_radius = 0.25f;
    public float min_feet_keep_radius = 0.025f;
    public float feet_keep_back_speed = 0.5f;
    /// <summary>
    /// How much of distance between target and current feet position determine choosing the current feet to move 
    /// </summary>
    public float distance_priority_param = 1f;
    /// <summary>
    /// How much of radian between target to current feet position vector and other feet to current feet position vector determine choosing the current feet to move 
    /// </summary>
    public float radian_priority_param = 0.1f;
    /// <summary>
    /// When root speed on horizontal plane is less than this gate, then feet adjust freely without direction restriction
    /// </summary>
    public float feet_adjust_root_gate_linear_speed = 0.5f;
    /// <summary>
    /// Speed to make leg's root joint (thigh) position to follow it target root joint position, prevent the leg jump up
    /// </summary>
    public float keep_root_up_speed = 2f;
    /// <summary>
    /// When gait of feet is finished, that feet may not contact the ground and will cause some problems like root body shaking, so we moving feet down until it contact the ground 
    /// </summary>
    public float keep_feet_down_speed = 0.2f;
    /// <summary>
    /// Because the controll is physical based so feet will usually can't moving exactly to target position, 
    /// and after feet finish the moving and on the ground feet cannot be move accurately because of ground friction and effect on other body, 
    /// so instead of moving feet toward target we moving root body back to make the feet fit target area better,
    /// </summary>
    public float center_meet_speed = 0.25f;
    /// <summary>
    /// When feets that on ground only at one side of center of mass, then it will not only create upward support force,
    /// but also create some horizontal force that push the body aside, so we need add some extra movement to create this effect
    /// </summary>
    public float max_reverse_move_speed = 2f;
    public float max_reverse_move_add_speed = 1.25f;
    public float reverse_move_decline_speed = 5f;

    [Space(5f)][Header("Self balance gait plan parameters")]
    /// <summary>
    /// Max time feet can move form start to target position
    /// </summary>
    public float feet_move_time = 0.3f;
    /// <summary>
    /// Use root speed on horizontal plane to lerp the length of additional feet movement that will shift target position
    /// </summary>
    public float max_over_feet_root_linear_speed = 3f;
    public float min_over_feet_root_linear_speed = 0.5f;
    public float Linear_max_over_feet_radius = 0.5f;
    /// <summary>
    /// Use root rotation different with initial pose to lerp the length of additional feet movement that will shift target position
    /// </summary>
    public float max_over_feet_root_rot = 90f;
    public float min_over_feet_root_rot = 5f;
    public float rotate_max_over_feet_radius = 0.25f;
    /// <summary>
    /// Use root angular velocity to lerp the length of additional feet movement that will shift target position
    /// </summary>
    public float max_over_feet_root_angular_speed = 60f;
    public float min_over_feet_root_angular_speed = 3f;
    public float angular_vel_max_over_feet_radius = 0.15f;
    /// <summary>
    /// Max feet above the ground height when feet moving for balance
    /// </summary>
    public float gait_max_height = 0.3f;
    /// <summary>
    /// At begining of each feet move for balance, we check the feet height above the ground, if feet higher than this value then feet use drop gait
    /// </summary>
    public float gait_gate_height = 0.1f;
    /// <summary>
    /// Gait cureve for feet to follow when feet moving for balance
    /// </summary>
    public AnimationCurve full_gait_curve;
    public AnimationCurve drop_gait_curve;

    [Space(5f)][Header("Self balance avoid obstacle parameters")]
    /// <summary>
    /// The maximal length vertical move feet can add
    /// </summary>
    public float max_vertical_avoid_length = 0.5f;
    /// <summary>
    /// Minimal length between two leg when position of leg's joint project on the plane which normal are target feet move direction 
    /// </summary>
    public float min_leg_avoid_length = 0.03f;
    /// <summary>
    /// Speed for feet move to avoid the obstacle (vertical move increase toward max) when there is obstacle between the feet and target
    /// </summary>
    public float avoid_move_speed = 1f;
    /// <summary>
    /// Speed for feet back to it original trajectory (vertical move decline toward zero) when there no obstacle between the feet and target
    /// </summary>
    public float feet_avoid_back_speed = 1f;
    
    [Space(5f)][Header("Rigidbody PD follow parameters")]
    /// <summary>
    /// PD parameters use on root body when character in physcial animation mode
    /// </summary>
    public float root_pa_position_param = 1000f;
    public float root_pa_velocity_param = 20f;
    public float root_pa_rotation_param = 500f;
    public float root_pa_angular_velocity_param = 40f;
    /// <summary>
    /// PD parameters use on root body when character in self balance mode
    /// </summary>
    public float root_sb_position_param = 500f;
    public float root_sb_velocity_param = 10f;
    public float root_sb_rotation_param = 300f;
    public float root_sb_angular_velocity_param = 20f;
    /// <summary>
    /// Maximal speed joint can rotate toward target roation
    /// </summary>
    public float joint_rotate_speed = 2000f;

    [Space(5f)][Header("Reference follow parameters")]
    /// <summary>
    /// Reference root transform use for set all body part's PD target
    /// </summary>
    public Transform ref_root_transform;
    /// <summary>
    /// Maximal horizontal positional different between root body and it's reference, limit the reference in range
    /// </summary>
    public float max_distance_diff = 0.5f;
    /// <summary>
    /// Maximal rotational different between root body and it's reference, limit the reference in range
    /// </summary>
    public float max_angle_diff = 15f;
    /// <summary>
    /// Reset all all rigidbody's maximal angular velocity
    /// </summary>
    public float max_rg_angular_velocity = 10000f;
    /// <summary>
    /// Physic material for feet use in physcial animation mod and self balance mode
    /// </summary>
    public PhysicMaterial pa_pm;
    public PhysicMaterial sb_pm;

    [HideInInspector]
    /// <summary>
    /// 
    /// </summary>
    public List<Quaternion> CoM_balance_rotation_bias = new List<Quaternion>();
    [HideInInspector]
    public BalanceFeet[] feets;
    [HideInInspector]
    public Rigidbody root_rg = null;
    [HideInInspector]
    public Rigidbody[] all_rgs;
    List<ConfigurableJoint> all_joints = new List<ConfigurableJoint>();
    List<Quaternion> joints_init_axis_rotation = new List<Quaternion>();
    List<Quaternion> world_to_joints_axis_space = new List<Quaternion>();
    List<Quaternion> joints_to_world_axis_space = new List<Quaternion>();
    List<Quaternion> last_joints_local_rotation = new List<Quaternion>();
    List<float[]> all_rgs_drags = new List<float[]>();
    Dictionary<string, Transform> ref_body_dict = new Dictionary<string, Transform>();
    Vector3 last_root_position;
    Vector3 root_linear_velocity;
    Quaternion last_root_rotation;
    Vector3 root_angular_velocity;
    [HideInInspector]
    public bool in_self_balance_mode = false;
    [HideInInspector]
    public bool feets_balanced = true;
    bool lock_in_self_balance_mode = false;
    bool is_feet_moving = false;
    int moveing_leg_index = -1;
    
    Vector3 root_plane_linear_velocity;
    Vector3 now_reverse_move_vel;
    Vector3 feet_vertical_move;
    Vector3 feet_start_bias;
    Vector3[] feets_keep_point;
    float now_gait_duration = 1f;
    float now_max_gait_height;
    float feet_angle_avoid_sign;
    AnimationCurve now_gait_curve;
    Collider[] change_pm_collider;


    // Debug 
    List<Vector3> draw_triangles = new List<Vector3>();
    List<Tuple<Vector3, Vector3>> draw_lines = new List<Tuple<Vector3, Vector3>>();
    List<Color> lines_colors = new List<Color>();
    List<Vector3> draw_trajectory = new List<Vector3>();
    List<Vector3> draw_target = new List<Vector3>();
    List<Color> target_colors = new List<Color>();
    [Space(25f)]
    public bool is_debug;    
    void Start()
    {
        if (root_rg == null)
            root_rg = GetComponentInChildren<Rigidbody>();
        if (all_rgs.Length == 0)
            all_rgs = GetComponentsInChildren<Rigidbody>();

        foreach (Rigidbody rg in all_rgs)
        {
            ConfigurableJoint joint;
            rg.maxAngularVelocity = max_rg_angular_velocity;
            if (rg.TryGetComponent<ConfigurableJoint>(out joint))
            {
                all_joints.Add(joint);
                joints_init_axis_rotation.Add(joint.transform.localRotation);
                // Calculate the rotation expressed by the joint's axis and secondary axis and get its space transform matrix
                var right = joint.axis;
                var up = joint.secondaryAxis;
                var forward = Vector3.Cross(right, up).normalized;
                Quaternion world_to_joint_space = Quaternion.LookRotation(forward, up);
                Quaternion joint_to_world_space = Quaternion.Inverse(world_to_joint_space);
                world_to_joints_axis_space.Add(world_to_joint_space);
                joints_to_world_axis_space.Add(joint_to_world_space);

                last_joints_local_rotation.Add(Quaternion.identity);
            }
            float[] drags = new float[2]{rg.drag, rg.angularDrag};
            all_rgs_drags.Add(drags);
        }

        var all_body = ref_root_transform.GetComponentsInChildren<Transform>();
        foreach (Transform body in all_body)
            ref_body_dict.Add(body.name, body);
        last_root_position = ref_root_transform.position;
        last_root_rotation = ref_root_transform.rotation;

        CalculateCenterOfMass();
        if (legs.Length == 0)
        {
            if (feets.Length == 0)
                feets = GetComponentsInChildren<BalanceFeet>();
            // Get all legs joints
            legs = new PhysicalLeg[feets.Length];
            for (int i = 0; i < feets.Length; i++)
            {
                PhysicalLeg new_leg = new PhysicalLeg(feets[i]);
                // Add all legs init parameters
                new_leg.ResetParameters(center_of_all_mass, feets[i]);
                legs[i] = new_leg;
            }            
        }
        feets_keep_point = new Vector3[feets.Length];
        change_pm_collider = new Collider[feets.Length];
        for (int i = 0; i < feets.Length; i++)
        {
            Vector3 feet_joint_position = feets[i].GetFeetJointPosition();
            feets_keep_point[i] = feets[i].now_ground.InverseTransformPoint(feet_joint_position);
            change_pm_collider[i] = feets[i].GetComponent<BoxCollider>();
            var sphere = feets[i].balance_area.GetComponent<SphereCollider>();
            sphere.radius = base_feet_radius;
        }
    }

    void FixedUpdate() 
    {
        // Debug
        draw_lines = new List<Tuple<Vector3, Vector3>>();
        lines_colors = new List<Color>();
        draw_triangles = new List<Vector3>();
        draw_trajectory = new List<Vector3>();
        draw_target = new List<Vector3>();
        target_colors = new List<Color>();

        if (Input.GetKey("b"))
        {
            in_self_balance_mode = !in_self_balance_mode;
            lock_in_self_balance_mode = in_self_balance_mode;
        } 

        PAndSBSwitchControl();
    }

    public void CalculateCenterOfMass()
    {
        // Use root position as center of mass, we don't use real center or mass because we want the leg's balance pose is irrelevant with other body part's transform
        center_of_all_mass.position = transform.position;
        center_of_all_mass.rotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(CoM_rotator.forward, Vector3.up), Vector3.up);
        root_plane_linear_velocity = Vector3.ProjectOnPlane(root_rg.velocity, Vector3.up);
    }

    void PAndSBSwitchControl()
    {
        // 
        CalculateCenterOfMass();
        if (in_self_balance_mode)
        {
            foreach (var col in change_pm_collider)
                if (col.material != sb_pm) col.material = sb_pm;

            if (!lock_in_self_balance_mode)
                in_self_balance_mode = !feets_balanced;

            SelfBalance();
        }
        else
        {
            foreach (var col in change_pm_collider)
                if (col.material != pa_pm) col.material = pa_pm;
        }
        PhysicalAnimationBodyFollow();
        PhysicalAnimationRootFollow();
    }

    void SelfBalance()
    {
        // 
        for (int i = 0; i < feets.Length; i++)
        {
            feets[i].FeetFixedUpdate();
            Vector3 now_target_point = center_of_all_mass.TransformPoint(legs[i].all_base_feet_bias[pose_index]);
            now_target_point.y = feets[i].garound_surface_height;
            feets[i].balance_area.transform.position = now_target_point;
        }

        // According to character current state choose the feet to move
        if (!is_feet_moving)
        {
            // Set position info of where feet should keep right after that feet finish its gait for balance
            if (moveing_leg_index > -1)
            {
                Vector3 feet_joint_position = feets[moveing_leg_index].GetFeetJointPosition();
                feets_keep_point[moveing_leg_index] = feets[moveing_leg_index].now_ground.InverseTransformPoint(feet_joint_position);
                feets_keep_point[moveing_leg_index].y = 0f;
                moveing_leg_index = -1;
            }

            float max_priority = 0f;
            bool feet_can_move = true;
            bool speed_small = root_plane_linear_velocity.magnitude <= feet_adjust_root_gate_linear_speed;
            feets_balanced = true;
            for (int i = 0; i < legs.Length; i++)
            {
                /*
                    We calculate the priority of each feet that satisfy moving condition according to:
                    distance between target and current feet position, radian between target to current feet position vector and other feet to current feet position vector;
                    We choose the feet with maximal priority to moving
                */
                if (!feets[i].balance_area.is_feet_in_area)
                {
                    Vector3 now_feet_position = feets[i].GetFeetJointPosition();
                    Vector3 feet_move_direction = feets[i].balance_area.transform.position - now_feet_position;
                    feet_move_direction = Vector3.ProjectOnPlane(feet_move_direction, Vector3.up);
                    BalanceFeet other_feet = feets[GetOtherFeetIndex(i)];
                    Vector3 other_feet_position = other_feet.GetFeetJointPosition();
                    Vector3 feet_diff_direction = Vector3.ProjectOnPlane(other_feet_position - now_feet_position, Vector3.up);

                    float feet_radian_diff = Vector3.Angle(feet_diff_direction, feet_move_direction) * Mathf.Deg2Rad;
                    float now_priority = distance_priority_param * feet_move_direction.magnitude + radian_priority_param * feet_radian_diff;
                    if (now_priority > max_priority)
                    {
                        max_priority = now_priority;
                        moveing_leg_index = i;
                    }
                    feet_can_move = feet_can_move && (speed_small || Vector3.Dot(feet_move_direction, root_plane_linear_velocity) > 0f);
                    feets_balanced = false;             
                }
            }

            if (!feet_can_move) moveing_leg_index = -1;
            if (moveing_leg_index > -1)
            {
                feet_start_bias = legs[moveing_leg_index].GetNowFeetBias(center_of_all_mass);
                feet_vertical_move = Vector3.zero;
                is_feet_moving = true;
            }
        }
        // Plan the feet moving trajectory and moving it using IK->PD controller
        for (int i = 0; i < legs.Length; i++)
        {
            Vector3 now_target_point, now_root_point, now_pole_original_direction, now_original_target_c;
            (now_target_point, now_root_point, now_pole_original_direction, now_original_target_c) = legs[i].GetWorldBias(center_of_all_mass, pose_index);
            now_target_point.y = feets[i].garound_height;
            now_root_point.y = feets[i].garound_height + legs[i].all_base_root_height_bias[pose_index];

            if (i == moveing_leg_index)
            {
                now_target_point = PlanNowTargetPoint(i, now_target_point);
            } 
            else
            {
                // keep the feet keep point in range with end point
                Vector3 feet_world_keep_point = feets[i].now_ground.TransformPoint(feets_keep_point[i]);
                Vector3 now_feet_plane_position = Vector3.ProjectOnPlane(feets[i].GetFeetJointPosition(), Vector3.up);
                Vector3 now_keep_back_direction = Vector3.ProjectOnPlane(now_feet_plane_position - feet_world_keep_point, Vector3.up);
                if (now_keep_back_direction.magnitude > max_feet_keep_radius)
                    feet_world_keep_point += now_keep_back_direction.normalized * (now_keep_back_direction.magnitude - max_feet_keep_radius);
                if (now_keep_back_direction.magnitude > min_feet_keep_radius)
                    feet_world_keep_point = Vector3.MoveTowards(feet_world_keep_point, now_feet_plane_position, feet_keep_back_speed * Time.fixedDeltaTime);

                // keep feet down the ground
                if (!feets[i].ground_contactor.contact_ground)
                    feet_world_keep_point.y = Mathf.Max(feet_world_keep_point.y - keep_feet_down_speed * Time.fixedDeltaTime, -feets[i].constant_feet_height);
                now_target_point = new Vector3(feet_world_keep_point.x, now_target_point.y + feet_world_keep_point.y, feet_world_keep_point.z);
                feets_keep_point[i] = feets[i].now_ground.InverseTransformPoint(feet_world_keep_point);
            }
            LimbIKMoveToTargrtPoint(i, now_target_point, now_root_point, now_pole_original_direction, now_original_target_c);
        }
    }

    Vector3 PlanNowTargetPoint(int feet_index, Vector3 target_feet_point)
    {
        PhysicalLeg leg = legs[feet_index];
        BalanceFeet feet= feets[feet_index];
        int oter_feet_index = GetOtherFeetIndex(feet_index);
        PhysicalLeg other_leg = legs[oter_feet_index];
        // Use root speed on horizontal plane to lerp the length of additional feet movement that will shift target position
        Vector3 linear_velocity_add_move = root_plane_linear_velocity.normalized * 
            Mathf.Lerp(0f, Linear_max_over_feet_radius, Mathf.InverseLerp(min_over_feet_root_linear_speed, max_over_feet_root_linear_speed, root_plane_linear_velocity.magnitude));

        // Use root rotation different with initial pose to lerp the length of additional feet movement that will shift target position
        Quaternion now_balance_rotation = center_of_all_mass.rotation * CoM_balance_rotation_bias[pose_index];
        Vector3 axis = CalculateEulerAngleDiff(now_balance_rotation, CoM_rotator.rotation);
        axis = Vector3.ProjectOnPlane(axis, Vector3.up);
        Vector3 rot_vel_direction = (Quaternion.AngleAxis(-90f, Vector3.up) * axis).normalized;
        float angle = Quaternion.Angle(center_of_all_mass.rotation, CoM_rotator.rotation);
        Vector3 rotation_add_move = rot_vel_direction * Mathf.Lerp(0f, rotate_max_over_feet_radius, Mathf.InverseLerp(min_over_feet_root_rot, max_over_feet_root_rot, angle));

        // Use root angular velocity to lerp the length of additional feet movement that will shift target position
        Vector3 root_plane_angular_velocity = Vector3.ProjectOnPlane(root_rg.angularVelocity, Vector3.up);
        Vector3 ang_vel_direction = (Quaternion.AngleAxis(-90f, Vector3.up) * root_plane_angular_velocity).normalized;
        Vector3 angular_velocity_add_move = ang_vel_direction * 
            Mathf.Lerp(0f, angular_vel_max_over_feet_radius, Mathf.InverseLerp(min_over_feet_root_angular_speed, max_over_feet_root_angular_speed, root_plane_angular_velocity.magnitude));

        target_feet_point += linear_velocity_add_move + rotation_add_move + angular_velocity_add_move;  

        Vector3 start_feet_point = center_of_all_mass.TransformPoint(feet_start_bias);
        Vector3 feet_move_direction = target_feet_point - start_feet_point;
        feet_move_direction = Vector3.ProjectOnPlane(feet_move_direction, Vector3.up);

        float feet_move_speed = feet_move_direction.magnitude / feet_move_time;
        Vector3 now_feet_move_length = feet_move_direction.normalized * feet_move_speed * Time.fixedDeltaTime;

        Vector3 now_feet_position = feet.GetFeetJointPosition();
        Vector3 feet_already_move_length = Vector3.Project(now_feet_position - start_feet_point, feet_move_direction);
        if (Vector3.Dot(feet_already_move_length, feet_move_direction) < 0f) feet_already_move_length = Vector3.zero;
        // At begining of this feet moving
        if (now_gait_duration == 1f)
        {
            now_gait_duration = 0f;
            // Determine the gait value
            if (!feet.ground_contactor.contact_ground && start_feet_point.y > feet.garound_height + gait_gate_height)
            {
                now_gait_curve = drop_gait_curve;
                now_max_gait_height = start_feet_point.y - feet.garound_height;
            }
            else
            {
                now_gait_curve = full_gait_curve;
                now_max_gait_height = gait_max_height;                
            }
            // Determine direction of feet avoid obstacle, relative to target feet move direction;
            Vector3 root_joint_diff_direction = Vector3.ProjectOnPlane(other_leg.GetNowJointPosition(2) - leg.GetNowJointPosition(2), Vector3.up);
            Vector3 feet_joint_diff_direction = Vector3.ProjectOnPlane(other_leg.GetNowJointPosition(0) - now_feet_position, Vector3.up);
            feet_angle_avoid_sign = Vector3.SignedAngle(root_joint_diff_direction, feet_move_direction, Vector3.up) + 
                                    Vector3.SignedAngle(feet_joint_diff_direction, feet_move_direction, Vector3.up) < 0f ? -1f : 1f;           
        }
        // Update gait curve time and get current target position for feet joint
        now_gait_duration = Mathf.Max(Mathf.InverseLerp(0f, feet_move_direction.magnitude, feet_already_move_length.magnitude),
            now_gait_duration + Mathf.InverseLerp(0f, feet_move_direction.magnitude, now_feet_move_length.magnitude));
        if (now_gait_duration >= 1f)
        {
            now_gait_duration = 1f;
            is_feet_moving = false;
        }
        Vector3 now_target_point = start_feet_point + feet_move_direction * now_gait_duration;
        now_target_point.y = Mathf.Lerp(feet.garound_height, Mathf.Max(feet.garound_height + now_max_gait_height, now_feet_position.y), now_gait_curve.Evaluate(now_gait_duration));

        // Leg avoid other leg
        Vector3 now_knee_position = leg.GetNowJointPosition(1);
        Vector3 other_knee_position = other_leg.GetNowJointPosition(1);
        Vector3 other_feet_position = other_leg.GetNowJointPosition(0);

        Vector3 project_knee_point = Vector3.ProjectOnPlane(now_knee_position, feet_move_direction);
        Vector3 project_feet_point = Vector3.ProjectOnPlane(now_feet_position, feet_move_direction);
        Vector3 project_other_knee_point = Vector3.ProjectOnPlane(other_knee_position, feet_move_direction);
        Vector3 project_other_feet_point = Vector3.ProjectOnPlane(other_feet_position, feet_move_direction);

        Vector3 knee_closest_point_dir = project_knee_point - GetClosestPointOnFiniteLine(project_knee_point, project_other_feet_point, project_other_knee_point);
        Vector3 feet_closest_point_dir = project_feet_point - GetClosestPointOnFiniteLine(project_feet_point, project_other_feet_point, project_other_knee_point);
        Vector3 other_knee_closest_point_dir = GetClosestPointOnFiniteLine(project_other_knee_point, project_feet_point, project_knee_point) - project_other_knee_point;
        Vector3 other_feet_closest_point_dir = GetClosestPointOnFiniteLine(project_other_feet_point, project_feet_point, project_knee_point) - project_other_feet_point;

        Vector3 vertical_dir = Quaternion.AngleAxis(feet_angle_avoid_sign * 90f, Vector3.up) * feet_move_direction.normalized;
        Vector3 knee_diff_move_dir = Vector3.ProjectOnPlane(other_knee_position - now_knee_position, Vector3.up);
        Vector3 feet_diff_move_dir = Vector3.ProjectOnPlane(other_feet_position - now_feet_position, Vector3.up);
        Vector3 target_dir = Vector3.ProjectOnPlane(target_feet_point - now_feet_position, Vector3.up);
        bool leg_overlap = ((Vector3.Dot(knee_diff_move_dir, target_dir) > 0f && Vector3.Project(knee_diff_move_dir, feet_move_direction).magnitude < target_dir.magnitude) || 
                            (Vector3.Dot(feet_diff_move_dir, target_dir) > 0f && Vector3.Project(feet_diff_move_dir, feet_move_direction).magnitude < target_dir.magnitude)) && 
                            ((Vector3.Dot(vertical_dir, knee_closest_point_dir) < 0f || knee_closest_point_dir.magnitude < min_leg_avoid_length) || 
                            (Vector3.Dot(vertical_dir, feet_closest_point_dir) < 0f || feet_closest_point_dir.magnitude < min_leg_avoid_length) || 
                            (Vector3.Dot(vertical_dir, other_knee_closest_point_dir) < 0f || other_knee_closest_point_dir.magnitude < min_leg_avoid_length) || 
                            (Vector3.Dot(vertical_dir, other_feet_closest_point_dir) < 0f || other_feet_closest_point_dir.magnitude < min_leg_avoid_length));
        
        if (leg_overlap)
            feet_vertical_move = Vector3.MoveTowards(feet_vertical_move, vertical_dir.normalized * max_vertical_avoid_length, avoid_move_speed * Time.fixedDeltaTime);       
        else
            feet_vertical_move = Vector3.MoveTowards(feet_vertical_move, Vector3.zero, feet_avoid_back_speed * Time.fixedDeltaTime);

        now_target_point += feet_vertical_move;


        // Debug
        for (float t = now_gait_duration; t <= 1f; t += 0.1f)
        {
            Vector3 v = start_feet_point + feet_move_direction * t;
            v.y = Mathf.Lerp(feet.garound_height, Mathf.Max(feet.garound_height + now_max_gait_height, now_feet_position.y), now_gait_curve.Evaluate(t));
            draw_trajectory.Add(v);
        }
        draw_target.Add(target_feet_point);
        target_colors.Add(Color.green);  
        
        draw_lines.Add(new Tuple<Vector3, Vector3>(root_rg.position, root_rg.position + linear_velocity_add_move));
        draw_lines.Add(new Tuple<Vector3, Vector3>(root_rg.position, root_rg.position + rotation_add_move)); 
        draw_lines.Add(new Tuple<Vector3, Vector3>(root_rg.position, root_rg.position + angular_velocity_add_move));
        lines_colors.Add(Color.black);
        lines_colors.Add(Color.blue);
        lines_colors.Add(Color.gray);

        if (leg_overlap)
        {
            draw_lines.Add(new Tuple<Vector3, Vector3>(now_feet_position, now_feet_position + feet_vertical_move.normalized));
            lines_colors.Add(Color.green);

            Debug.DrawLine(now_knee_position, now_knee_position - knee_closest_point_dir, Color.yellow);
            Debug.DrawLine(now_feet_position, now_feet_position - feet_closest_point_dir, Color.yellow);
            Debug.DrawLine(other_knee_position, other_knee_position + other_knee_closest_point_dir, Color.yellow);
            Debug.DrawLine(other_feet_position, other_feet_position + other_feet_closest_point_dir, Color.yellow);
        }

        return now_target_point;
    }

    /// <summary>
    /// Moving leg to given target using IK->PD controller
    /// </summary>
    /// <param name="feet_index"> current feet index </param>
    /// <param name="now_target_point"> current target feet joint world position </param>
    /// <param name="now_root_point"> current target leg root joint world position </param>
    /// <param name="now_pole_original_direction"> current pole original world direction </param>
    /// <param name="now_original_target_c"> current original world direction of leg root joint to feet joint </param>
    void LimbIKMoveToTargrtPoint(int feet_index, Vector3 now_target_point, Vector3 now_root_point, Vector3 now_pole_original_direction, Vector3 now_original_target_c)
    {
        /*
                            c
           leg root joint ______ feet joint
                          \    /
                         a \  / b
                            \/
                        knee joint
        */

        PhysicalLeg leg = legs[feet_index];
        ConfigurableJoint joint_u = leg.joints[2];
        ConfigurableJoint joint_l = leg.joints[1];
        float upper_leg_length = leg.link_length[1];
        float lower_leg_length = leg.link_length[0];

        // Get base height of upper joint for character to maintain
        Vector3 joint_pos_u = joint_u.connectedBody.transform.TransformPoint(joint_u.connectedAnchor);
        joint_pos_u = Vector3.MoveTowards(joint_pos_u, now_root_point, keep_root_up_speed * Time.fixedDeltaTime);

        Vector3 target_c = now_target_point - joint_pos_u;
        float max_leg_extend_length = upper_leg_length + lower_leg_length;
        if (target_c.magnitude > max_leg_extend_length)
            target_c = target_c.normalized * max_leg_extend_length;

        float angle_u = LawOfCosines(upper_leg_length, target_c.magnitude, lower_leg_length);
        Vector3 pole_middle_point = joint_pos_u + target_c.normalized * upper_leg_length * Mathf.Cos(angle_u);
        float angle_diff_target_c = Vector3.Angle(now_original_target_c, target_c);
        Vector3 axis_diff_target_c = Vector3.Cross(now_original_target_c, target_c);
        Vector3 pole_direction = Quaternion.AngleAxis(angle_diff_target_c, axis_diff_target_c) * now_pole_original_direction.normalized * upper_leg_length;
        Vector3 now_pole_point = pole_middle_point + pole_direction;
        Vector3 ik_plane_normal = Vector3.Cross(target_c, now_pole_point - joint_pos_u);

        Vector3 target_a = Quaternion.AngleAxis(angle_u * Mathf.Rad2Deg, ik_plane_normal) * target_c.normalized * upper_leg_length;
        Vector3 joint_pos_l = joint_pos_u + target_a;
        Vector3 target_b = now_target_point - joint_pos_l;
        // Transform link-ik axis rotation from world-local-joint space
        Quaternion ik_joint_u_target_wrd_rot = Quaternion.LookRotation(target_a.normalized, ik_plane_normal);
        Quaternion ik_joint_u_target_loc_rot = Quaternion.Inverse(joint_u.connectedBody.transform.rotation) * ik_joint_u_target_wrd_rot;
        Quaternion joint_u_target_loc_rot = ik_joint_u_target_loc_rot * leg.all_ik_axis_bias[pose_index].quaternions[1];

        Quaternion ik_joint_l_target_wrd_rot = Quaternion.LookRotation(target_b.normalized, ik_plane_normal);
        Quaternion ik_joint_l_target_loc_rot = Quaternion.Inverse(joint_l.connectedBody.transform.rotation) * ik_joint_l_target_wrd_rot;
        Quaternion joint_l_target_loc_rot = ik_joint_l_target_loc_rot * leg.all_ik_axis_bias[pose_index].quaternions[0];
        
        SetJointPDTarget(all_joints.IndexOf(joint_u), joint_u_target_loc_rot);
        SetJointPDTarget(all_joints.IndexOf(joint_l), joint_l_target_loc_rot);

        // Use pole info to set feet fit the ground surface
        feets[feet_index].FeetFitSurface(pole_direction.normalized);

        // Debug
        Vector3 ik_plane_center = (joint_pos_l + joint_pos_u + now_target_point) / 3f;
        draw_lines.Add(new Tuple<Vector3, Vector3>(ik_plane_center, ik_plane_center + ik_plane_normal));
        lines_colors.Add(Color.white);

        draw_triangles.Add(joint_pos_u);
        draw_triangles.Add(joint_pos_u + target_a);
        draw_triangles.Add(now_target_point);

        draw_lines.Add(new Tuple<Vector3, Vector3>(pole_middle_point, now_pole_point));
        lines_colors.Add(Color.blue);          
    }

    void PhysicalAnimationRootFollow()
    {
        CalculateRootTransformInfo();

        // Determine proportion of effort to make root rigidbody follow the target transform according the relation between center of balance and feets that on ground's position
        float back_force_scaler = 1f;
        if (in_self_balance_mode)
        {
            Vector3 center_of_balance = Vector3.zero;
            Vector3 center_of_feets = Vector3.zero;
            for (int i = 0; i < feets.Length; i++)
            {
                center_of_balance += center_of_all_mass.TransformPoint(legs[i].all_base_feet_bias[pose_index]);
                center_of_feets += feets[i].GetFeetJointPosition();
            }
                
            center_of_balance /= feets.Length;
            center_of_feets /= feets.Length;
            center_of_balance = Vector3.ProjectOnPlane(center_of_balance, Vector3.up);
            center_of_feets = Vector3.ProjectOnPlane(center_of_feets, Vector3.up);

            Dictionary<int, Vector3> feet_center_diffs = new Dictionary<int, Vector3>();
            for (int i = 0; i < feets.Length; i++)
            {
                if (feets[i].ground_contactor.contact_ground)
                {
                    Vector3 feet_center_diff = feets[i].GetFeetJointPosition() - center_of_balance;
                    feet_center_diff = Vector3.ProjectOnPlane(feet_center_diff, Vector3.up);
                    feet_center_diffs.Add(i, feet_center_diff);
                }
            }
            // When feets at same side of center of balance, there usuall only the feet that closest to the center of balance take all the weight, so we calcutale value we need from that
            if (!(feet_center_diffs.Count == 2 && Vector3.Dot(feet_center_diffs[0], feet_center_diffs[1]) <= 0f))
            {
                back_force_scaler = 0f;               
                foreach (Vector3 feet_center_diff in feet_center_diffs.Values)
                    back_force_scaler = Mathf.Max(back_force_scaler, Mathf.InverseLerp(max_feet_balance_radius, min_feet_balance_radius, feet_center_diff.magnitude));
            }

            // Moving reference root body back to center_of_feets so the feet fit target area better
            Vector3 root_target_diff =  Vector3.ProjectOnPlane(transform.position - center_of_balance, Vector3.up);
            Vector3 ref_root_parent_diff = Vector3.ProjectOnPlane(ref_root_transform.parent.position - ref_root_transform.position, Vector3.up);
            Vector3 ref_root_target_position = new Vector3(center_of_feets.x, ref_root_transform.parent.position.y, center_of_feets.z) + root_target_diff + ref_root_parent_diff;
            ref_root_transform.parent.position = Vector3.MoveTowards(ref_root_transform.parent.position, ref_root_target_position, back_force_scaler * center_meet_speed * Time.fixedDeltaTime);
            // 
            now_reverse_move_vel += (center_of_balance - center_of_feets).normalized * max_reverse_move_add_speed * (1f - back_force_scaler);
            if (now_reverse_move_vel.magnitude > max_reverse_move_speed) now_reverse_move_vel = now_reverse_move_vel.normalized * max_reverse_move_speed;
            ref_root_transform.parent.position += now_reverse_move_vel * Time.fixedDeltaTime;
            now_reverse_move_vel = Vector3.MoveTowards(now_reverse_move_vel, Vector3.zero, back_force_scaler * reverse_move_decline_speed * Time.fixedDeltaTime);
        }

        // Keep ref root transform in range with self transform (physical root transform)
        Vector3 pos_diff = Vector3.ProjectOnPlane(transform.position - ref_root_transform.position, Vector3.up);
        if (pos_diff.magnitude > max_distance_diff)
        {
            pos_diff = pos_diff.normalized * (pos_diff.magnitude - max_distance_diff);
            ref_root_transform.parent.position += pos_diff;
        }
        float rot_diff = CalculateEulerAngleDiff(ref_root_transform.rotation, transform.rotation).y;
        if (Mathf.Abs(rot_diff) > max_angle_diff)
        {
            float rot_sign = rot_diff < 0f ? -1f : 1f;
            rot_diff = rot_diff - rot_sign * max_angle_diff;
            ref_root_transform.parent.rotation = Quaternion.LookRotation(Quaternion.AngleAxis(rot_diff, Vector3.up) * ref_root_transform.parent.forward, Vector3.up);
        }

        SetAllDrag(back_force_scaler);
        SetRootPD(ref_root_transform.position, root_linear_velocity, ref_root_transform.rotation, root_angular_velocity, back_force_scaler);
    }

    /// <summary>
    /// Make the physical body follow the animation
    /// </summary>
    void PhysicalAnimationBodyFollow()
    {
        for (int i = 1; i < all_rgs.Length; i++)
        {
            // When in self balance mode, the leg shouldn't use the PD controller again
            bool joint_can_move = true;
            if (in_self_balance_mode)
            {
                foreach (var leg in legs)
                {
                    if (leg.leg_rgs.Contains(all_rgs[i]))
                    {
                        joint_can_move = false;
                        break;
                    }
                }
            }
            // Use the PD controller directly
            if (joint_can_move)
            {     
                Quaternion joint_target_local_rotation = GetBodyLoaclRotation(all_rgs[i].name);
                SetJointPDTarget(i-1, joint_target_local_rotation);
            }
        }
    }

    void SetAllDrag(float drag_scaler)
    {
        // Drag is good for maintain the physical animation more accurately
        for (int i = 0; i < all_rgs.Length; i++)
        {
            all_rgs[i].drag = Mathf.Lerp(0f, all_rgs_drags[i][0], drag_scaler);
            all_rgs[i].angularDrag = Mathf.Lerp(0.05f, all_rgs_drags[i][1], drag_scaler);
        }
    }

    void SetRootPD(Vector3 target_position, Vector3 target_linear_velocity, Quaternion target_rotation, Vector3 target_angular_velocity, float back_force_scaler)
    {
        float root_pd_position_param = 0f, root_pd_velocity_param= 0f, root_pd_rotation_param= 0f, root_pd_angular_velocity_param= 0f;
        if (in_self_balance_mode)
        {
            root_pd_position_param = root_sb_position_param;
            root_pd_velocity_param = root_sb_velocity_param;
            root_pd_rotation_param = root_sb_rotation_param;
            root_pd_angular_velocity_param = root_sb_angular_velocity_param; 
        }
        else
        {
            root_pd_position_param = root_pa_position_param;
            root_pd_velocity_param = root_pa_velocity_param;
            root_pd_rotation_param = root_pa_rotation_param;
            root_pd_angular_velocity_param = root_pa_angular_velocity_param;           
        }
        // Follow position and linear velocity in world-space
        Vector3 force = root_pd_position_param * (target_position - transform.position) + root_pd_velocity_param * (target_linear_velocity - root_rg.velocity);
        root_rg.AddForce(force * back_force_scaler, ForceMode.Acceleration);
        // Follow rotation and angular velocity in world-space
        Vector3 rot_diff = CalculateEulerAngleDiff(transform.rotation, target_rotation);
        Vector3 torque = root_pd_rotation_param * rot_diff + root_pd_angular_velocity_param * (target_angular_velocity - root_rg.angularVelocity);
        root_rg.AddTorque(torque * back_force_scaler, ForceMode.Acceleration);        
    }

    void SetJointPDTarget(int joint_index, Quaternion target_local_rotation)
    {
        ConfigurableJoint joint = all_joints[joint_index];
        Quaternion world_to_joint_space = world_to_joints_axis_space[joint_index];
        Quaternion joint_to_world_space = joints_to_world_axis_space[joint_index];
        Quaternion axis_rotation = joints_init_axis_rotation[joint_index];
        // Control the max rotate speed.
        Quaternion now_target_rotation = Quaternion.RotateTowards(joint.transform.localRotation, target_local_rotation, joint_rotate_speed * Time.fixedDeltaTime);
        // Matrix change of basis, Transform different of local rotation back into joint space
        Quaternion joint_target_local_rotation = joint_to_world_space * (Quaternion.Inverse(now_target_rotation) * axis_rotation) * world_to_joint_space;
        // Set target rotation to our newly calculated rotation
        joint.targetRotation = joint_target_local_rotation;    

        // Set target angular velocity according to last and current real joint target rotation
        Quaternion real_joint_target_local_rotation = joint_to_world_space * (Quaternion.Inverse(now_target_rotation) * axis_rotation) * world_to_joint_space;
        Quaternion last_joint_target_local_rotation = last_joints_local_rotation[joint_index];
        last_joints_local_rotation[joint_index] = real_joint_target_local_rotation;
        Vector3 now_target_angular_velocity = CalculateAngularVelocity(last_joint_target_local_rotation, real_joint_target_local_rotation);
        joint.targetAngularVelocity = now_target_angular_velocity;
    }

    Quaternion GetBodyLoaclRotation(string body_name)
    {
        Transform ref_body = ref_body_dict[body_name];
        Quaternion local_rotation = ref_body.localRotation;
        return local_rotation;
    }

    void CalculateRootTransformInfo()
    {
        // Calculate ref root transform linear velocity
        root_linear_velocity = (ref_root_transform.position - last_root_position) / Time.fixedDeltaTime;
        last_root_position = ref_root_transform.position;

        // Calculate ref root transform angular velocity
        root_angular_velocity = CalculateAngularVelocity(last_root_rotation, ref_root_transform.rotation);
        last_root_rotation = ref_root_transform.rotation;
    }

    Vector3 CalculateAngularVelocity(Quaternion last_rotation, Quaternion now_rotation)
    {
        Vector3 euler_rotation = CalculateEulerAngleDiff(last_rotation, now_rotation);
        Vector3 angular_velocity = euler_rotation / Time.fixedDeltaTime * Mathf.Deg2Rad;
        return angular_velocity;
    }

    Vector3 CalculateEulerAngleDiff(Quaternion last_rotation, Quaternion now_rotation)
    {
        Quaternion delta_rotation = now_rotation * Quaternion.Inverse(last_rotation);
        Vector3 euler_rotation = new Vector3(
            Mathf.DeltaAngle(0, delta_rotation.eulerAngles.x),
            Mathf.DeltaAngle(0, delta_rotation.eulerAngles.y),
            Mathf.DeltaAngle(0, delta_rotation.eulerAngles.z));
        return euler_rotation;
    }

    float LawOfCosines(float a, float b, float c)
    {
        /*
               c
           B ______ A
             \    /
            a \  / b
               \/
               C
        */
        float cosC = (Mathf.Pow(a, 2f) + Mathf.Pow(b, 2f) - Mathf.Pow(c, 2f)) / (2f * a * b);
        float C = Mathf.Acos(Mathf.Clamp(cosC, -1f, 1f));
        return C;
    }

    // For finite lines:
    Vector3 GetClosestPointOnFiniteLine(Vector3 point, Vector3 line_start, Vector3 line_end)
    {
        Vector3 line_direction = line_end - line_start;
        float line_length = line_direction.magnitude;
        line_direction.Normalize();
        float project_length = Mathf.Clamp(Vector3.Dot(point - line_start, line_direction), 0f, line_length);
        return line_start + line_direction * project_length;
    }

    // For infinite lines:
    Vector3 GetClosestPointOnInfiniteLine(Vector3 point, Vector3 line_start, Vector3 line_end)
    {
        return line_start + Vector3.Project(point - line_start, line_end - line_start);
    }

    int GetOtherFeetIndex(int feet_index)
    {
        return feet_index == 0 ? 1 : 0;
    }


    void OnDrawGizmos()
    {
        if (is_debug)
        {
            // Draw IK triangles
            for (int i = 0; i < draw_triangles.Count; i++)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawLine(draw_triangles[i], draw_triangles[(i+1) % 3 == 0 ? i-2 : i+1]);
                Gizmos.DrawSphere(draw_triangles[i], 0.05f);
            }
            // Draw line
            for (int i = 0; i < draw_lines.Count; i++)
            {
                Gizmos.color = lines_colors[i];
                Gizmos.DrawLine(draw_lines[i].Item1, draw_lines[i].Item2);
            }
            // Draw trajectory
            for (int i = 0; i < draw_trajectory.Count; i++)
            {
                Gizmos.color = Color.yellow;
                if (i > 0)
                {
                    Gizmos.DrawLine(draw_trajectory[i], draw_trajectory[i-1]);
                }
                Gizmos.DrawSphere(draw_trajectory[i], 0.025f);
            }
            // Draw target
            for (int i = 0; i < draw_target.Count; i++)
            {
                Gizmos.color = target_colors[i];
                Gizmos.DrawWireSphere(draw_target[i], base_feet_radius);
            }
        }
    }
}
[System.Serializable]
public class PhysicalLeg
{
    /// <summary>
    /// Name of each balance pose
    /// </summary>
    public List<string> pose_names = new List<string>();
    [HideInInspector]
    public List<Rigidbody> leg_rgs = new List<Rigidbody>();
    [HideInInspector]
    public List<ConfigurableJoint> joints = new List<ConfigurableJoint>();
    [HideInInspector]
    public List<float> link_length = new List<float>();
    [HideInInspector]
    public List<ListQuaternion> all_ik_axis_bias = new List<ListQuaternion>();
    [System.Serializable]
    public class ListQuaternion
    {
        public List<Quaternion> quaternions = new List<Quaternion>();
    }
    [HideInInspector]
    public List<Vector3> all_base_feet_bias = new List<Vector3>();
    [HideInInspector]
    public List<Vector3> all_base_root_bias = new List<Vector3>();
    [HideInInspector]
    public List<Vector3> all_base_pole_direction = new List<Vector3>();
    [HideInInspector]
    public List<Vector3> all_base_target_c_direction = new List<Vector3>();
    [HideInInspector]
    public List<float> all_base_root_height_bias = new List<float>();

    public PhysicalLeg(BalanceFeet feet)
    {
        ConfigurableJoint leg_joint = feet.GetComponent<ConfigurableJoint>();
        Vector3 last_joint_position = leg_joint.connectedBody.transform.TransformPoint(leg_joint.connectedAnchor);
        while (true)
        {
            joints.Add(leg_joint);
            leg_rgs.Add(leg_joint.GetComponent<Rigidbody>());
            leg_joint = leg_joint.connectedBody.GetComponent<ConfigurableJoint>();
            if (leg_joint == null)
                break;
            Vector3 now_joint_position = leg_joint.connectedBody.transform.TransformPoint(leg_joint.connectedAnchor);
            link_length.Add((last_joint_position - now_joint_position).magnitude);
            last_joint_position = now_joint_position;
        }
    }
    /// <summary>
    /// Calculate the info the leg needed in self balance to transform the all targrt world position in the joint space rotation
    /// </summary>
    public void ResetParameters(Transform center_of_all_mass, BalanceFeet feet, string add_pose_name = "", bool calculate_feet_data = true, 
        bool calculate_pole_by_leg = true, bool calculate_pole_by_feet = false, Vector3 leg_pole_world_direction = new Vector3())
    {
        Vector3 feet_joint_position = GetNowJointPosition(0);
        Vector3 base_feet_bias = center_of_all_mass.InverseTransformPoint(feet_joint_position);
        Vector3 root_joint_position = GetNowJointPosition(joints.Count-1);
        Vector3 base_root_bias = center_of_all_mass.InverseTransformPoint(root_joint_position);

        float base_root_height_bias = root_joint_position.y - feet_joint_position.y;

        Vector3 knee_joint_position = GetNowJointPosition(1);
        Vector3 target_c = feet_joint_position - root_joint_position;
        Vector3 target_a = knee_joint_position - root_joint_position;
        float upper_leg_length = target_a.magnitude;
        float angle_u = Vector3.Angle(target_a, target_c);
        Vector3 pole_middle_point = root_joint_position + target_c.normalized * upper_leg_length * Mathf.Cos(angle_u);

        Vector3 knee_pole_direction = Vector3.ProjectOnPlane(knee_joint_position - pole_middle_point, target_c);
        Vector3 back_up_pole_direction = Vector3.ProjectOnPlane(feet.toes.position - feet.transform.position, target_c);
        Vector3 pole_direction;
        if (calculate_pole_by_leg)
            pole_direction = knee_pole_direction;
        else if (calculate_pole_by_feet)
            pole_direction = back_up_pole_direction;
        else
            pole_direction = leg_pole_world_direction;

        Vector3 pole_world_position = pole_middle_point + pole_direction.normalized * upper_leg_length;
        Vector3 ik_plane_normal = Vector3.Cross(feet_joint_position - root_joint_position, pole_world_position - root_joint_position);
        
        Vector3 base_pole_direction = center_of_all_mass.InverseTransformDirection(pole_direction.normalized);
        Vector3 base_target_c_direction = center_of_all_mass.InverseTransformDirection(target_c.normalized);
        // Use to transform link-ik axis rotation to the joint space rotation later on
        var ik_axis_bias = new ListQuaternion();
        for (int i = 1; i < joints.Count; i++)
        {
            Quaternion ik_wrd_rot = Quaternion.LookRotation((GetNowJointPosition(i-1) - GetNowJointPosition(i)).normalized, ik_plane_normal);
            Quaternion ik_loc_rot = Quaternion.Inverse(joints[i].connectedBody.transform.rotation) * ik_wrd_rot;
            ik_axis_bias.quaternions.Add(Quaternion.Inverse(ik_loc_rot) * joints[i].transform.localRotation);
        }

        pose_names.Add(add_pose_name);
        all_ik_axis_bias.Add(ik_axis_bias);
        all_base_feet_bias.Add(base_feet_bias);
        all_base_root_bias.Add(base_root_bias);
        all_base_pole_direction.Add(base_pole_direction);
        all_base_target_c_direction.Add(base_target_c_direction);
        all_base_root_height_bias.Add(base_root_height_bias);

        if (calculate_feet_data)
            feet.FeetStart(pole_direction.normalized);
    }

    public Vector3 GetNowFeetBias(Transform center_of_all_mass)
    {
        Vector3 feet_joint_position = GetNowJointPosition(0);
        return center_of_all_mass.InverseTransformPoint(feet_joint_position);
    }

    public Tuple<Vector3, Vector3, Vector3, Vector3> GetWorldBias(Transform center_of_all_mass, int pose_index)
    {   
        Vector3 world_base_feet_bias = center_of_all_mass.TransformPoint(all_base_feet_bias[pose_index]);
        Vector3 world_base_root_bias = center_of_all_mass.TransformPoint(all_base_root_bias[pose_index]);
        Vector3 world_base_pole_direction = center_of_all_mass.TransformDirection(all_base_pole_direction[pose_index]);
        Vector3 world_base_target_c_direction = center_of_all_mass.TransformDirection(all_base_target_c_direction[pose_index]);
        return new Tuple<Vector3, Vector3, Vector3, Vector3>(world_base_feet_bias, world_base_root_bias, world_base_pole_direction, world_base_target_c_direction);
    }

    public Vector3 GetNowJointPosition(int i)
    {
        return joints[i].connectedBody.transform.TransformPoint(joints[i].connectedAnchor);
    }
}
