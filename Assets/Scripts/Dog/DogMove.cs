using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DogMove : MonoBehaviour
{
    public Transform test_target;
    [Header("Self balance parameters")]
    public float keep_back_speed = 5f;
    public float feet_move_min_speed = 4f;
    public float feet_move_max_speed = 8f;
    /// <summary>
    /// Use the root rigidbody's fall speed to determine the feet move speed
    /// </summary>
    public float root_max_speed_gate = -3f;
    public float base_feet_radius = 0.4f;
    public float gait_max_height = 0.3f;
    public AnimationCurve gait_curve;

    [Header("External parameters")]
    public float external_acceleration = 1f;
    public float squat_distance = 0.75f;
    Rigidbody root_rg;
    List<Rigidbody> all_rgs;
    ConfigurableJoint joint_lu;
    ConfigurableJoint joint_ll;
    ConfigurableJoint joint_ru;
    ConfigurableJoint joint_rl;
    Transform leg_lu;
    Transform leg_ll;
    Transform leg_ru;
    Transform leg_rl;
    Transform feet_l;
    Transform feet_r;
    float upper_axis_angle;
    float lower_axis_angle;
    float upper_leg_length;
    float lower_leg_length;
    float max_leg_extend_length;
    float base_height;
    float base_left_feet_bias;
    float base_right_feet_bias;
    float base_left_root_bias;
    float base_right_root_bias;
    /// <summary>
    /// Recoard unit duration of gait from strat of that gait, use to get gait curve value, in range [0, 1]
    /// </summary>
    float now_gait_duration = 0f;
    float now_gait_duration_start = 0f;
    float now_gait_duration_scale = 0f;
    float start_point_y;
    float start_feet_bias;
    float now_max_distance;
    Vector3 center_of_all_mass;
    bool is_feet_moveing = false;
    bool is_right_feet;
    bool is_squat = false;
    const float constant_garound_height = -1.6f;
    // Debug 
    List<Vector3> draw_triangles = new List<Vector3>();
    List<Tuple<Vector3, Vector3>> draw_lines = new List<Tuple<Vector3, Vector3>>();
    List<Color> lines_colors = new List<Color>();
    List<Vector3> draw_trajectory = new List<Vector3>();

    // Start is called before the first frame update
    void Start()
    {
        root_rg = GetComponent<Rigidbody>();
        all_rgs = transform.parent.GetComponentsInChildren<Rigidbody>().ToList();

        leg_lu = GameObject.Find("Leg_LU").transform;
        leg_ll = GameObject.Find("Leg_LL").transform;
        leg_ru = GameObject.Find("Leg_RU").transform;
        leg_rl = GameObject.Find("Leg_RL").transform;
        feet_l = GameObject.Find("Feet_L").transform;
        feet_r = GameObject.Find("Feet_R").transform;
        joint_lu = leg_lu.GetComponent<ConfigurableJoint>();
        joint_ll = leg_ll.GetComponent<ConfigurableJoint>();
        joint_ru = leg_ru.GetComponent<ConfigurableJoint>();
        joint_rl = leg_rl.GetComponent<ConfigurableJoint>();

        upper_leg_length = (transform.TransformPoint(joint_lu.connectedAnchor) - leg_lu.TransformPoint(joint_ll.connectedAnchor)).magnitude;
        lower_leg_length = (leg_lu.TransformPoint(joint_ll.connectedAnchor) - feet_l.position).magnitude;
        max_leg_extend_length = upper_leg_length + lower_leg_length;

        upper_axis_angle = Vector3.SignedAngle(transform.forward, leg_lu.forward, Vector3.right);
        lower_axis_angle = Vector3.SignedAngle(-leg_lu.forward, -leg_ll.forward, Vector3.right);

        base_height = transform.TransformPoint(joint_lu.connectedAnchor).y; 

        CalculateCenterOfMass();
        base_left_feet_bias = center_of_all_mass.z - feet_l.position.z;
        base_right_feet_bias = feet_r.position.z - center_of_all_mass.z;
        base_left_root_bias = feet_l.position.z - transform.TransformPoint(joint_lu.connectedAnchor).z;
        base_right_root_bias = feet_r.position.z - transform.TransformPoint(joint_ru.connectedAnchor).z;

    }
    void CalculateCenterOfMass()
    {
        center_of_all_mass = Vector3.zero;
        float c = 0f;
        foreach (var rg in all_rgs)
        {
            center_of_all_mass += rg.worldCenterOfMass * rg.mass;
            c += rg.mass;
        }
        center_of_all_mass /= c;

        // Debug
        draw_lines.Add(new Tuple<Vector3, Vector3>(center_of_all_mass, center_of_all_mass - Vector3.up * 3f));
        lines_colors.Add(Color.black);
    }
    void SelfBalance()
    {
        CalculateCenterOfMass();
        if (!is_feet_moveing)
        {
            float now_left_feet_bias = center_of_all_mass.z - feet_l.position.z;
            float now_right_feet_bias = feet_r.position.z - center_of_all_mass.z;
            
            if (Mathf.Abs(base_left_feet_bias - now_left_feet_bias) > Mathf.Abs(base_right_feet_bias - now_right_feet_bias))
            {
                is_feet_moveing = base_left_feet_bias + base_feet_radius < now_left_feet_bias || now_left_feet_bias < base_left_feet_bias - base_feet_radius;
                is_right_feet = false;
                start_point_y = feet_l.position.y;
                start_feet_bias = now_left_feet_bias;
                now_max_distance = start_feet_bias - base_left_feet_bias;
            }
            else
            {
                is_feet_moveing = base_right_feet_bias + base_feet_radius < now_right_feet_bias || now_right_feet_bias < base_right_feet_bias - base_feet_radius;
                is_right_feet = true;
                start_point_y = feet_r.position.y;
                start_feet_bias = now_right_feet_bias;
                now_max_distance = start_feet_bias - base_right_feet_bias;
            }
        }

        Vector3 now_target_point_r = new Vector3(0f, constant_garound_height, feet_r.position.z);
        Vector3 now_target_point_l = new Vector3(0f, constant_garound_height, feet_l.position.z);
        float now_target_root_z_r = feet_r.position.z - base_right_root_bias;
        float now_target_root_z_l = feet_l.position.z - base_left_root_bias;
        if (is_feet_moveing)
        {
            if (is_right_feet)
                (now_target_point_r, now_target_root_z_r) = PlanNowTargetPoint(feet_r.position, base_right_feet_bias, base_right_root_bias);
            else
                (now_target_point_l, now_target_root_z_l) = PlanNowTargetPoint(feet_l.position, -base_left_feet_bias, base_left_root_bias);
        }
        IKMoveToTargrtPoint(joint_ru, joint_rl, now_target_point_r, now_target_root_z_r);
        IKMoveToTargrtPoint(joint_lu, joint_ll, now_target_point_l, now_target_root_z_l);

        // Debug
        Vector3 p_l_l = new Vector3(0f, constant_garound_height, center_of_all_mass.z - (base_left_feet_bias - base_feet_radius));
        Vector3 p_l_u = new Vector3(0f, constant_garound_height, center_of_all_mass.z - (base_left_feet_bias + base_feet_radius));
        Vector3 p_r_l = new Vector3(0f, constant_garound_height, center_of_all_mass.z + (base_right_feet_bias - base_feet_radius));
        Vector3 p_r_u = new Vector3(0f, constant_garound_height, center_of_all_mass.z + (base_right_feet_bias + base_feet_radius));
        draw_lines.Add(new Tuple<Vector3, Vector3>(p_l_l, p_l_l + Vector3.up * 0.2f));
        draw_lines.Add(new Tuple<Vector3, Vector3>(p_l_u, p_l_u + Vector3.up * 0.2f));
        draw_lines.Add(new Tuple<Vector3, Vector3>(p_r_l, p_r_l + Vector3.up * 0.2f));
        draw_lines.Add(new Tuple<Vector3, Vector3>(p_r_u, p_r_u + Vector3.up * 0.2f));
        draw_lines.Add(new Tuple<Vector3, Vector3>(p_l_l, p_l_u));
        draw_lines.Add(new Tuple<Vector3, Vector3>(p_r_l, p_r_u));
        for (int i = 0; i < 6; i++)
            lines_colors.Add(Color.green);
    }
    Tuple<Vector3, float> PlanNowTargetPoint(Vector3 feet_position, float base_feet_bias, float base_root_bias)
    {
        float target_point_z = center_of_all_mass.z + base_feet_bias;
        float start_point_z = center_of_all_mass.z - start_feet_bias; 

        //float now_feet_move_speed = Mathf.Lerp(feet_move_min_speed, feet_move_max_speed, Mathf.InverseLerp(0f, root_max_speed_gate, root_rg.velocity.y));
        float feet_move_sign = feet_position.z < target_point_z ? 1f : -1f;
        float now_feet_move_length = feet_move_sign * feet_move_min_speed * Time.fixedDeltaTime;
        float feet_already_move_length = feet_position.z - start_point_z;

        if (now_gait_duration == 0f)
        {
            while (now_gait_duration < 0.5f && Mathf.Lerp(constant_garound_height, constant_garound_height + gait_max_height, gait_curve.Evaluate(now_gait_duration)) < start_point_y)
            {
                now_gait_duration += 0.01f;
            }
            now_gait_duration_start = now_gait_duration;
            now_gait_duration_scale = 1f - now_gait_duration_start;
        }
        float new_gait_duration = Mathf.Max(now_gait_duration_start + Mathf.InverseLerp(0f, now_max_distance, feet_already_move_length) * now_gait_duration_scale,
            now_gait_duration + Mathf.InverseLerp(0f, now_max_distance, now_feet_move_length));
        if (new_gait_duration >= 1f)
        {
            now_feet_move_length *= (1f - now_gait_duration) / (new_gait_duration - now_gait_duration);
            now_gait_duration = 0f;
            is_feet_moveing = false;
        }
        else
        {
            now_gait_duration = new_gait_duration;
        }
        float now_gait_height = Mathf.Lerp(constant_garound_height, Mathf.Max(constant_garound_height + gait_max_height, feet_position.y), gait_curve.Evaluate(now_gait_duration));
        float now_target_point_z = start_point_z + now_max_distance * now_gait_duration + now_feet_move_length;

        Vector3 now_target_point = new Vector3(0f, now_gait_height, now_target_point_z);
        float now_target_root_z = target_point_z - base_root_bias;

        if (now_gait_duration > 0f)
        {
            // Debug
            for (float t = now_gait_duration; t <= 1f; t += 0.1f)
            {
                float h = Mathf.Lerp(constant_garound_height, Mathf.Max(constant_garound_height + gait_max_height, feet_position.y), gait_curve.Evaluate(t));
                float z = start_point_z + now_max_distance * t;
                draw_trajectory.Add(new Vector3(0f, h, z));
            }
        }
        
        return new Tuple<Vector3, float>(now_target_point, now_target_root_z);
    }
    void IKMoveToTargrtPoint(ConfigurableJoint joint_u, ConfigurableJoint joint_l, Vector3 now_target_point, float now_target_root_z)
    {
        // Get base height of upper joint for character to maintain
        Vector3 joint_pos_u = transform.TransformPoint(joint_u.connectedAnchor);
        float target_height;
        if (is_squat)
            target_height = base_height - squat_distance;
        else
            target_height = Mathf.Max(base_height, joint_pos_u.y);
        float now_base_height = Mathf.MoveTowards(joint_pos_u.y, target_height, keep_back_speed * Time.fixedDeltaTime);
        float now_base_root_z = Mathf.MoveTowards(joint_pos_u.z, now_target_root_z, keep_back_speed * Time.fixedDeltaTime);
        
        // c is vector from upper joint to target, a is vector from upper joint to lower joint, b is vector from lower joint to target
        joint_pos_u = new Vector3(0f, now_base_height, now_base_root_z);
        Vector3 target_c = now_target_point - joint_pos_u;
        if (target_c.magnitude > max_leg_extend_length)
            target_c = target_c.normalized * max_leg_extend_length;

        float angle_u = LawOfCosines(upper_leg_length, target_c.magnitude, lower_leg_length);
        Vector3 target_a = (Quaternion.Euler(angle_u, 0f, 0f) * target_c).normalized * upper_leg_length;
        Vector3 base_axis_u = Quaternion.Euler(upper_axis_angle, 0f, 0f) * transform.forward;
        float angle_diff_u = Vector3.SignedAngle(-target_a, base_axis_u, Vector3.right);

        Vector3 joint_pos_l = joint_pos_u + target_a;
        Vector3 target_b = now_target_point - joint_pos_l;
        Vector3 base_axis_l = Quaternion.Euler(lower_axis_angle, 0f, 0f) * target_a;
        float angle_diff_l = Vector3.SignedAngle(-target_b, base_axis_l, Vector3.right);

        joint_u.targetRotation = Quaternion.Euler(angle_diff_u, 0f, 0f);
        joint_l.targetRotation = Quaternion.Euler(angle_diff_l, 0f, 0f);

        // Debug
        //Debug.Log("Upper angle: " + angle_diff_u.ToString("F3") + " Quaternion: " + joint_u.targetRotation.ToString("F3"));
        //Debug.Log("Lower angle: " + angle_diff_l.ToString("F3") + " Quaternion: " + joint_l.targetRotation.ToString("F3"));
        draw_lines.Add(new Tuple<Vector3, Vector3>(joint_pos_u, joint_pos_u + base_axis_u));
        draw_lines.Add(new Tuple<Vector3, Vector3>(joint_pos_l, joint_pos_l + base_axis_l.normalized));
        lines_colors.Add(Color.blue);
        lines_colors.Add(Color.blue);
        draw_triangles.Add(joint_pos_u);
        draw_triangles.Add(joint_pos_u + target_a);
        draw_triangles.Add(now_target_point);

        //Debug.Log(root_rg.velocity.ToString("F2"));
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
        float C = Mathf.Acos(Mathf.Clamp(cosC, -1f, 1f)) * Mathf.Rad2Deg;
        return C;
    }

    void FixedUpdate()
    {
        // Debug
        draw_lines = new List<Tuple<Vector3, Vector3>>();
        lines_colors = new List<Color>();
        draw_triangles = new List<Vector3>();
        draw_trajectory = new List<Vector3>();

        Vector3 direction = new Vector3();
        if (Input.GetKey("a"))
        {
            direction += new Vector3(0f, 0f, -1f);
        }
        if (Input.GetKey("d"))
        {
            direction += new Vector3(0f, 0f, 1f);
        }
        direction = direction.normalized;
        root_rg.AddForce(direction * external_acceleration, ForceMode.Acceleration);

        is_squat = Input.GetKey("s");

        SelfBalance();
    }
    /*
    void OnDrawGizmos()
    {
        // Draw IK triangles
        for (int i = 0; i < draw_triangles.Count; i++)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(draw_triangles[i], draw_triangles[(i+1) % 3 == 0 ? i-2 : i+1]);
            Gizmos.DrawSphere(draw_triangles[i], 0.1f);
        }
        
        for (int i = 0; i < draw_lines.Count; i++)
        {
            Debug.DrawLine(draw_lines[i].Item1, draw_lines[i].Item2, lines_colors[i]);
        }
        
        for (int i = 0; i < draw_trajectory.Count; i++)
        {
            Gizmos.color = Color.yellow;
            if (i > 0)
            {
                Gizmos.DrawLine(draw_trajectory[i], draw_trajectory[i-1]);
            }
            Gizmos.DrawSphere(draw_trajectory[i], 0.025f);
        }
    }
    */
}
