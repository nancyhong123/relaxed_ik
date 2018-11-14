include("GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")
include("GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("GROOVE_Julia/groove.jl")
include("GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")
include("Utils_Julia/transformations.jl")

mutable struct RelaxedIK
    relaxedIK_vars
    groove
end

function RelaxedIK(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types; position_mode = "relative", rotation_mode = "relative", solver_name="slsqp")
    relaxedIK_vars = RelaxedIK_vars(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, position_mode = position_mode, rotation_mode = rotation_mode)
    groove = get_groove(relaxedIK_vars.vars, solver_name)
    return RelaxedIK(relaxedIK_vars, groove)
end


function get_standard(path_to_src, info_file_name; solver_name = "slsqp")
    objectives = [position_obj, rotation_obj, min_jt_vel_obj, min_jt_accel_obj]
    grad_types = ["forward_ad", "forward_ad", "forward_ad", "forward_ad"]
    weight_priors = [50., 49., 1.,1.]
    inequality_constraints = []
    ineq_grad_types = []
    equality_constraints = []
    eq_grad_types = []
    return RelaxedIK(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types)
end

function solve(relaxedIK, goal_positions, goal_quats; prev_state = [])
    if relaxedIK.relaxedIK_vars.position_mode == "relative"
        gp = []
        for i = 1:length(relaxedIK.relaxedIK_vars.robot.num_chains)
            push!(gp, SVector(relaxedIK.relaxedIK_vars.init_ee_positions[i]) + goal_positions[i])
        end
        relaxedIK.relaxedIK_vars.goal_positions = gp
    else
        relaxedIK.relaxedIK_vars.goal_positions = goal_positions
    end

    if relaxedIK.relaxedIK_vars.rotation_mode == "relative"
        gq = []
        for i = 1:length(relaxedIK.relaxedIK_vars.robot.num_chains)
            push!(gq, goal_quats[i] * relaxedIK.relaxedIK_vars.init_ee_quats[i])
        end
        relaxedIK.relaxedIK_vars.goal_quats = gq
    else
        relaxedIK.relaxedIK_vars.goal_quats = goal_quats
    end

    xopt = groove_solve(relaxedIK.groove, prev_state=prev_state)
    update_relaxedIK_vars!(relaxedIK.relaxedIK_vars, xopt)

    return xopt
end