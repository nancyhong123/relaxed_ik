using LinearAlgebra
using StaticArrays
using Rotations
include("../Utils_Julia/transformations.jl")
include("../Utils_Julia/geometry_utils.jl")
include("../Utils_Julia/nn_utils.jl")

function lookat_obj_1(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    eeMat = vars.robot.arms[2].out_frames[end]
    camera_pt = vars.robot.arms[2].out_pts[end]
    manipulation_pt = vars.robot.arms[1].out_pts[end]

    forward = -eeMat[:,3]
    # forward = eeMat[:,2]

    x_val = pt_dis_to_line_seg(manipulation_pt, camera_pt, camera_pt + 100.0*forward)
    return groove_loss(x_val, 0.,2.,.1,10.,2.)
end

function lookat_obj_2(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    eeMat = vars.robot.arms[1].out_frames[end]
    camera_pt = vars.robot.arms[1].out_pts[end]
    manipulation_pt = vars.robot.arms[2].out_pts[end]

    forward = -eeMat[:,3]
    # forward = eeMat[:,2]

    x_val = pt_dis_to_line_seg(manipulation_pt, camera_pt, camera_pt + 100.0*forward)
    return groove_loss(x_val, 0.,2.,.1,10.,2.)
end

function camera_dis_obj_1(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    camera_pt = vars.robot.arms[2].out_pts[end]
    manipulation_pt = vars.robot.arms[1].out_pts[end]

    x_val = (LinearAlgebra.norm(camera_pt - manipulation_pt) - vars.additional_vars.goal_distance_to_target)^2
    return groove_loss(x_val, 0.,2.,.5,10.,2.)
end


function camera_dis_obj_2(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    camera_pt = vars.robot.arms[1].out_pts[end]
    manipulation_pt = vars.robot.arms[2].out_pts[end]

    x_val = (LinearAlgebra.norm(camera_pt - manipulation_pt) - vars.additional_vars.goal_distance_to_target)^2
    return groove_loss(x_val, 0.,2.,.5,10.,2.)
end

function camera_upright_obj_1(x, vars)
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    eeMat = vars.robot.arms[2].out_frames[end]
    side = eeMat[:,2]
    # side = eeMat[:,1]

    x_val = LinearAlgebra.dot(side, [0.,0.,1.])^2
    return groove_loss(x_val, 0.,2.,.1,10.,2.)
end

function camera_upright_obj_2(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])

    eeMat = vars.robot.arms[1].out_frames[end]
    side = eeMat[:,2]
    # side = eeMat[:,1]

    x_val = LinearAlgebra.dot(side, [0.,0.,1.])^2
    return groove_loss(x_val, 0.,2.,.1,10.,2.)
end


function camera_occlusion_avoid_obj_1(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    eeMat = vars.robot.arms[2].out_frames[end]
    camera_pt = vars.robot.arms[2].out_pts[end]
    forward = -eeMat[:,3]
    # forward = eeMat[:,2]

    x_val = 0.0
    out_pts = vars.robot.arms[1].out_pts
    num_pts = length(out_pts)
    for i = 1:num_pts-1
        dis = dis_between_line_segments(camera_pt, camera_pt + 100.0*forward, out_pts[i], out_pts[i+1])
        c = 0.07
        x_val += (2.718281828459^((-(dis)^2) / (2.0 * c^2.0)) )
    end

    # x_val = 0.0
    # return groove_loss(x_val, 0.,2.,.35, .4,2.)
    return groove_loss(x_val, 0.,2.,.8, .4,2.)
end

function camera_occlusion_avoid_obj_2(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    eeMat = vars.robot.arms[1].out_frames[end]
    camera_pt = vars.robot.arms[1].out_pts[end]
    forward = -eeMat[:,3]
    # forward = eeMat[:,2]

    x_val = 0.0
    out_pts = vars.robot.arms[2].out_pts
    num_pts = length(out_pts)
    for i = 1:num_pts-1
        dis = dis_between_line_segments(camera_pt, camera_pt + 100.0*forward, out_pts[i], out_pts[i+1])
        c = 0.06
        x_val += (2.718281828459^((-(dis)^2) / (2.0 * c^2.0)) )
    end

    # x_val = 0.0
    return groove_loss(x_val, 0.,2.,.8, .4,2.)
end

function avoid_environment_occlusions_obj_1(x, vars)
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    # eeMat = vars.robot.arms[2].out_frames[end]
    camera_pt = vars.robot.arms[2].out_pts[end]
    goal_position = vars.additional_vars.camera_goal_position

    x_val = LinearAlgebra.norm(goal_position - camera_pt)
    return groove_loss(x_val, 0.,2.,.1, 3.0,2.)
end



function gravitate_to_natural_position_obj1(x, vars)
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    camera_pt = vars.robot.arms[2].out_pts[end]
    natural_point = [0.0, 0.1, 1.0]
    x_val = LinearAlgebra.norm(natural_point - camera_pt)
    return groove_loss(x_val, 0.,2.,.9, .1,2.)
end

function look_at_visual_target_obj1(x, vars)
    # vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    eeMat = vars.robot.arms[2].out_frames[end]
    camera_pt = vars.robot.arms[2].out_pts[end]
    # manipulation_pt = vars.robot.arms[1].out_pts[end]
    visual_target = vars.additional_vars.visual_target_position

    forward = -eeMat[:,3]
    # forward = eeMat[:,2]

    x_val = pt_dis_to_line_seg(visual_target, camera_pt, camera_pt + 100.0*forward)
    return groove_loss(x_val, 0.,2.,.1,10.,2.)
end


function look_at_visual_target_obj2(x, vars)
    # vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])

    eeMat = vars.robot.arms[1].out_frames[end]
    camera_pt = vars.robot.arms[1].out_pts[end]
    # manipulation_pt = vars.robot.arms[1].out_pts[end]
    visual_target = vars.additional_vars.visual_target_position

    forward = -eeMat[:,3]
    # forward = eeMat[:,2]

    x_val = pt_dis_to_line_seg(visual_target, camera_pt, camera_pt + 100.0*forward)
    return groove_loss(x_val, 0.,2.,.1,10.,2.)
end
