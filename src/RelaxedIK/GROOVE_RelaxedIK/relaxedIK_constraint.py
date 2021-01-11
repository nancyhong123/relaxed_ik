from ..GROOVE.GROOVE_Utils.constraint import Constraint
from ..GROOVE.GROOVE_Utils.objective import get_groove_global_vars
# import numpy as np
# from pyquaternion import Quaternion
# from ..Utils import tf_fast as Tf

class Singularity_Avoidance_Constraint(Constraint):
    def __init__(self, *args): pass
    def constraintType(self): return 'ineq'
    def name(self): return 'singularity_avoidance'
    def func(self, x, *args):
        vars = get_groove_global_vars()
        mean = vars.yoshikawa_mean
        std = vars.yoshikawa_std
        min = mean - 2.6*std

        yoshikawa_score = vars.arm.getYoshikawaMeasure(x)

        return [yoshikawa_score - min]


class Joint_Velocity_Constraint(Constraint):
    def __init__(self, *args):
        self.joint_idx = args[0]
        self.velocity_scale = args[1]
    def constraintType(self): return 'ineq'
    def name(self): return 'joint_velocity_constraint_{}'.format(self.joint_idx)
    def func(self, x, *args):
        vars = get_groove_global_vars()
        avg = vars.avg_solution_time
        # avg = 0.02
        # print avg
        diff = abs(x[self.joint_idx] - vars.xopt[self.joint_idx])
        vel_limit = vars.robot.velocity_limits[self.joint_idx] * self.velocity_scale
        vel_limit_per_update = vel_limit * avg
        return vel_limit_per_update - diff
        # return .1*x[0]

# class Face_Target_Constraint(Constraint):
#     def __init__(self, *args): pass
#     def constraintType(self): return 'ineq'
#     def name(self): return 'face_target'
#     def func(self, x, *args):
#         vars = get_groove_global_vars()
#         for i, f in enumerate(vars.frames):
#             positions = f[0]
#             eePos = positions[-1]
#             goal_pos = vars.goal_positions[i]

#             z_axis = np.array([0, 1, 0])

#             eeMat = f[1][-1]
        
#             new_mat = np.zeros((4, 4))
#             new_mat[0:3, 0:3] = eeMat
#             new_mat[3, 3] = 1
            
#             ee_quat = Tf.quaternion_from_matrix(new_mat)
#             # ee_quat = Tf.quaternion_multiply(ee_quat,vars.init_ee_quats[i])

#             ee_rot = Quaternion(ee_quat[0],ee_quat[1],ee_quat[2],ee_quat[3])

#             ee_orientation_vect = ee_rot.rotate(z_axis)

#             ee_orientation_vect = ee_orientation_vect/np.linalg.norm(ee_orientation_vect)

#             ee_to_target_vect = np.array(goal_pos) - np.array(eePos)


#         return np.dot(ee_to_target_vect,ee_orientation_vect)




