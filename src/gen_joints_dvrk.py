#! /usr/bin/env python
import numpy as np
import random as rand


if __name__ == '__main__':
    # joint_limits = np.array([[-1.57,1.57],[-0.79,0.79],[0.1,0.24],[-4,4],[-1.57,1.57],[-1.57, 1.57]])
    # num_joints = 6
    # num_samples = 300
    # generated_samples = np.zeros((num_samples,6))
    
    collision_free_state_deg = np.array([[ 0.00,-11.37,0.150, 0.0,0.0,0.0],[ 30.0,-11.37,0.150, 0.0,0.0,0.0],[ 30.0,-11.37,0.200, 0.0,0.0,0.0],[15.0,-11.37,0.200, 0.0,0.0,0.0], [10.0,-11.37,0.200, 0.0,0.0,0.0], [10.0,20.0,0.200, 0.0,0.0,0.0],[-10.0,20.0,0.200, 0.0,0.0,0.0],[-8.56,9.03,160.00,0.10500,-34.97,69.24],[0,9.03,0.160.00,105.00,-34.97,69.24],[-10.00,-10.0,0.16000,105.00,-34.97,69.24]])
    
    generated_samples = np.radians(collision_free_state_deg)
    generated_samples[:,2] = collision_free_state_deg[:,2] 

    
    # for i in range(num_samples):
    #     for j in range(num_joints):
    #         generated_samples[i,j] = rand.randrange(joint_limits[j,0]*1000000., joint_limits[j,1]*1000000.)/1000000.


    
    
    print(generated_samples.tolist())
        
    

   

