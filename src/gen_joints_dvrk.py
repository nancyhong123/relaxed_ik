#! /usr/bin/env python
import numpy as np
import random as rand


if __name__ == '__main__':
    joint_limits = np.array([[-1.57,1.57],[-0.79,0.79],[0.1,0.24],[-4,4],[-1.57,1.57],[-1.57, 1.57]])
    num_joints = 6
    num_samples = 300

    

    generated_samples = np.zeros((num_samples,6))
    for i in range(num_samples):
        for j in range(num_joints):
            generated_samples[i,j] = rand.randrange(joint_limits[j,0]*1000000., joint_limits[j,1]*1000000.)/1000000.

    
    print(generated_samples.tolist())
        
    

   

