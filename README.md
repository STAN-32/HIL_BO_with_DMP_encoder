# HIL_BO_with_DMP_encoder

## DMP_encoder
This folder provides examples of trajectory encoding using Dynamic Movement Primitives (DMP).

The encoding results can be generated and saved in compare.png by running the following file.
```
plot_simu.m
```

## HIL_BO
This folder contains the ROS package for implementing the Bayesian algorithm for human-in-the-loop optimization, which can be run with the following command after compilation.
```
roslaunch bayes_op bo.launch 
```
