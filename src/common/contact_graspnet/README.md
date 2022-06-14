Grasp generation has been tested with python 3.6, tensorflow 2.6.2, CUDA 11.2, NVIDIA-SMI 470.103.01

Object detection and segmentation has been tested with python 3.6, torch 1.9, CUDA 11.2, NVIDIA-SMI 470.103.01

This package has to be built in a catkin workspace configured for python3

Launch file will launch three servers:
        
    graspnet_detection.py will detect and segment object from the dataset.
    
    generate_grasps_server_local.py will generate grasps using contact graspnet NN

**Note**: Torch currently defaults to run inference using CPU, and tensorflow uses GPU. This can be changed in the files:

    utils/mask_rcnn.py
    utils/ycb_mask_rcnn.py
    generate_grasps_server_local.py

**Dependencies**:

    Download mask-RCNN/robocup.weights and place the weights under contact_graspnet/src/contact_graspnet/utils directory.
    
    Download contact-graspnet/checkpoints.tar, **tar -xf** it and place it the root of this package (contact_graspnet/).

   
**Compiling PointNet2**

    PointNet2 needs to be compiled in order to run contact_graspnet.
    All you need to do is run: **sh compile_pointnet_tfops.sh**. You may have to do **chmod +x compile_pointnet_tfops.sh** beforehand.