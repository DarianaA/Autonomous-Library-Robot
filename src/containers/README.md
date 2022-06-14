## Installing singularity

#### Prerequisites

You need to install `wget, git and sudo` in your host as follows:

```
sudo apt update && sudo apt install sudo wget git
```

#### Install singularity

From **outside of the repository**, run:

```
source /containers/install_singularity.sh
```

This will install singularity in the host computer. The script may take a while to finish.



## Building the container

Once singularity has install normally (no errors), we are ready to build the container. The following command will install the container in your home folder. Do not put the container inside the repository (to prevent pushing it by mistake). To build the container, run:

```
sudo singularity build $HOME/tiago.sif tiago.def 
```

This will take a long while. The process may freeze the system at some points (probably in the  `INFO:    Creating SIF file...` step), just let it be and it should finish. It is better if you run this while your computer has a low load (so not a lot of programs running and as much free RAM as possible).

## Building the container with CUDA 11.2, CUDNN 8.1, tensorflow-gpu==2.6.2 and pytorch==1.9.0

```
sudo singularity build $HOME/tiago_cuda112.sif tiago_cuda.def
```

## Running the container

In order to launch a terminal inside the container, you can run the following (graphic card info at the end):

```
singularity run $HOME/tiago.sif
```

It all went well if after running this you see:

```
I am in Singularity!
Singularity>
```

### Graphic cards

If you have a graphic card, the command will be different in order to run gazebo and rviz. If you have an **nvidia** graphic card, run singularity as:

```
singularity run --nv $HOME/tiago.sif
```

If you have an **AMD** card, run it as:

```
singularity run --rocm $HOME/tiago.sif
```

### Writable containers

It will be useful to some individuals to have access to writable containers whilst developing - particularly when adding and testing new dependencies.
In order to achieve this, you need to build with a writable-tmpfs or sandbox - I think sandbox is more suitable here.

```
sudo singularity build --sandbox $HOME/tiago-rw.sif tiago.def
```
This will build the container in a writable sandbox.

In order to write to the container after it is built (i.e. install some python package):

```
singularity run --writable $HOME/tiago-rw.sif tiago.def
```
Note that you **cannot** combine the `--writable` and `--nv` flags. You may also have to run as `sudo`.

Install a dependancy when running as writable:

```
Singularity> pip3 install pip_install_test
```

Exit the writable container and test persistence (here we can use --nv flag again):

```
singularity run --nv $HOME/tiago-rw.sif tiago.def
Singularity> python3 -c 'import pip_install_test'
```

You should see output from the in-line Python verifying that the package was correctly installed.



### Dealing with storage issues

It's possible you may get errors while building, particularly the CUDA version.
You can solve this as such:

```
truncate -s 35G tmpfs
mkfs -t ext4 tmpfs
sudo mount -o loop tmpfs /var/cache
export SINGULARITY_CACHEDIR=/var/cache/
sudo -E singularity build --tmpdir $SINGULARITY_CACHEDIR $HOME/tiago.sif tiago.def
```
