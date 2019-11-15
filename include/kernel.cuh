#pragma once
#include <stdio.h>
#include <cuda_runtime.h>
#include <cuda.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <time.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <vector>
#include "device_launch_parameters.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <time.h>
#include "dev_array.h"
#include <curand.h>
#include <cuda_profiler_api.h>

int Jobs2GPU(double x,double y,double yaw,double tire_angle);
int *functiongpu(int num);
