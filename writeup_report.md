# **PID Control** 
---

**CarND-PID-Control-Project**

The goals / steps of this project are the following:
* Assign random PID coefficients and calculate steering error using simulator
* Manually tune PID coefficients based on aggregated error calculated using simulator
* Use twiddle method to further fine-tune PID coefficients keeping aggregate error the lowest
* Test that with final PID coefficients the car drives around track in the simulator without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[video1]: ./output_videos/pid_only_proportional.mp4  "Output video with autonomous driving using only proportional pid"
[video2]: ./output_videos/pid_only_integral.mp4  "Output video with autonomous driving using only integral pid"
[video3]: ./output_videos/pid_only_derivative.mp4  "Output video with autonomous driving using only derivative pid"
[video4]: ./output_videos/pid_tuned_manually.mp4  "Output video with autonomous driving using pid tuned manually"
[video5]: ./output_videos/pid_tuned_twiddle.mp4  "Output video with autonomous driving using pid tuned by twiddle"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1972/view) individually and describe how I addressed each point in my implementation.  

---
### Compilation

#### 1. Your code should compile

Code compiled without errors with cmake and make.

No change was made in CMakeLists.txt and thus the code is expected to compile on any platform.

### Implementation

#### 1. The PID procedure follows what was taught in the lessons

Random PID coefficients were assigned (main.cpp lines 63-65) to start with to calculate steering error using simulator. Thereafter 
with PID coefficients were maually tuned based on aggregated error calculated using simulator. Finally twiddle method (PID.cpp lines 76-121). was used to further fine-tune PID coefficients keeping aggregate error the lowest.

### Reflection

#### 1. Describe the effect each of the P, I, D components had in your implementation

Term P is proportional to the current value of the error (cross track error or proportional error) which is difference between expected positon of the car and the current position of the car. When there is large error, a large magnitude of steering output is expected. When there is no error, zero steering output is expected. With just the proportional component tuned, and rest of the components being set to zero, at the beginning, when the car was starting from center of the road on the track, there was close to no error, thus very little steering output/correction was given. At turns on the track, there was large error, thus large steering output/correction was given. But the 
correction was always delayed causing the car to oscillate right and left erratically at turns and thereby going outside the track quite soon.

`pid_only_proportional.mp4` captured the output as driving the car in autonomous mode with only proportional component set. Here's a ![link to my video result - pid_only_proportional.mp4][video1]

Term I accumulates past values of the error (cross track error) and provides sum or integral error. On application of proportional component, there is often some residue error which isn't compensated completely by proportional error. The integral error is the sum of all those cross track error which seeks to eliminate residue error. Thus, when diminishing of proportional effect is decreased with decrease in error, the effect of integral error grows into effect and together helps to minimize steering oscillations. With only the integral component available, past error got accumulated at every instance. The corrections made weren't strong enough to compensate for proportional loss and the car went out of track.

`pid_only_integral.mp4` captured the output as driving the car in autonomous mode with only propportional component set. Here's a ![link to my video result - pid_only_integral.mp4][video2]

Term D denotes future trend of the error (cross track error) based on current rate of change (derivative) of error. It assists to reduce the effects of proportional error by providing damping effect. If the error change is high, then high oscillations of steering is expected. With this derivative term, it intends to dampen the oscillations with the rate of change of error. With only the derivative component available, the corrections made weren't strong enough to compensate for proportional loss and the car went out of track.

`pid_only_derivative.mp4` captured the output as driving the car in autonomous mode with only propportional component set. Here's a ![link to my video result - pid_only_derivative.mp4][video3]

#### 2. Describe how the final hyperparameters were chosen
Student discusses how they chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination!

To begin with, assigned random PID coefficients and calculate steering error using simulator. Then, manually tune PID coefficients based on aggregated error calculated using simulator. The PID coefficients were changed first to ensure the car completes a significant portion on the track even if the drive was erratic and filled with oscillations. Then, once the car looks like able to complete a loop, I used twiddle method to further fine-tune PID coefficients keeping aggregate error the lowest for one complete loop of the track (approx 2500 counts). At the end, I tested that with final PID coefficients and the car drives around track in the simulator without leaving the road and with less osciallatory behavior.

`pid_tuned_manually.mp4` captured the output as driving the car in autonomous mode with only propportional component set. Here's a ![link to my video result - pid_tuned_manually.mp4][video4]

`pid_tuned_twiddle.mp4` captured the output as driving the car in autonomous mode with only propportional component set. Here's a ![link to my video result - pid_tuned_twiddle.mp4][video5]

The hyperparameters for different cases were,

| Method         		|          P          |          I          |          D          |
|:-----------------:|:-------------------:|:-------------------:|:-------------------:| 
| proportional     	|       -0.09         |         0.0         |         0.0         |
| integral     	    |        0.0          |       -0.0001       |         0.0         |
| derivative     	  |        0.0          |         0.0         |        -1.80        |
| manually         	|       -0.09         |       -0.0001       |        -1.80        |
| twiddle     	    |       -0.2504       |       -0.0001       |        -1.822       |

### Simulation

#### 1. The vehicle must successfully drive a lap around the track

At the end, I tested that with final PID coefficients (one with manual tuning and the other with fine-tuning using twiddle) and the car drives around track in the simulator without leaving the road and with less osciallatory behavior in both the cases.

