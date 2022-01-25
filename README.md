# odometry_challenge

Odometry of a Differential Drive Robot

This application will calculate the odometry of a differenital drive robot given: left and right wheel encoder readings, robot initial position, robot width, and wheel radius.

MTRE 6100, Odometry Challenge
(Friday, September/10/2021) 
 
Please use Python to create a program that calculates the odometry of a different drive 
robot based on the measurements of its encoder sensors. To understand the concepts and 
process of calculating odometry, please review the section “odometry of a differential 
drive robot” on page 58~ page 71 of the lecture slides: “MTRE 6100: Sensors and 
Programming”.  
 
Problem setting: 
• The width of the robot: 2L = 1 meter 
• The radius of the wheel: 𝑟 = 0.1 meter 
We assume the encoder measurement is given at 2 Hz. That is, two measurements will be 
generated per second (should be higher in a real encoder).  The following is the output 
angular velocity for two encoders in 10 seconds: 
• Left encoder output 𝜔!= [2, 2, 2.5, 2.5, 3, 3, 3.5, 3.5, 5, 5, 5 ,5, 3.5, 3.5, 3, 3, 2, 2, 
2,2].  
• Left encoder output 𝜔"= [2, 2, 2, 2, 2.5, 2.5, 3.5, 3.5, 5, 5, 5 ,5, 4.5, 4.5, 3.5, 3.5, 
2.5, 2.5, 2, 2]. 
• Angular velocity is in rad/s. 
We assume the robot start position is 𝑃#= (0,0,0).  
 
 
Expect result of your program:  
1. Output the speed of the wheels (𝜗! for left wheel, 𝜗" for right wheel) based on the 
encoder outputs, the speed of both wheels should synchronize with the output of 
the encoders. The equation is on page 58: 𝜗=𝜔∙𝑟. 
 
2. Calculate the move distances of two wheels: Δ𝑠!  and Δ𝑠" . The encoder's 
output at 2 Hz, which is 2 measurements per second. Hence, the Δ𝑡 between 
two successive measurements is Δ𝑡 =0.5 second. We assume the angular 
velocity in Δ𝑡 will not change.  For example, the left encoder indicates𝜔! at the 
time 𝑡$= 0.5 s is 2 rad/s, and 𝑡%= 1s is 2.5 rad/s, thus, the speed between 𝑡$ and 𝑡%  
is the 2 rad/s. (Equation Δ𝑠!= θ! Δ𝑡 ). 
 
3. Calculate the positions (𝑃$~𝑃$&) of the robot. 
 
Your program should output the above 3 results in a step-by-step way. For example, 
you should output 𝜗!, 𝜗" , Δ𝑠!, Δ𝑠", and 𝑃$ for the first step.  
 

