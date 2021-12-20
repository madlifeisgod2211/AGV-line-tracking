# AGV-line-tracking
DESIGN CONTROL ALGORITHM FOR AGV LINE TRACKING ROBOT BY USING FUZZY LOGIC CONTROLLER

Author: Nguy Minh Tai
Date: 20/12/2021

This code illustrates how to design Fuzzy controller to generate 3-wheel mobile robot to track the available line. 
The target sets are described as follows:
 - Desired velocity: 1,5 m/s
 - Desired error: +- 10 mm
 - Completing time: 10 seconds

System parameters:
  1. Fuzzy type: Mamdani
  2. Input: e and de
  3. Output: wl, wr - omega of left and right motors
  4. Defuzzification: Center of Average (COA)
  5. Number of rules: 15
