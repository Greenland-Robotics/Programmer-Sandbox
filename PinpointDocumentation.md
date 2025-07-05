The goBILDA Pinpoint Odometry Computer is a special device that helps your robot know exactly where it is on a playing field. Think of it like a super-accurate GPS for your robot. It's mainly used in robotics competitions like First Tech Challenge (FTC).

**What it does in simple terms:**

Your robot moves around, and the Pinpoint helps it keep track of its position (like its X and Y coordinates on a map) and which way it's facing. It does this by using two small wheels, called "odometry pods," that roll on the ground and an internal sensor that tells it about its rotation.

**Why it's good:**

*   **Super Accurate:** It's designed to be extremely precise, updating your robot's position very, very quickly (1500 times per second!). This means your robot can move fast and still know exactly where it is, which is important for competition.
*   **Easy to Use:** If you use goBILDA's own odometry pods, a lot of the tricky setup is already done for you.
*   **Smart Combination:** It combines information from the odometry pods and its internal rotation sensor to give you the most accurate location possible.

**How to set it up and use it (simplified):**

1.  **Mount the Pinpoint:** Place the Pinpoint device on your robot with the sticker facing up.
2.  **Attach the Odometry Pods:** You'll need two odometry pods.
    *   One pod should be mounted so it tracks how far your robot moves *forward and backward* (this is often called the "X pod").
    *   The other pod should be mounted so it tracks how far your robot moves *side to side* (this is often called the "Y pod").
3.  **Connect and Power Up:** Connect the Pinpoint to your robot's main controller using an I²C cable. Then, turn on your robot's power.
4.  **Tell it about your pods:**
    *   **Encoder Direction:** This is very important! You need to tell the Pinpoint which way your odometry pods are spinning.
        *   First, run the Odo Tuner opMode
        *   When your robot moves straight forward, the "X pod" numbers should increase. If they decrease, you need to tell the Pinpoint to reverse that direction.
        *   When your robot moves straight to the left, the "Y pod" numbers should increase. If they decrease, you need to tell the Pinpoint to reverse that direction.
    *   **Pod Offsets:** You can also tell the Pinpoint exactly where on your robot the odometry pods are located. This helps it be even more accurate about your robot's central position.
        You want to be exact with this. You can test this by runing the Odo Tuner opMode and spinning the robot in a circle around its center, making sure you don't move the center of the robot. After making a full 360 turn, the x and y values of the robot should be about 0. If they are far away from 0(>30), remeasure the center to your pods.
      
5.  **Check the Status Light:** The Pinpoint has a light that tells you what it's doing. Green usually means everything is working correctly. Different colors can indicate issues like a pod not being plugged in.
6.  **Programming:**
    *   The Pinpoint sends your robot's position and heading information to your robot's main controller.
    *  To access these readings, use the `getX()`, `getY()`, and `getAngle()` methods in your opMode. You typically will only use this for telemetry, as all pathing logic is handled for you
  

In short, the goBILDA Pinpoint is a smart device that, with a bit of physical setup and some simple programming instructions, helps your robot know its exact location and direction on the field, making it much better at navigating.
