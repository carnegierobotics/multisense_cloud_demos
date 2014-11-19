# MultiSense Cloud Demos

A set of demo software showing usage of Carnegie Robotic's MultiSense stereo products including the S7 and S21.

Note that these software packages generally assume operation with the 2 MP version of CRL's stereo systems and will need some tweaking to work with 4 MP imagers.


### Building

You will need to install a few prerequisites:

    > sudo apt-get install build-essential cmake freeglut3-dev libcv-dev libopencv-contrib-dev libopencv-photo-dev

Then build:

    > mkdir build
    > cd build
    > cmake .. -DCMAKE_BUILD_TYPE=Release
    > make

Output files will be copied to ./bin


### Running


For best performance, plug the MultiSense unit directly into a Gigabit Ethernet
port on your machine.  See the ConfigureNetwork.sh script for an example of how
to configure the network for good performance.  The default MultiSense IP
address is 10.66.171.21. In order to restart the network-manager after you are
through running the demos, run:

sudo service network-manager restart


### Controls

All these demo programs share a set of simple keyboard commands.
Generally, a lower-case letter decreases a parameter, and the same upper-case letter increases it.
So, for example, 'g' reduces the gain while 'G' increases the gain.
There are also simple motion commands which allow you to fly around the scene.

Common demo commands:

**MOUSE**

* Left button:  Pitch and yaw view
* Scroll wheel: Fly forward and back
* Right button: Pan view

**KEYBOARD**

```
Move camera:     w = move forwards             r = move up
              a  &  d = move left and right
                 s = move backwards            f = move down

Tilt camera:     u = pitch up
              h  &  k = yaw left and right
                 j = pitch down

g,G = Adjust live camera gain
n,N = Adjust live camera noise (stereo post filtering)
p,P = Adjust 3D point display size
x,X = Adjust X clipping (removes left and right sides)
y,Y = Adjust Y clipping (removes the ceiling)
z,Z = Adjust Z clipping (removes distant objects)
m,M = Step through variety of color display modes
 ,  = Toggle overlay of range and color data in same display
 .  = Change which axis (Z or Y) is used for color range display
+,- = Brighten/darken on-screen display
 q  = Quit program
```

You can try adjusting the gain and the noise parameters to try to fill in some of your missing/black point cloud areas.

### License

Copyright 2014
Carnegie Robotics, LLC
4501 Hatfield Street, Pittsburgh, PA 15201
http://www.carnegierobotics.com

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of the Carnegie Robotics, LLC nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.