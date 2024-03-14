## COMP0129 Robotic Sensing, Manipulation and Interaction Coursework 1 Team 6 



## Authors: 

Yujie Wang (ucab211@ucl.ac.uk)

Yufen Wu (yufeng.wu.22@ucl.ac.uk)

Xingyu Chen(xingyu.chen.23@ucl.ac.uk)



## How to Build and Run the Package:

To build the package, run:
```console
> catkin build
```

To run the package, type the following code in the terminal:
```console
> source devel/setup.bash
> roslaunch cw1_team_6 run_solution.launch 
```

The call the services for each tasks, open a new terminal and source:

```console
> source devel/setup.bash
```

In the same terminal, type one of the following code to start a task service:

```console
> rosservice call /task 1
> rosservice call /task 2
> rosservice call /task 3
```



## Tasks Informations:

#### Total time: 80 hours

#### Task 1: MoveIt! - Pick and Place at given positions [3 hours]

Contributions: Yujie 33%, Yufen 33%, Xingyu 33%

```console
> rosservice call /task 1
```

#### Task 2: PCL - Object Detection & Colour Identification [22 hours]

Contributions: Yujie 33%, Yufen 33%, Xingyu 33%

```console
> rosservice call /task 2
```

The result of Task 2 is shown in the ROS console, in the form of: 

```console
[ INFO] [1707440706.445080405, 29.986000000]: The Basket colors: ["none", "red", "blue", "red"]
```

#### Task 3: Planning and Execution [55 hours]

Contributions: Yujie 33%, Yufen 33%, Xingyu 33%

```console
> rosservice call /task 3
```



## License:

LICENSE: MIT. 

DISCLAIMER:

THIS INFORMATION AND/OR SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS INFORMATION AND/OR
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright (C) 2019-2024 Dimitrios Kanoulas except where specified

 
