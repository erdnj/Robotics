# Robotics
A repository to store robotic field try and learn files.

# Autonomous Transpalet Navigation in a Virtual Factory Environment Project


> The main goal of that project is finding a path in given map and moving the transpalet without crashing or stucking in the environment. The palets location and goal points has known. In that project, all the orientation and location informations has also been known.
> While preparing the project, some difficulties and problems have encoutered. In that article, some of them will be discussed and some of them solved with some tricks.


</br>


### Some discussions about the start point

* At start, handling all of the things could b seem hard to begginers just like me. Dividing tasks into more smaller ones and focusing them could bring project to success just other things in life.
* As start point I tried to find path, not so sophisticated one just find a way. The main goal of that job is to moving transpalet on that path. If transpalet could move through the path correctly, drawing the path succesfully will handle our moving part.
* After we handle the navigation, we could move further which is the adjusting position of the transpalet related the target palet and goal location. First achivement is the taking the palet successfully. Then moving the palet to goal location without unnecessary movement. 
* Finally, as there will be moving objects, we could consider them to moving our robot.
* These are the little jobs to illustrate on further headlines.

</br>

### Step 1: Moving Transpalet On Path

The path tracking algorithm is simple and effective with true paramaters. In my algorith there is two parameters one for the keeping robot on line, the other one is to keeping the direction right on line. First one multiplied with robot location to line distance, second one multiplied with cross product of the line and robot directions. 
With try-fail method, I selected the parameters. With my parameters, my robot is more biased to keeping direction right. It is better in my case because the robot can change its direction without moving further, with that biased version, robot could succesfully turn sharp edges which keep the robot on line perfectly.

![](https://hackmd.io/_uploads/SyYpql-Dn.gif) </br>
Front Moving 

</br>

![](https://hackmd.io/_uploads/BymyogZD2.gif) </br>
Back Moving


As robot have capability to moving on two direction, I have taken advantage of that opportinity. Let say the front side is the palet taking side and the back side is the one which have moving wheel. While robot using is front direction, stucking through the walls is more possible while back side is more mobile at that case. But while joining the rooms through the doorhall back side has drawbacks if robot try to turn immadiately. 
The possible solutions are giving robot size higher than the usual to getting more safe paths from the path planner or checking robot environments before turnings. My aim was to keeping algorithm simple. There are states of the robot while handling jobs. Moving the robot on path has just one state. The turning or moving are all in that state and working together related to robot and line informations. So turning is not a special case in that algorithm, it is just usual as tracking path. So I prefered to getting more safer paths from path planner algorthm which is RRT*.

![](https://hackmd.io/_uploads/ryNtogbP2.gif) </br>
Consider the turning dynamics of front moving

</br>

![](https://hackmd.io/_uploads/S13XixWD2.gif) </br>
Although front moving has more mobile, back moving is much safer




### Step 2: Palet Taking & Dropping
After I achieve the path tracking, now it is time to taking transpalet from target point. After that robot will just track the path of goal point. Also in that section I will mention the palet dropping, which is also similar to taking palet. 

Robot use front side to go to taking palet job. Robot move to a point which is little behind of the palet location, after robot arrive that point, as it is arrived that point with front side, it adjust it orientation according to palet orientation. After that it just go some distance and stop. After it stop, we know that it is ready to take that palet, so we just elevate the forks of that transpalet. Now transpalet is weighted.

![](https://hackmd.io/_uploads/r1bHng-wh.gif) </br>
Watch the transpalet's taking palet action


If transpalet is weighted, it use back side path tracking. I prefered that just because the ease of exiting the place without some extra movements. The extra movements will be done at dropping area. As I mentioned earlier, the back side moving is more robust except the doorhalls, as we handled doorhall problem with getting more padded paths, we can discard that drawback.

As we taking the palet, we move little behind of the drop point. After arriving there, our robot adjusting its orientation. It was not easy like taking palet just because it nearly turning 180 degree. If it choose the wrong side, as you expect it could crach walls. At there, again I prefered simplicity and trusted my algorith as it prefered to choose side which need to turn less degree. For example let say palet drop is at right bottom corner of the room, robot joined the room from left side. Robot will arrive at point with orientation which looking right and little bottom, at that condition its turning movement will not crash the wall as it preffer the turning left. So crashing the wall is just a little possibility, so I discarded it. For more sophisticated implementations, it could be checked for turning side selections. After our robot adjust it orientation, it draw a path to droping palet, now robot just track that path to arrive drop point, the lower its fork to success delivery. As final it goes backward to assign new task.

![](https://hackmd.io/_uploads/SyrK2g-Pn.gif) </br>
Consider the transpalet's turning direction and adjusting capability


### Step 3: Handling Moving Objects

I know there are many sophisticated algorithm to handle moving objects. But as the bones of that algorithm is depend on line tracking, I tried to handle it in my way. 

To avoid moving objects, my robot check its center radius with 2-3 second intervals. If it detect any moving object in that radius it recalculate the target path with regarding moving object's location. The recalculating the path is hard work for robot but to keeping simplicity I prefered a simple solution. Which has done its job successfully even it wait me in simulation to re-calculate new path.


### Final Overview

As I mentioned the back moving has drawback which is stucking doorhall. I want to show that case. It could drop the palet or shift it on transpalet, in each case it cause problematic results and it should handled with mentioned solving methods.

![](https://hackmd.io/_uploads/HJQvTgbD3.gif) </br>
Palet completely droped from the transpalet sadly

</br>

![](https://hackmd.io/_uploads/Sy_FTxZPh.gif) </br>
Palet shifted slightly, still problematic case

</br>
