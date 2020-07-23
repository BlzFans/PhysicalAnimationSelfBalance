# Physical Animation Self Balance System for Game Character
## RESULTS
![SBS0](/Imgs/SelfBalanceGif_0.gif)
![SBS1](/Imgs/SelfBalanceGif_1.gif)
![SBS2](/Imgs/SelfBalanceGif_2.gif)
![SBS3](/Imgs/SelfBalanceGif_3.gif)
![SBS4](/Imgs/SelfBalanceGif_4.gif)  
## **[Video on Reddit]()**
---
## INTRODUCTION
We want game character that **physical based and fully interactable** since the day game been invented, imagine a game where you can hit your enemy exactly where you want hit, and enemy will responds naturally just like what you expected in real life, they both not just some prefab animation like in most of today's 3A games where every time you press the same button, the game will play same animation for the character you control and enemy you hit over and over again. 

More specifically lets say you want your character use the sword sweep the enemy on stomach but enemy block it down with its weapon, so your character end up hit the enemy's knee, but because hit so heavy so it knock that leg of the enemy off the ground, then the enemy will try to keep the balance, ***as natural as one expected***.

Those kind of complex natural actions we can see on movie, CG, or real life, but not in today's real-time gameplay. in most of today's game, your character's sword will go right through the enemy's weapon and body, and enemy will play the same animation or same set of animations every time been hit, ***as mechanical as it sounds***.

## RELATED WORKS
The most straightforward approach to achive animation with physics is the **Physical Animation** system in most of today's game engin, they constrain the root body (e.g hip for humanoid) to follow the animation's transform precisely, and use the [PD Controller](https://www.matthewpeterkelly.com/tutorials/pdControl/index.html) to set force on ecah joint to make sure rest of body follow the animation approximately. But this approach is very problematic.

1. The root body its not really physical based, which means whenever somethings contact it, the root body will not have any responds, it will still follow the root body's animation exactly, one need use code to set the character's root transform procedurally for fake the phycial effect; 

2. When environment setting have gravity (which usually does), after character been hit it will naturally seek to balance, i.e usually involved complex motions, but physical animation will simply take the body straight back to where animation should be, this maybe good in some of the situations like error between current and target transform is small, but its terrible in lot of other situations like when leg been hit off the ground; so in order to use the physical animation to achive the not very bad result, you have to fake it with quite lot of code.

The one looking promising approach try to solve the character balance problem is used in game like [this](https://www.reddit.com/r/Unity3D/comments/hu3c8v/now_active_ragdoll_enemies_can_go_on_patrol/) and [MetallCore](https://www.reddit.com/user/MetallCore999/), instead of constrain the root body to follow the animation, when character try to keep balance we abandon the prefab animation at all make it fully physical based, and try to set the character's joint target transform according to our knowledge about character gait (e.g human walking gait in real life) both procedurally and manually, then use PD Controller to follow it directly, finally maybe add some extra force on root body for help balance according to character state, e.g the feets relative position with center of mass, etc. This approach also have some problems.

1. Its very difficult to set the joint target transform to even just roughly follow the gait we want, you need lot of tuning on parameters for each joint in order to achive some not bad result, with help force on root body its very hard for tuning parameters to make it looks believable, but without help force on root body usually make tuning phase extremely hard even just for keep the character on its feet.

2. Since the character is fully physical based then it means its very hard to make the body other than legs (e.g arm) to follow the animation even approximately, like for character reach hands to the target position while the character try to keep the balance, especially without help force on root body, one may address this by switch between physical animation and self balance, but again, it require quite of work for good result.

And one interesting approach (We often see today's walking robot have similar approach) has been post before by [DeepMotion](https://www.youtube.com/watch?v=Hk_7wKN48R4) (this work is little different and not as good as we discussed here, cause it constraint rotation of the root body and may have fake the root body's movement) and resently in [Wilnyl](https://www.reddit.com/user/Wilnyl/)'s [robot game](https://www.reddit.com/r/GamePhysics/comments/eojjlw/untitled_robot_game_making_a_game_with_a_bunch_of/) one of the developer of popular game [TABS](https://www.epicgames.com/store/en-US/product/totally-accurate-battle-simulator/home), instead of make character fully physical based, it make the body other than legs physical based, and the legs is mvoing according to procedural animation. At each time step, there have mechanism to **planning** things like need move leg or not, which leg to move, target position of feet, trajectory of feet, etc, and use the IK (Inverse kinematics) to moving feet according to that trajectory, finally apply extra force on root body according to character state e.g the feets relative position with center of mass, etc, this will create the self balance animation looks very believable, but problem is very obvious.

1. The leg has no natural physics and not interactable whatsoever, you could use the static physic but that stil not interactable just like the root body of physical animation, wanna fake it? good luck with that.

2. Since the leg is drive by IK procedural animation, so if we want use original anmations, we need some extra work for it to tranform believable and smoothly with original animation.

## METHODS
I propose new methods that make character's self balance looks very believable and also fully physical based, it work well regardless the ground is flat or uneven, I will explain the basic idea behind it first, after that I will explain some crucial mechanism in detail, without those mechanism the idea will never work.

### Basic Process :
1. **Planning** :   
At each time step according to some information about character's state, e.g if the character are already balanced, if the feet is moving toward target, root horizontal plane velocity, distance between feet current location and target location, etc, we determine move leg or not and which leg to move; after we choose the feet to moving we need determine that feet's moving trajectory for it to follow in given time, at each time step we use some character's state information like root horizontal plane velocity, root body rotation, original feet moving end target position to calculate new feet moving end target position, then according to new feet moving end target position, feet gait curve (usually something like `y = -x^2 + 1, or Normal distribution), time of that feet alread moved, etc, to get current feet target position on feet moving trajectory.

2. **IK && PD Controller** :   
After we know where the feet should move to, we need using current feet target position, current target direction of knee, current target position of IK's root joint (e.g thigh) and use IK algorithm (in this work is simple Trigonometric IK) to calculate the current target local rotation for each joint on leg, now we know the target local rotation for each of the leg joint, we simply need using the PD Controller make joint move directly toward it, another information needed by PD Controller is joint's target angular velocity which could calculate out easily.

3. **Balance Force** :  
After the feet moving finished, we will apply extra force on root body according to character state e.g the feets relative position with center of mass, etc, in this work we have one balance detect mechanism for determine how much the state of character is clost to balance and set the balance force proportionally.

### Crucial Mechanism : 
* **Define the Balance** :   
When character at what state or pose we can say the character is balanced? In this work, we let user to pre-record the pose they want use in slef balance.  
System take the pose then calculate and store the center of mass's local data, for each leg get local feet position, local root joint position, the height different between feet and root joint, the knee or pole direction and some conversion matrix, the height different between feet and ground, etc, for center of mass, we use root body's position as its position and rotation use the character facing direction and world up direction to calculate, we don't use real center or mass because we want the leg's balance pose is irrelevant with other body part's transform, all those data will later use in system to calculate the joint's target world position.  
Since we can get balance pose's target world position of feet, so its natural for us to consider character is balanced when feet at that position, but because the character is physic drived everything could only be approximate they cannot moving to exact position, so we consider the character is balanced when feet is inside some range of balance pose's feet world position, we call it target area, when feet overlap the target area, we say the character is balanced (can be develop using feet collider enter the target trigger collider).

* **Feet Adjust By Ground** :  
In self balance mode at each time step, we need get information like ground normal and height under the feets (system in this work just simply use the one ray cast from feet joint straight down the ground), then we will use it to adjust feet rotation for better fit ground surface and determine the height of all leg's target root joint's position for keep root body always at the same height from ground surface, this is very important component especially when character is on the uneven ground.

* **Reference Root Follow && Limit** :   
Each physical based character have a 1:1 reference character (only have skeleton, not visible in game) for get the reference animation out of animation system (its the easiest way in unity game engine maybe you can get the animation data out of animation system directly in other game engine), and a character controller for control the reference character's movement and animation flow.  
The root body of character try to follow the transform of reference character's root body on condition of balance detect mechanism and PD Controller, but if one only move the feet and root toward target position will usually result some bad looking result, where the feet is barely touch the target area but the system is consider the character balanced even if the whole body of character is lean forward (mostly because of ground friction, inaccuracy and delay of PD Controller).
In order to address this problem, we using same balance detect mechanism to determine how much the reference character should moving back, so feets can fit target area better and so the balance motion can looks much more believable.  
The problem of make all target areas and they corresponding feets position have minimal distance, can be viewed as finding the position for each line segment so they corresponding endpoint have minimal distance, back moving mechanism used in this work is simply moving character controller to make center of all feets target positions move toward center of current all feets positions. 
Because we using PD Controller to add the extra force on root body, so its good idea for keep the different between root body and reference root body root's horizontal plane position and world up axis rotation in some range, so the PD Controller don't generate extreme large force to casue character doing undesirable action, if we only limit the force of PD Controller but not that transform difference, there sometime will have large gape between character and its reference, then character's will consitent move toward the reference even there not have any user input or force we can see in the scene, it gonna feel weird.
In real life, when feets that on ground only at one side of center of mass, then it will not only create upward support force, but also create some horizontal force that push the body aside, so we add some extra movement to character controller on condition of balance detect mechanism for create this effect as well.  
![Target Move Back](/Imgs/Target_Move_Back.png)

* **Leg Avoid** :   
When leg moving toward its target and other leg just happend on that path, then two leg will collide on each other , this is catastrophic for physical based character, we can't just simply disable the collision between legs cause that gonna look awful, so we need the way to make sure the leg know when and how to avoid the other leg that on they path.  
The method use in this work is designed for three joint two link leg (human like leg), we view the calf line as the line segment between knee and feet, when one leg is moving toward its target, we project all leg's calf line on the plane that normal is moving feet's target direction (from feet's start position to target position), then we calculate the vectors between each end point of other leg's projected calf line and its closest point on moving leg's projected calf line, and the vector from moving leg's projected calf line to other leg's projected calf line at reverse direction, then we compare the difference between those vectors and vertical direction which is direction determined at begining of each feet's gait where moving feet can move in vertical direction to avoid other leg, if all those vectors have same direction (Vector Dot > 0) with vertical direction and minimal distance in those vectors is bigger than some value, then we can say there is no other leg on moving leg's path (decline the vertical movement we add before so the leg is slowly back to its original path), otherwise we add some movement on vertical direction to avoid the other leg, this method is designed specifically for avoid leg, one could use the physic cast detection on moving leg instead, for avoid leg and other obstacle as well, of course there more than one feature we could add, we discuss it at final section.  
![Leg Avoid Leg](/Imgs/Leg_Avoid_Leg.png)

Of course, those are not all the detail in this work, e.g you could add the animation while balance for body's other than leg (e.g arm reach for balance, this doesn't effect the system itself much but will looks more natural), but if I do that would make this paper too long and tedious, if you interesting in this work you can check the [source of this work](https://github.com/MrForExample/PhysicalAnimationSelfBalance) and see for yourself, the code of this work is well annotated, anyone have basic knowledge on C# and Math (mainly vary basic Geometry and Linear algebra) should be able to understand it easily.

## DISCUSSION AND FUTURE WORK
Before this work, self balance for fully physical based character (especially the humanoid) is really feels like the problem can only be solved appropriately by Machine learning (ML), but this work shows just assemble some crucial mechanism togather, we can still get pretty good approximation of what we actually want in game.  
There lot of features can be add or improve in future, e.g:
* We could use physic cast detection and some clever mechanism for avoid different kind of leg and other obstacle.
* When avoid the obstacle we may also change IK leg's rotation by adjust its pole direction instead of only feet position (like human walking in side walk, moving feet will rotate to avoid the other feet).
* Better ground surface detect method could be easily add.
* The system in this work not specifically deal with the case when character is in air (two feet off the ground), one can easily add different effect by game needs; 
* Maybe in a game where ground is not consistent, then we need better gait plan mechanism to make sure the feet won't step in a hole.
* Mechanism for adjust feet mvoing trajectorys to be able to walking upstairs and so on.
* Some mechanism to make the character get up more believable after fall on the ground.
* When one feet of character been catched by enemy the character should know how to jump for balance. 
* Right now the system can only to balance to pre-record given pose, and it need switch between physical animation and self balance, we could build the system that balance to follow the real time animation directly, use IK && PD for leg all the time instead of switch to physical animation's direct PD Controller.
* According to game needs, etc. 

Still, the system in this work have dozens of parameters you need to adjust for good result (but we know exactly each one does and they basic range, so it shouldn't take long), so in future we could make some process in system more automatic or maybe relpace it by ML model, we already see this kind of work in today's state of the art robots gait algorithm (they use the same basic process but some more steps like they need depth map prediction of the ground form camera), so naturally, the future is all ML's, but until the ML model's training difficulty and time cost are significantly reduced or computer's calculate power significantly increased, we could already have lot of lovely characters bumping around naturally in our games.








