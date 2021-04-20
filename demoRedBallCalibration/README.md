# Automatic calibration for red ball demo :red_circle:

The *automatic calibration for red ball demo* includes two (independent) steps:
1. [color calibration](#color-calibration);
2. [reaching/grasping offsets calibration](#offsets-calibration).

## :red_circle: :large_blue_circle: Color calibration
The `calibColor` module calibrates the color of the image provided as input by maintaining the white balance.
In the red ball demo, `calibColor` is used as input to `pf3dTracker`. 
An example of output is the following:

https://user-images.githubusercontent.com/9716288/112461719-843e8a00-8d60-11eb-81eb-0acdfbf1c215.mp4


## Offsets calibration

### :new: The devised procedure

The devised procedure estimates the offsets required by `demoRedBall` merging the information from **vision** and from the **cartesian controllers**: the offset is estimated as difference between the **end-effector** provided by the controller (green) and the **ball center** provided by the visual tracker (yellow), discounted by its radius. 

<p float="right">
  <img src="https://user-images.githubusercontent.com/4537987/107539674-7cb68d80-6bc5-11eb-9129-36b9575b00b2.png" width="500" />
</p>

More specifically, the devised steps are the following:

1. run [**demoRedBal** application](https://github.com/robotology/icub-basic-demos/blob/master/demoRedBall/app/scripts/demoRedBall.xml.template) and its [dependencies](https://github.com/robotology/icub-main/blob/master/app/iCubStartup/scripts/iCubStartup.xml.template), without launching `demoRedBall` module (which is used for reaching and grasping);
2. run the [icub skin](https://github.com/robotology/icub-main/blob/master/app/skinGui/scripts/skinGuiAll.xml.template);
3. run `calibOffsets` and connect;
5. open a terminal and type: 
   ```
    yarp rpc calibOffsets/rpc 
    lookAndCalibrate part
    ``` 
    where `part` is the arm you want to calibrate.
    This will move the **desired arm/hand** in a predefined position and the **gaze** to look towards the end-effector. Now you can **push** the ball into the **middle of the palm** of the hand (to trigger calibration) (blue arrow in image). Only when the **offsets** between **end-effector** (green) and **tracking centroid** (yellow) will be computed, the service will provide an acknowledgment:

![Screenshot from 2021-04-20 14-00-07](https://user-images.githubusercontent.com/9716288/115392359-c385bc80-a1e0-11eb-9524-44e84edc7465.png)

_Note: the reaching and the grasping offsets contain a ball radius and a ball diameter respectively to avoid the hand touching the ball while reaching._  

6. in the same terminal type:

   ```
    writeToFile part
    ```

    This will create a file called `calibOffsetsResults.txt` in the `calibOffsets` context.

7. run the script `copyParamsRedBall.sh calibOffsetsResults.txt` to write the offsets into `demoRedBall` configuration file.

Now we can run `demoRedBall` with the newly adapted conf file and do all required connections. 



### Deployment

If you want to run the automatic deployment, you can refer to the instructions in the Applications section of [Robot Bazaar](https://robot-bazaar.iit.it/apps/academy/courses/1).

### :dart: Result

The following video shows the entire pipeline working on the robot. 

https://user-images.githubusercontent.com/9716288/115233678-80601680-a118-11eb-9b27-41ba01d7db8c.mp4


