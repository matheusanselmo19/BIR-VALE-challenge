-- This is the Epuck principal control script. It is threaded




actualizeLEDs=function()
    if (relLedPositions==nil) then
        relLedPositions={{-0.0343,0,0.0394},{-0.0297,0.0171,0.0394},{0,0.0343,0.0394},
                    {0.0297,0.0171,0.0394},{0.0343,0,0.0394},{0.0243,-0.0243,0.0394},
                    {0.006,-0.0338,0.0394},{-0.006,-0.0338,0.0394},{-0.0243, -0.0243,0.0394}}
    end
    if (drawingObject) then
        sim.removeDrawingObject(drawingObject)
    end
    type=sim.drawing_painttag+sim.drawing_followparentvisibility+sim.drawing_spherepoints+
        sim.drawing_50percenttransparency+sim.drawing_itemcolors+sim.drawing_itemsizes+
        sim.drawing_backfaceculling+sim.drawing_emissioncolor
    drawingObject=sim.addDrawingObject(type,0,0,bodyElements,27)
    m=sim.getObjectMatrix(ePuckBase,-1)
    itemData={0,0,0,0,0,0,0}
    sim.setLightParameters(ledLight,0)
    for i=1,9,1 do
        if (ledColors[i][1]+ledColors[i][2]+ledColors[i][3]~=0) then
            p=sim.multiplyVector(m,relLedPositions[i])
            itemData[1]=p[1]
            itemData[2]=p[2]
            itemData[3]=p[3]
            itemData[4]=ledColors[i][1]
            itemData[5]=ledColors[i][2]
            itemData[6]=ledColors[i][3]
            sim.setLightParameters(ledLight,1,{ledColors[i][1],ledColors[i][2],ledColors[i][3]})
            for j=1,3,1 do
                itemData[7]=j*0.003
                sim.addDrawingObjectItem(drawingObject,itemData)
            end
        end
    end
end

getLightSensors=function()
    data=sim.receiveData(0,'EPUCK_lightSens')
    if (data) then
        lightSens=sim.unpackFloatTable(data)
    end
    return lightSens
end

function sysCall_threadmain()
    -- Put some initialization code here:
    sim.setThreadSwitchTiming(200) -- We will manually switch in the main loop
    bodyElements=sim.getObjectHandle('ePuck_bodyElements')
    leftMotor=sim.getObjectHandle('ePuck_leftJoint')
    rightMotor=sim.getObjectHandle('ePuck_rightJoint')
    ePuck=sim.getObjectHandle('ePuck')
    ePuckBase=sim.getObjectHandle('ePuck_base')
    ledLight=sim.getObjectHandle('ePuck_ledLight')
    proxSens={-1,-1,-1,-1,-1,-1,-1,-1}
    for i=1,8,1 do
        proxSens[i]=sim.getObjectHandle('ePuck_proxSensor'..i)
    end
    maxVel=180*math.pi/180
    ledColors={{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}}

    -- Braitenberg weights for the 4 front prox sensors (avoidance):
    braitFrontSens_leftMotor={1,2,-2,-1}
    -- Braitenberg weights for the 2 side prox sensors (following):
    braitSideSens_leftMotor={-1,0}
    -- Braitenberg weights for the 8 sensors (following):
    braitAllSensFollow_leftMotor={-3,-1.5,-0.5,0.8,1,0,0,-4}
    braitAllSensFollow_rightMotor={0,1,0.8,-0.5,-1.5,-3,-4,0}
    braitAllSensAvoid_leftMotor={0,0.5,1,-1,-0.5,-0.5,0,0}
    braitAllSensAvoid_rightMotor={-0.5,-0.5,-1,1,0.5,0,0,0}

    -- Here we execute the regular thread code:
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        st=sim.getSimulationTime()
        
        velLeft=0
        velRight=0
        opMode=sim.getScriptSimulationParameter(sim.handle_self,'opMode')
        lightSens=getLightSensors()
        s=sim.getObjectSizeFactor(bodyElements) -- make sure that if we scale the robot during simulation, other values are scaled too!
        noDetectionDistance=0.07*s
        proxSensDist={noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance}
        for i=1,8,1 do
            res,dist=sim.readProximitySensor(proxSens[i])
            if (res>0) and (dist<noDetectionDistance) then
                proxSensDist[i]=dist
            end
        end
        

           
           velLeft=20
           velRight=20
           sim.setJointTargetVelocity(leftMotor,velLeft)
           sim.setJointTargetVelocity(rightMotor,velRight)         

           wl=sim.getObjectHandle('wall_left') --left wall of the simulation 
           wr=sim.getObjectHandle('wall_right')--right wall of the simulation 
           wb=sim.getObjectHandle('wall_back')--back wall of the simulation 
           wf=sim.getObjectHandle('wall_front')--front wall of the simulation 
          
           A3,AA3,AAA3,AAAA3=sim.readProximitySensor(proxSens[3])-- front sensor=> getting the handle of the object tha the sensor is detecting
           A4,AA4,AAA4,AAAA4=sim.readProximitySensor(proxSens[4])-- front sensor=> getting the handle of the object tha the sensor is detecting
           A2,AA2,AAA2,AAAA2=sim.readProximitySensor(proxSens[2])-- left front sensor => getting the handle of the object tha the sensor is detecting
           A5,AA5,AAA5,AAAA5=sim.readProximitySensor(proxSens[5])--  right front sensor => getting the handle of the object tha the sensor is detecting
           A1,AA1,AAA1,AAAA1=sim.readProximitySensor(proxSens[1])--  left side sensor=> getting the handle of the object tha the sensor is detecting
           A6,AA6,AAA6,AAAA6=sim.readProximitySensor(proxSens[6])-- right side sensor => getting the handle of the object tha the sensor is detecting
          
           finalpoint=sim.getObjectHandle('obstacle_1')-- point where the robot must stop=> getting the handle of this object

           if(AAAA4== finalpoint) or (AAAA3==finalpoint) or (AAAA2==finalpoint) or (AAAA5==finalpoint) then  -- condition to robot stop at the end point
                 velLeft=0 --
                 velRight=0
                 sim.setJointTargetVelocity(leftMotor,velLeft)
                 sim.setJointTargetVelocity(rightMotor,velRight)-- => the robot must stop 
                 break

           end
               
           -- this part of this code is to make the robot  do not touch in any wall of the simulation with the front part  
           if (AAAA4==wf and AAAA3==wf and AAAA2==wf and AAAA5==wf) or (AAAA4==wb and AAAA3==wb and AAAA2==wb and AAAA5==wb) or(AAAA4==wl and AAAA3==wl and AAAA2==wl and AAAA5==wl)or(AAAA4==wr and AAAA3==wr and AAAA2==wr and AAAA5==wr) then
                  while(AA2+AA3+AA5+AA4>0) do 
                      velLeft=5
                      velRight= -5
                      sim.setJointTargetVelocity(leftMotor,velLeft)  --the robot make a turn to avoid the wall
                      sim.setJointTargetVelocity(rightMotor,velRight)
                      A2,AA2,AAA2,AAAA2=sim.readProximitySensor(proxSens[2])
                      A3,AA3,AAA3,AAAA3=sim.readProximitySensor(proxSens[3])
                      A4,AA4,AAA4,AAAA4=sim.readProximitySensor(proxSens[4])
                      A5,AA5,AAA5,AAAA5=sim.readProximitySensor(proxSens[5])
      

                    end 
           end 
          -- this part of this code is make the robot do not with the its left side in any wall of the simulation
          if (AAAA1==wf) or (AAAA1==wb) or(AAAA1==wl) or (AAAA1==wr) then
                 while( AA1>0) do
                                   -- make a turn to right side
                      velLeft=5
                      velRight= -5
                      sim.setJointTargetVelocity(leftMotor,velLeft)
                      sim.setJointTargetVelocity(rightMotor,velRight)
                      A1,AA1,AAA1,AAAA1=sim.readProximitySensor(proxSens[1])--  

                 end

           end

           -- this part of this code is make the robot do not with the its right side in any wall of the simulation
          if (AAAA6==wf) or (AAAA6==wb) or(AAAA6==wl) or (AAAA6==wr) then
  
                   while( A6==1) do
                      -- make a turn to left side
                        velLeft=-5
                        velRight= 5
                        sim.setJointTargetVelocity(leftMotor,velLeft)
                        sim.setJointTargetVelocity(rightMotor,velRight)
                        A6,AA6,AAA6,AAAA6=sim.readProximitySensor(proxSens[6])--  left side

                    end

          end



         





      -- this part of this code is make the robor get in or get back track line and to improve evansiness on obstacles
 
        while  ((lightSens[1]<0.5) and (lightSens[2]<0.5) and (lightSens[3]<0.5)) and (proxSensDist[2]+proxSensDist[3]+proxSensDist[4]+proxSensDist[5]==noDetectionDistance*4) do
             
                   while (proxSensDist[6]>0) do
                         velLeft=-5
                         velRight= 5
                         sim.setJointTargetVelocity(leftMotor,velLeft)
                         sim.setJointTargetVelocity(rightMotor,velRight)
                         lightSens=getLightSensors()
                         proxSensDist[6]=sim.readProximitySensor(proxSens[6])
                         
                    end

                  while(lightSens[2]<0.5) do
                        velLeft=5
                        velRight= -5
                        sim.setJointTargetVelocity(leftMotor,velLeft)
                        sim.setJointTargetVelocity(rightMotor,velRight)
                        lightSens=getLightSensors()
                   end

                   

                  while (proxSensDist[1]>0) do
                     velLeft=5
                     velRight= -5
                     sim.setJointTargetVelocity(leftMotor,velLeft)
                     sim.setJointTargetVelocity(rightMotor,velRight)
                     lightSens=getLightSensors()
                     proxSensDist[1]=sim.readProximitySensor(proxSens[1])
                         --if (res==0) then
                    --        break
                      end
                   
               -- end
                 while (proxSensDist[6]>0) do
                    velLeft=-5
                    velRight= 5
                    sim.setJointTargetVelocity(leftMotor,velLeft)
                    sim.setJointTargetVelocity(rightMotor,velRight)
                    lightSens=getLightSensors()
                    proxSensDist[6]=sim.readProximitySensor(proxSens[6])
                    
                    end
             
            
        end
        if (opMode==0) then -- We wanna follow the line
            if (math.mod(st,2)>1.5) then
                intensity=1
            else
                intensity=0
            end
            for i=1,9,1 do
                ledColors[i]={intensity,0,0} -- red
            end
     
        if lightSens and ((lightSens[1]<0.5)or(lightSens[2]<0.5)or(lightSens[3]<0.5)) and (proxSensDist[2]+proxSensDist[3]+proxSensDist[4]+proxSensDist[5]==noDetectionDistance*4) then
                if (lightSens[1]>0.5) then 
                    velLeft=maxVel
                else
                    velLeft=maxVel*0.25
                end
                if (lightSens[3]>0.5) then 
                    velRight=maxVel
                else
                    velRight=maxVel*0.25
                end
            else
                velRight=maxVel
                velLeft=maxVel
                if (proxSensDist[2]+proxSensDist[3]+proxSensDist[4]+proxSensDist[5]==noDetectionDistance*4) then
                    -- Nothing in front. Maybe we have an obstacle on the side, in which case we wanna keep a constant distance with it:
                    if (proxSensDist[1]>0.25*noDetectionDistance) then
                        velLeft=velLeft+maxVel*braitSideSens_leftMotor[1]*(1-(proxSensDist[1]/noDetectionDistance))
                        velRight=velRight+maxVel*braitSideSens_leftMotor[2]*(1-(proxSensDist[1]/noDetectionDistance))
                    end
                    if (proxSensDist[6]>0.25*noDetectionDistance) then
                        velLeft=velLeft+maxVel*braitSideSens_leftMotor[2]*(1-(proxSensDist[6]/noDetectionDistance))
                        velRight=velRight+maxVel*braitSideSens_leftMotor[1]*(1-(proxSensDist[6]/noDetectionDistance))
                    end
                else
                    -- Obstacle in front. Use Braitenberg to avoid it
                    for i=1,4,1 do
                        velLeft=velLeft+maxVel*braitFrontSens_leftMotor[i]*(1-(proxSensDist[1+i]/noDetectionDistance))
                        velRight=velRight+maxVel*braitFrontSens_leftMotor[5-i]*(1-(proxSensDist[1+i]/noDetectionDistance))
                    end
                end
            end
        end
        if (opMode==1) then -- We wanna follow something!
            index=math.floor(1+math.mod(st*3,9))
            for i=1,9,1 do
                if (index==i) then
                    ledColors[i]={0,0.5,1} -- light blue
                else
                    ledColors[i]={0,0,0} -- off
                end
            end
            velRightFollow=maxVel
            velLeftFollow=maxVel
            minDist=1000
            for i=1,8,1 do
                velLeftFollow=velLeftFollow+maxVel*braitAllSensFollow_leftMotor[i]*(1-(proxSensDist[i]/noDetectionDistance))
                velRightFollow=velRightFollow+maxVel*braitAllSensFollow_rightMotor[i]*(1-(proxSensDist[i]/noDetectionDistance))
                if (proxSensDist[i]<minDist) then
                    minDist=proxSensDist[i]
                end
            end

            velRightAvoid=0
            velLeftAvoid=0
            for i=1,8,1 do
                velLeftAvoid=velLeftAvoid+maxVel*braitAllSensAvoid_leftMotor[i]*(1-(proxSensDist[i]/noDetectionDistance))
                velRightAvoid=velRightAvoid+maxVel*braitAllSensAvoid_rightMotor[i]*(1-(proxSensDist[i]/noDetectionDistance))
            end
            if (minDist>0.025*s) then minDist=0.025*s end
            t=minDist/(0.025*s)
            velLeft=velLeftFollow*t+velLeftAvoid*(1-t)
            velRight=velRightFollow*t+velRightAvoid*(1-t)
        end
        sim.setJointTargetVelocity(leftMotor,velLeft)
        sim.setJointTargetVelocity(rightMotor,velRight)
        actualizeLEDs()
        sim.switchThread() -- Don't waste too much time in here (simulation time will anyway only change in next thread switch)
    end
  




end
function sysCall_cleanup()
    -- Put some clean-up code here:
    for i=1,9,1 do
        ledColors[i]={0,0,0} -- no light
    end
    actualizeLEDs()
end