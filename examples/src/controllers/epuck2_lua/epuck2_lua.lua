-- epuck2_lua.lua
-- by Daniel H. Stolfi
-- ADARS project -- PCOG / SnT / University of Luxembourg

-- Use Shift + Click to select a robot
-- When a robot is selected, its variables appear in this editor

-- Use Ctrl + Click (Cmd + Click on Mac) to move a selected robot to a different location



-- Put your global variables here
counter = 0
rotate = false

--[[ This function is executed every time you press the 'execute' button ]]
function init()
   -- put your code here
   robot.colored_blob_perspective_camera.enable()
   robot.leds.set_all_rgb_colors("white")
   robot.leds.set_all_reds(true)
   robot.leds.set_front(true)
   robot.leds.set_body(true)
end

--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()
  log( string.format("Angles: %.0f %.0f %.0f %.0f %.0f %.0f %.0f %.0f",
            math.deg(robot.proximity[1].angle), math.deg(robot.proximity[2].angle),
            math.deg(robot.proximity[3].angle), math.deg(robot.proximity[4].angle),
            math.deg(robot.proximity[5].angle), math.deg(robot.proximity[6].angle),
            math.deg(robot.proximity[7].angle), math.deg(robot.proximity[8].angle)) )

  log( string.format("Proximity: %4d %4d %4d %4d %4d %4d %4d %4d",
            robot.proximity[1].value, robot.proximity[2].value, robot.proximity[3].value,
            robot.proximity[4].value, robot.proximity[5].value, robot.proximity[6].value,
            robot.proximity[7].value, robot.proximity[8].value) )

  log( string.format("Light: %4d %4d %4d %4d %4d %4d %4d %4d",
            robot.light[1].value, robot.light[2].value, robot.light[3].value,
            robot.light[4].value, robot.light[5].value, robot.light[6].value,
            robot.light[7].value, robot.light[8].value) )

  log( string.format("ToF: %4d", robot.tof) )

  log( string.format("Encoders: %4d %4d", robot.encoder.left, robot.encoder.right) )

  log( string.format("Ground: %4d %4d %4d", robot.ground[1], robot.ground[2], robot.ground[3]) )

  log( string.format("Battery -- Charge: %.3f Time Remaining: %.1f", robot.battery.available_charge, robot.battery.time_left) )

  log( string.format("Camera:") )
  for i = 1,#robot.colored_blob_perspective_camera do
     log( string.format("-- LED: %d X:%2d Y:%2d RGB:%3d,%3d,%3d",
        i,
        robot.colored_blob_perspective_camera[i].x,
        robot.colored_blob_perspective_camera[i].y,
        robot.colored_blob_perspective_camera[i].color.red,
        robot.colored_blob_perspective_camera[i].color.green,
        robot.colored_blob_perspective_camera[i].color.blue) )
  end

  if counter % 10 == 0 then
    robot.leds.set_single_rgb_color(2, robot.random.uniform(255), robot.random.uniform(255), robot.random.uniform(255))
    robot.leds.set_single_rgb_color(4, robot.random.uniform(255), robot.random.uniform(255), robot.random.uniform(255))
    robot.leds.set_single_rgb_color(6, robot.random.uniform(255), robot.random.uniform(255), robot.random.uniform(255))
    robot.leds.set_single_rgb_color(8, robot.random.uniform(255), robot.random.uniform(255), robot.random.uniform(255))
    robot.leds.set_single_red(1, robot.random.uniform(1) < 0.5)
    robot.leds.set_single_red(3, robot.random.uniform(1) < 0.5)
    robot.leds.set_single_red(5, robot.random.uniform(1) < 0.5)
    robot.leds.set_single_red(7, robot.random.uniform(1) < 0.5)
    robot.leds.set_front(robot.random.uniform(1) < 0.3333)
    robot.leds.set_body(robot.random.uniform(1) < 0.3333)
  end
  counter = counter + 1

  if not rotate and robot.tof < 100 then
     rotate = true
  end
  if rotate then
     robot.wheels.set_velocity(1, -1)
  else
     robot.wheels.set_velocity(3, 3)
  end
end

--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
   -- put your code here
end

--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
   -- put your code here
end
