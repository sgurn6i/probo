-- test sequence 1
-- 2016-06-27 17:00:12 Sgurn6i
local b1 = create_body("body1")
local c1 = b1:create_controller("ctlr1")
local j1 = c1:create_joint("joint1")
local j2 = c1:create_joint("joint2")
print (c1:get_name().." children: "..c1:get_children_amt())
print (b1:get_name().." children: "..b1:get_children_amt())
b1:set_tick(50.0)
print ("tick: " .. b1:get_tick())
j1:target(22.5)
j2:target(-90)
b1:do_em_in(150)
j2:target(90)
b1:do_em_in(150)
print (j1:get_name() .. " pos: " .. j1:get_curr_pos()) 
print (j2:get_name() .. " pos: " .. j2:get_curr_pos()) 
-- b1:do_em_in(); -- error
