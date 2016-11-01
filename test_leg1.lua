-- test leg 1
-- 2016-10-11 12:52:51 Sgurn6i
local b1 = create_body("body1")
local c1 = b1:create_controller("ctlr1")
local pwmc1 = create_pwmc( "pca9685", "/dev/i2c-1", 0x40, 50 )
c1:attach_pwmc(pwmc1)
b1:set_tick(50.0)
print ("tick: " .. b1:get_tick())

local leg_amt = 4
local servos_per_leg = 3
local chs_per_leg = 4
local jt = {}
local pwms = {}
for ix = 0, leg_amt - 1 do
   jt[chs_per_leg * ix + 0] = c1:create_joint("j_hip"..ix)
   jt[chs_per_leg * ix + 1] = c1:create_joint("j_thigh"..ix)
   jt[chs_per_leg * ix + 2] = c1:create_joint("j_shin"..ix)
   print(jt[chs_per_leg * ix]:get_name())
   for jx = 0, servos_per_leg - 1 do
      local ch = chs_per_leg * ix + jx;
      pwms[ch] = create_pwmservo()
      print(ix, jx, "ch "..ch)
       pwms[ch]:init(ch, "rs304md" )
      jt[ch]:attach_pwmservo(  pwms[ch] )
   end
end
collectgarbage("collect")
print("0deg");
for ix = 0, leg_amt - 1 do
   jt[chs_per_leg * ix + 0]:target(0)      
   jt[chs_per_leg * ix + 1]:target(0)      
   jt[chs_per_leg * ix + 2]:target(0)
end
b1:do_em_in(0)
b1:do_em_in(450)
print("45deg");
for ix = 0, leg_amt - 1 do
   jt[chs_per_leg * ix + 0]:target(45)      
   jt[chs_per_leg * ix + 1]:target(45)      
   jt[chs_per_leg * ix + 2]:target(45)
   b1:do_em_in(200)
end
print("0deg");
for ix = 0, leg_amt - 1 do
   jt[chs_per_leg * ix + 0]:target(0)      
   jt[chs_per_leg * ix + 1]:target(0)      
   jt[chs_per_leg * ix + 2]:target(0)
   b1:do_em_in(200)
end
print("45deg");
for ix = 0, leg_amt - 1 do
   jt[chs_per_leg * ix + 0]:target(45)      
   jt[chs_per_leg * ix + 1]:target(45)      
   jt[chs_per_leg * ix + 2]:target(45)
   b1:do_em_in(200)
end
while 1 do
   b1:do_em_in(500)
end

-- end

