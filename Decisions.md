Body Tube Diameter: 66mm/BT80
Motor Type: C
$$
M_R = 1kg \newline
gravitational..force = M_R * 9.8 = 0.05398*9.8 = 0.529 N
$$
Sure. Let's use an Estes Alpha III, to keep it basic:

Weight: 1.2 ounces empty. I will add 0.7 ounces for the engine.
Diameter: 0.976 inches
Motor: C6-7 (we'll use a big one for this rocket) 
Note that the Motor Dimension WWW Page tells us that the Estes motor has a 90% "rating" - you'll see what that means in a moment.
Now let's follow the equations above and see what we get:

Compute the Useful Terms
Mass of the rocket M = (weight in ounces)/16/2.2 = (1.2+0.7) / 16 / 2.2 = 0.05398 kg
Area of the rocket: A = pi*r^2 = 3.14*(0.5*0.976/12*0.3048)^2 = 0.000483 m^2
Compute wind resistance factor: k = 0.5*rho*Cd*A = 0.5*1.2*0.75*0.000483 = 0.000217
A C6 motor has a nominal impulse of 10 N-s and thrust of 6 N. The "rating" cited above applies to the impulse, giving us an actual impulse for an Estes C6 of 10*90% = 9 N-s.
Compute the burn time t = I / T = 9 / 6 = 1.5 sec.
The gravitational force = M*9.8 = 0.05398*9.8 = 0.529 newton
Compute My Terms
q = sqrt([T - M*g] / k) = sqrt([6 - 0.05398*9.8] / 0.000217) = 158.8
x = 2*k*q / M = 2*0.000217*158.8 / 0.05398 = 1.277
Now the good stuff:
v = q*[1-exp(-x*t)] / [1+exp(-x*t)] 
= 158.8*[1-exp(-1.277*1.5)] / [1+exp(-1.277*1.5)] = 118.0 m/s 
if this number doesn't mean anything to you, multiply by 2.237 to get velocity at burn-out in mph: 118.0*2.237 = 264.0 mph! And you won't get a ticket!
yb = [-M / (2*k)]*ln([T - M*g - k*v^2] / [T - M*g]) 
= [-0.05398 / (2*0.000217)]*ln([6 - 0.05398*9.8 - 0.000217*118.0^2] / [6 - 0.05398*9.8]) = 99.95 m 
remember this is the height reached during boost. Multiply by 3.3 to get it in feet: 99.95*3.3 = 329.8 feet.
yc = [+M / (2*k)]*ln([M*g + k*v^2] / [M*g]) 
= [+0.05398 / (2*0.000217)]*ln([0.05398*9.8 + 0.000217*118.0^2] / [0.05398*9.8]) 
= 236.8 m = 781.4 feet. 
Notice: the rocket goes more than twice as far after the burn as during the burn! 

AND THE TOTAL ALTITUDE IS... yb + yc = 329.8 + 781.4 = 1,111 feet
