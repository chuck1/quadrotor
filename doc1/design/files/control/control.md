# Control

output

- F\_M - vec4 - force from motors
- f\_Mx - scalar - force from individual motor

input

- f\_zRB - scalar - net body-frame force of rotors on drone
- T\_RB - vec3 - net body-frame torque of rotors on drone

F\_M = [ f\_M0 , f\_M1 , f\_M2 , f\_M3 ] = func(f\_zRB, T\_RB)

[attitude](attitude/attitude.md)

[position](position/position.md)

