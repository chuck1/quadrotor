# Control

## motor speeds

for any given motor

- s - scalar - motor speed
- f - scalar - lift force of rotor

s = func(f)

the function is based on propeller shape

## individual motor force

output

- F\_M - vec4 - force from motors
- f\_Mx - scalar - force from individual motor

input

- f\_zRB - scalar - z-component net body-frame force of rotors on drone
- T\_RB - vec3 - net body-frame torque of rotors on drone

F\_M = [ f\_M0 , f\_M1 , f\_M2 , f\_M3 ] = func(f\_zRB, T\_RB)

methods for calculating f\_zRB are descibed in

[position](position/position.md)

method for calculating T\_RB are described in

[attitude](attitude/attitude.md)
