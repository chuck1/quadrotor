The position control law outputs net jerk in the inertial frame.

j = C_9 chi_5 + C_10 (x_ref - x) + C_11 (x_ref' - v) + C_12 (x_ref'' - a) + x_ref'''

NOTE:
the following describes how to use known jerk to calculate f\_zRB and omega\_x and omega\_y.
How jerk is initially calculated is irrelevant (could be to control position, velcity, etc.).

From this jerk, x- and y-components of angular velcity and rate of change of net thrust are calculated.

Ignoring drag, the net force in the global frame is

- F - net force on drone in global frame
- F_g - force from gravity
- F_R - force from rotors in global frame
- F_RB - force from rotors in body frame

F = F_g + F_R

Differentiate in time

F' = F_R' = m j

Rotor force in the inertial and body frame is related by

F_R = q* F_RB q

Differentiate in time

F_R' = q* ( F_RB' + F_RB x omega ) q

In our case

F_RB = [ 0 , 0 , f_zRB ]

and the above becomes

q F_R' q* = [ -omega_y f_zRB , omega_x f_zRB , f_zRB' ]

The left side is known.
First, the derivative of f\_zRB is integrated to get rotor force for the current time step.

postfix (i) represents the value at discrete time i

f_zRB(i) = f_zRB(i-1) + f_zRB' dt

The angular velocity components can then be calculated. These are the reference values for the second controller which outputs torque.

## Time Derivative of quaternion rotation

In general, if q rotates from y to x frame

\[
\V{y} = \V{q}^* \V{x} \V{q}
\]

\[
\Vd{y} = \Vd{q}^* \V{x} \V{q} + \V{q}^* \Vd{x} \V{q} + \V{q}^* \V{x} \Vd{q}
\]

\[
\Vd{y} = \V{q}^* \G{\omega}^* \V{x} \V{q} + \V{q}^* \Vd{x} \V{q} + \V{q}^* \V{x} \G{\omega} \V{q}
\]

\[
\Vd{y} = \V{q}^* \left( \G{\omega}^* \V{x} + \Vd{x} + \V{x} \G{\omega} \right) \V{q}
\]

\[
\V{q} \Vd{y} \V{q}^* = \G{\omega}^* \V{x} + \Vd{x} + \V{x} \G{\omega}
\]



which leads to the simple result

\[
\G{\omega}^* \V{x} + \Vd{x} + \V{x} \G{\omega} = 
\begin{bmatrix}
0 \\
- \omega_y a_{RB} \\
\omega_x a_{RB} \\
\dot{a}_{RB}
\end{bmatrix}
\]

Notice that $\omega_z$ does not appear because it has no effect on the translation of the vehicle.
In order to solve for $\omega_x$, $\omega_y$, and $a_{RB}$, we need a relationship between $a_{RB}$ and its derivative.
For example, we could use forward differencing.






