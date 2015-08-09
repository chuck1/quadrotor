# Acceleration Based

## position error

- e_5 - vector - position error
- x - vector - position
- x_ref - vector - position target
- C_5 - matrix - coefficient

e_5 = x_ref - x

e_5' = x_ref' - x'

e_5' = x_ref' - v

v_ref = C_5 e_5 + x_ref' + Lambda_5 chi_5

v_ref = C_5 \dot{\mathbf{e}}_5 + \ddot{\mathbf{x}}_{ref} + \boldsymbol\Lambda_5 \mathbf{e}_5

\[
\mathbf{e}_6 = \mathbf{v}_{ref} - \mathbf{v}
\]

\[
\mathbf{e}_6 = \mathbf{C}_5 \mathbf{e}_5 + \dot{\mathbf{x}}_{ref} + \boldsymbol\Lambda_5 \boldsymbol\chi_5 - \mathbf{v}
\]

\[
\dot{\mathbf{e}}_6 = \dot{\mathbf{v}}_{ref} - \dot{\mathbf{v}} = \Vd{v}_{ref} - \V{a}
\]

\[
\dot{\mathbf{e}}_6 = \mathbf{C}_5 \dot{\mathbf{e}}_5 + \ddot{\mathbf{x}}_{ref} + \boldsymbol\Lambda_5 \mathbf{e}_5 - \ddot{\mathbf{x}}
\]

\[
\dot{\mathbf{x}}_{ref} = \mathbf{v}_{ref} - \mathbf{C}_5 \mathbf{e}_5 - \boldsymbol\Lambda_5 \boldsymbol\chi_5
\]

\[
\dot{\mathbf{x}}_{ref} = \mathbf{e}_6 + \mathbf{v} - \mathbf{C}_5 \mathbf{e}_5 - \boldsymbol\Lambda_5 \boldsymbol\chi_5
\]

\[
\dot{\mathbf{e}}_5 = \mathbf{e}_6 - \mathbf{C}_5 \mathbf{e}_5 - \boldsymbol\Lambda_5 \boldsymbol\chi_5
\]

\[
\dot{\mathbf{e}}_6 = \mathbf{f}_6(\mathbf{e}_5,\mathbf{e}_6)
\]

\[
\mathbf{f}_6(\mathbf{e}_5,\mathbf{e}_6)
= \mathbf{C}_5 \left( \mathbf{e}_6 - \mathbf{C}_5 \mathbf{e}_5 - \boldsymbol\Lambda_5 \boldsymbol\chi_5 \right)
+ \ddot{\mathbf{x}}_{ref}
+ \boldsymbol\Lambda_5 \mathbf{e}_5
- \ddot{\mathbf{x}}
\]

\[
\Vdd{x}
= -\mathbf{f}_6(\mathbf{e}_5,\mathbf{e}_6)
+ \mathbf{C}_5 \left( \mathbf{e}_6 - \mathbf{C}_5 \mathbf{e}_5 - \boldsymbol\Lambda_5 \boldsymbol\chi_5 \right)
+ \ddot{\mathbf{x}}_{ref}
+ \boldsymbol\Lambda_5 \mathbf{e}_5
\]

\[
\mathbf{F}_{R}
= m \left(
	-\mathbf{C}_7 \mathbf{e}_5 - \mathbf{C}_8 \mathbf{e}_6
	+ \mathbf{C}_5 \left( \mathbf{e}_6 - \mathbf{C}_5 \mathbf{e}_5 - \boldsymbol\Lambda_5 \boldsymbol\chi_5 \right)
	+ \ddot{\mathbf{x}}_{ref}
	+ \boldsymbol\Lambda_5 \mathbf{e}_5
	- \mathbf{g}
	- \frac{1}{m} \mathbf{F}_{D}
\right)
\]

\[
\mathbf{F}_{R}
= m \left(
	\left( -\mathbf{C}_7 - \mathbf{C}_5^2 + \boldsymbol\Lambda_5\right) \mathbf{e}_5 +
	\left( \mathbf{C}_8 + \mathbf{C}_5 \right) \mathbf{e}_6 
	- \mathbf{C}_5 \boldsymbol\Lambda_5 \boldsymbol\chi_5
	+ \ddot{\mathbf{x}}_{ref}
	- \mathbf{g}
	- \frac{1}{m} \mathbf{F}_{D}
\right)
\]

\begin{align}\nonumber
\mathbf{F}_{R}
= m \left( \right. & \left( -\mathbf{C}_7 - \mathbf{C}_5^2 + \boldsymbol\Lambda_5 \right) \mathbf{e}_5 \\\nonumber
	+ &\left( -\mathbf{C}_8 + \mathbf{C}_5 \right) \left( \mathbf{C}_5 \mathbf{e}_5 + \dot{\mathbf{x}}_{ref} + \boldsymbol\Lambda_5 \boldsymbol\chi_5 - \mathbf{v}\right) \\\nonumber
	- &\left. \mathbf{C}_5 \boldsymbol\Lambda_5 \boldsymbol\chi_5
	+ \ddot{\mathbf{x}}_{ref}
	- \mathbf{g}
	- \frac{1}{m} \mathbf{F}_{D}
\right)
\end{align}

\begin{align}\nonumber
\frac{1}{m} \mathbf{F}_{R}
= & \left( -\mathbf{C}_7 - \mathbf{C}_5^2 + \boldsymbol\Lambda_5 + \left( -\mathbf{C}_8 + \mathbf{C}_5 \right) \mathbf{C}_5 \right) \mathbf{x}_{ref} \\\nonumber
- &\left( -\mathbf{C}_7 - \mathbf{C}_5^2 + \boldsymbol\Lambda_5 + \left( -\mathbf{C}_8 + \mathbf{C}_5 \right) \mathbf{C}_5 \right) \mathbf{x} \\\nonumber
+ &\left( -\mathbf{C}_8 + \mathbf{C}_5 \right) \dot{\mathbf{x}}_{ref} \\\nonumber
+ &\left( -\mathbf{C}_8 \right) \boldsymbol\Lambda_5 \boldsymbol\chi_5 \\\nonumber
- &\left( -\mathbf{C}_8 + \mathbf{C}_5 \right) \mathbf{v} \\\nonumber
+ &\ddot{\mathbf{x}}_{ref} - \mathbf{g} - \frac{1}{m} \mathbf{F}_{D}
\end{align}

\begin{align}\nonumber
0 = + &~ \mathbf{C}_{11} \boldsymbol\chi_5 \\\nonumber
+ &~ \mathbf{C}_9 (\mathbf{x}_{ref} - \mathbf{x}) \\\nonumber
+ &~ \mathbf{C}_{10} (\dot{\mathbf{x}}_{ref} - \mathbf{v}) \\\nonumber
+ &~ \Vdd{x}_{ref} - \Vdd{x}
\end{align}

\subsubsection{Transfer Function}

\begin{align}\nonumber
	\ddot{x} = \left( \right. & c_9 y - c_9 x + c_{10} \dot{y} - c_{10} \dot{x} \\\nonumber
	& + c_{11} \int y - c_{11} \int x \\\nonumber
	& + \left. \ddot{y} - g - \frac{F_D}{m} \right)
\end{align}


\begin{align}\nonumber
	s^2 X - s x(0) - \dot{x}(0) = \left( \right. & c_9 Y - c_9 X \\\nonumber
	& + c_{10} sY - c_{10} y(0) - c_{10} sX + c_{10} x(0) \\\nonumber
	& + c_{11} \frac{Y}{s} - c_{11} \int x \\\nonumber
	& + \left. \ddot{y} - g - \frac{F_D}{m} \right)
\end{align}







