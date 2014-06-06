\subsubsection{Euler}

Tracking error
\[
\mathbf{e}_1 = \boldsymbol\theta_{ref} - \boldsymbol\theta
\]

\[
\dot{\mathbf{e}}_1 = \dot{\boldsymbol\theta}_{ref} - \dot{\boldsymbol\theta}
\]

\begin{equation}
\dot{\mathbf{e}}_1 = \dot{\boldsymbol\theta}_{ref} - \mathbf{A}_5 \boldsymbol\omega
\label{eq:e1_dyn}
\end{equation}

The $\boldsymbol\omega$ is not our control input and has its own dynamic.
So we set fot it a desired behavior and consider it as our virtual control

\begin{equation}
\boldsymbol\omega_{ref} = c_1 \mathbf{e}_1 + \mathbf{A}_5^{-1} \dot{\boldsymbol\theta}_{ref} + \lambda_1 \chi_1
\label{eq:om_ref}
\end{equation}

\begin{equation}
\mathbf{e}_2 = \boldsymbol\omega_{ref} - \boldsymbol\omega
\label{eq:e2}
\end{equation}

\[
\dot{\mathbf{e}}_2 = \dot{\boldsymbol\omega}_{ref} - \dot{\boldsymbol\omega}
\]

\begin{equation}
\dot{\mathbf{e}}_2 = c_1 \dot{\mathbf{e}}_1 + \ddot{\boldsymbol\theta}_{ref} + \lambda_1 \mathbf{e}_1 - \ddot{\boldsymbol\theta}
\label{eq:e2_dyn}
\end{equation}



Solve \eqref{eq:om_ref} for $\dot{\boldsymbol\theta}_{ref}$

\begin{equation}
\dot{\boldsymbol\theta}_{ref} = \mathbf{A}_5 \left( \boldsymbol\omega_{ref} - c_1 \mathbf{e}_1 - \lambda_1 \boldsymbol\chi_1 \right)
\label{eq:om_ref_2}
\end{equation}

Solve \eqref{eq:e2} for $\boldsymbol\omega_{ref}$

\begin{equation}
\boldsymbol\omega_{ref} = \mathbf{e}_2 + \boldsymbol\omega
\label{eq:e2_2}
\end{equation}

Combine \eqref{eq:om_ref_2} and \eqref{eq:e2_2}

\begin{equation}
\dot{\boldsymbol\theta}_{ref} =
\mathbf{A}_5 \left( \mathbf{e}_2 + \boldsymbol\omega - c_1 \mathbf{e}_1 - \lambda_1 \boldsymbol\chi_1 \right)
\label{eq:om_ref_3}
\end{equation}

Rewrite \eqref{eq:e1_dyn} using \eqref{eq:om_ref_3}

%\begin{equation}
\[
\dot{\mathbf{e}}_1 =
\mathbf{A}_5 \left( \mathbf{e}_2 + \boldsymbol\omega - c_1 \mathbf{e}_1 - \lambda_1 \boldsymbol\chi_1 \right) -
\mathbf{A}_5 \boldsymbol\omega
\]
%\end{equation}

%\begin{equation}
\[
\dot{\mathbf{e}}_1 =
\mathbf{A}_5 \left( \mathbf{e}_2 - c_1 \mathbf{e}_1 - \lambda_1 \boldsymbol\chi_1 \right)
\]
%\end{equation}

Replace $\ddot{\boldsymbol\theta}$ in \eqref{eq:e2_dyn} with its model in \eqref{eq:eom_theta}

\begin{equation}
\dot{\mathbf{e}}_2 =
c_1 \dot{\mathbf{e}}_1 +
\ddot{\boldsymbol\theta}_{ref} +
\boldsymbol\Lambda_1 \mathbf{e}_1
- \left(
	\dot{\mathbf{A}}_3 \boldsymbol\omega
	+ \mathbf{A}_5 \mathbf{I}^{-1} \left( \boldsymbol\tau - \boldsymbol\omega \times \left( \mathbf{I} \boldsymbol\omega \right) \right)
\right)
\label{eq:}
\end{equation}


\begin{align}
\nonumber
\dot{\mathbf{e}}_2 = &
\mathbf{C}_1 \mathbf{A}_5 \left( \mathbf{e}_2 - \mathbf{C}_1 \mathbf{e}_1 - \boldsymbol\Lambda_1 \boldsymbol\chi_1 \right) +
\ddot{\boldsymbol\theta}_{ref} +
\boldsymbol\Lambda_1 \mathbf{e}_1 \\
&- \left(
	\dot{\mathbf{A}}_3 \boldsymbol\omega
	+ \mathbf{A}_5 \mathbf{I}^{-1} \left( \boldsymbol\tau - \boldsymbol\omega \times \left( \mathbf{I} \boldsymbol\omega \right) \right)
\right)
\label{eq:}
\end{align}

We now choose a desirable form for the dynamics of the angular speed tracking error. An example would be

\[
\dot{\mathbf{e}}_2 = -c_2 \mathbf{e}_2 - \mathbf{e}_1
\]

but for now we will generalize this as

\[
\dot{\mathbf{e}}_2 = \mathbf{f}_2 \left( \mathbf{e}_1, \mathbf{e}_2 \right)
\]


\begin{align}
\nonumber
\mathbf{f}_2 = &
\mathbf{C}_1 \mathbf{A}_5 \left( \mathbf{e}_2 - \mathbf{C}_1 \mathbf{e}_1 - \boldsymbol\Lambda_1 \boldsymbol\chi_1 \right) +
\ddot{\boldsymbol\theta}_{ref} +
\boldsymbol\Lambda_1 \mathbf{e}_1 \\
&- \left(
	\dot{\mathbf{A}}_3 \boldsymbol\omega
	+ \mathbf{A}_5 \mathbf{I}^{-1} \left( \boldsymbol\tau - \boldsymbol\omega \times \left( \mathbf{I} \boldsymbol\omega \right) \right)
\right)
\label{eq:}
\end{align}



\begin{align}
\nonumber
\boldsymbol\tau
= \mathbf{I} \mathbf{A}_5^{-1} \left(
	-\mathbf{f}_2
	+ \mathbf{C}_1 \mathbf{A}_5 \left( \mathbf{e}_2 - \mathbf{C}_1 \mathbf{e}_1 - \boldsymbol\Lambda_1 \boldsymbol\chi_1 \right) +
	\ddot{\boldsymbol\theta}_{ref} +
	\boldsymbol\Lambda_1 \mathbf{e}_1
	- \dot{\mathbf{A}}_3 \boldsymbol\omega
\right)
+ \boldsymbol\omega \times \left( \mathbf{I} \boldsymbol\omega \right)
\label{eq:}
\end{align}




