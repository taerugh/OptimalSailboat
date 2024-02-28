#### State Variables:
 - $(x,y)$ - global position
 - $\theta$ - global orientation
 - $v$ - velocity
 - $\dot{\theta}$ - angular velocity

#### Control Variables:
 - $\delta_r$ - rudder angle
 - $\delta_s$ - sail angle

#### Wind Variables:
 - $a_{tw}$ - true wind speed
 - $\psi_{tw}$ - true wind direction

#### Parameters:
 - $p_1$ - drift coefficient
 - $p_2$ - tangential friction
 - $p_3$ - angular friction
 - $p_4$ - sail lift
 - $p_5$ - rudder lift
 - $p_6$ - distance to sail CoE (center of effort)
 - $p_7$ - distance to mast
 - $p_8$ - distance to rudder
 - $p_9$ - mass of boat
 - $p_{10}$ - moment of inertia
 - $p_{11}$ - rudder break coefficient

#### System:
$$ \begin{align}
x =& \begin{bmatrix} x & y & \theta & v & \dot{\theta} \end{bmatrix} ^T \\
u =& \begin{bmatrix} \delta_r & \delta_s \end{bmatrix} ^T \\
w =& \begin{bmatrix} a_{tw} & \psi_{tw} \end{bmatrix} ^T \\
\dot{x} = f(x,u,w) =& 
    \begin{bmatrix}
        v \cos{\theta} + p_1 a_{tw} \cos{\psi_{tw}} \\
        v \sin{\theta} + p_1 a_{tw} \sin{\psi_{tw}} \\
        \dot{\theta} \\
        \frac{g_s \sin{\delta_s} - g_r p_{11} \sin{\delta_r} - p_2 v^2}{p_9} \\
        \frac{g_s (p_6 - p_7 \cos{\delta_s}) - g_r p_8 \cos{d_r} - p_3 \dot{\theta} v}{p_{10}}
    \end{bmatrix}
\end{align} $$

#### Apparent Wind:
$$ \begin{align}
W_{aw} =&
    \begin{bmatrix}
        a_{tw} \cos{(\psi_{tw} - \theta)} - v \\
        a_{tw} \sin{(\psi_{tw} - \theta)}
    \end{bmatrix} \\
a_{aw} =& |W_{aw}| \\
\psi_{aw} =& \text{atan2}(W_{aw}) \\
\end{align} $$

#### Sail and Rudder Forces:
$$ \begin{align}
g_s =& p_4 a_{aw} \sin{(\delta_s - \psi_{aw})} \\
g_r =& p_5 v^2 \sin{\delta_r}
\end{align} $$