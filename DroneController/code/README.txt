Trajectories:

circle.m contains the trajectory for the helix, which was generated using the quinitc spline equations. This allows any curve to be represented as a polynomial, and hence was ideal for doing both the helix and the diamond.

diamond.m contains the diamond trajectory. This also uses the same quintic spline equations to form straight lines between two points. At every end point the parameters are updated to the next points. The total operation time is equally divided between the vertices.

controller.m contains the control equations for the quadrotor. The equations are derived from the Mellinger thesis. The inputs to the controller are the current states and desired states excluding pitch and roll, which are derived. Then a simple PD controller is applied to the errors and the gain values were manually tuned for optimal operation.
