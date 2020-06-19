# Systems

## Cart-Pole
A pole is attached by an un-actuated joint to a cart, which moves along a frictionless track. The cart is set to random initial conditions, and must reach a random target location while keeping the pole vertical.\
The environment is designed to look like the OpenAI gym environment, but is more challenging than that implemented by Rich Sutton et al. and used in OpenAI gym.\
The cart-pole must simultaneously move to a target location and keep the pole balanced. It obeys the Lagrangian dynamics formulation, includes friction, and is given random initial conditions in a much more extreme range.

Derivation of the Lagrangian mechanics can be found in:
```bash
cartpole.pdf
```
