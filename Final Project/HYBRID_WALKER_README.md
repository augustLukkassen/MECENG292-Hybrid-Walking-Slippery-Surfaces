# Hybrid Walker Simulation with Variable Friction

This package simulates a 3-link bipedal walker on surfaces with variable friction, implementing the hybrid dynamics from "Dynamic Walking on Slippery Surfaces" (arXiv:1812.04600).

## Files

1. **starter_code (1).m**: Generates symbolic dynamics and hybrid impact maps
2. **simulate_hybrid_walker_variable_mu.m**: Main simulation script with variable friction support
3. **run_hybrid_walker_example.m**: Example script showing how to use the simulation
4. **animate_two_link_walker.m**: Animation script (you may need to adapt for 3-link)

## Quick Start

1. **Generate functions** (run once):
   ```matlab
   run('starter_code (1).m')
   ```

2. **Run example simulation**:
   ```matlab
   run_hybrid_walker_example
   ```

## Friction Profiles

The simulation supports several friction profile options:

### 1. Random Friction
```matlab
mu_profile = 'random';  % Random between 0.3 and 1.2 at each step
```

### 2. Constant Friction
```matlab
mu_profile = 0.8;  % Constant friction coefficient
```

### 3. Friction Range
```matlab
mu_profile = [0.4, 1.0];  % Random in range [0.4, 1.0]
```

### 4. Position-Dependent Friction
```matlab
mu_profile = @(x, t) 0.5 + 0.3 * sin(0.5 * x(1));  % Varies with x position
```

### 5. Step Changes (Different Surface Patches)
```matlab
mu_values = [0.6, 0.9, 0.4, 1.0, 0.7];  % Different friction zones
zone_width = 1.0;
mu_profile = @(x, t) mu_values(min(floor(x(1)/zone_width) + 1, length(mu_values)));
```

## Usage

```matlab
[t_sol, x_sol, t_I, mu_history, domain_history] = ...
    simulate_hybrid_walker_variable_mu(x0, tspan, controller, mu_profile);
```

### Inputs:
- `x0`: Initial state [x; y; q1; q2; q3; dx; dy; dq1; dq2; dq3]
- `tspan`: Time span [t_start, t_end]
- `controller`: Function handle `u = controller(t, x)` returning [u1; u2]
- `mu_profile`: Friction profile (see options above)

### Outputs:
- `t_sol`: Time array
- `x_sol`: State array (each row is a state)
- `t_I`: Indices where impacts occurred
- `mu_history`: Friction coefficient at each step
- `domain_history`: Domain type ('stick' or 'slip') at each step

## Hybrid Dynamics Features

The simulation implements:

1. **Stick Domain**: No slip, full friction constraint
2. **Slip Domain**: Sliding with Coulomb friction
3. **Sticking Impact**: Both normal and tangential velocities set to zero
4. **Slipping Impact**: Normal velocity zero, tangential velocity may be non-zero
5. **Domain Transitions**: Automatic switching between stick and slip based on friction conditions

## Important Notes

### Friction Coefficient Limitation

The generated functions (`Fst_gen`, `dqPlus_stick_gen`, etc.) currently use a hardcoded friction coefficient `mu = 0.8` from the symbolic generation. The simulation uses the actual `mu` value for:
- Domain determination (stick vs slip)
- Impact map selection
- Guard condition evaluation

However, the **internal force computations** in the generated functions still use `mu = 0.8`. For full accuracy with variable friction, you should:

1. Modify `starter_code (1).m` to make `mu` a symbolic parameter
2. Regenerate functions with `mu` as an input parameter
3. Update the simulation to pass `mu` to these functions

### Example Modification for Variable Mu

In `starter_code (1).m`, change:
```matlab
mu = 0.8;  % Hardcoded
```

To:
```matlab
syms mu real  % Make mu symbolic
```

Then regenerate functions with `mu` as a variable:
```matlab
matlabFunction(FSt, 'File', 'gen/Fst_gen', 'Vars', {s, u, mu});
```

## Controller Design

Your controller should handle:
- **Stick domain**: Normal walking, no slip expected
- **Slip domain**: Reduced friction, may need more conservative control
- **Impact events**: Smooth transitions between steps

Example controller structure:
```matlab
function u = my_controller(t, x, domain, mu)
    % domain: 'stick' or 'slip'
    % mu: current friction coefficient
    
    if strcmp(domain, 'slip')
        % More conservative control for low friction
        Kp = 30;  % Reduced gains
    else
        % Normal control for high friction
        Kp = 50;
    end
    
    % Your control law here
    u = ...;
end
```

## Troubleshooting

1. **"Generated functions not found"**: Run `starter_code (1).m` first
2. **Simulation stops early**: Check impact detection tolerance or increase `max_steps`
3. **Unstable walking**: Adjust controller gains or check initial conditions
4. **Domain switching issues**: Verify friction profile values are reasonable (0.3-1.2 typical)

## References

- Ma, W.-L., Or, Y., & Ames, A. D. (2018). "Dynamic Walking on Slippery Surfaces: Demonstrating Stable Bipedal Gaits with Planned Ground Slippage." arXiv:1812.04600

