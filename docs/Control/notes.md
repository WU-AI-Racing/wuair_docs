---
---

# gain scheduling

## syms matlab, linearization

### Video Lecture

I link below a short (and dense) example of the Linearization of a
nonlinear set of dynamics using both MATLAB syms and a manual method:

[Linerization Example with syms in
MATLAB](https://www.youtube.com/watch?v=UQpt7qJdd1I)

### Definitions

We are some system described by the following equation, for state $`x`$
and input $`u`$:

``` math
\dot x = f(x,u)
```
, where our linearization is defined, about a linearization point
$`(x_0,u_0)`$:

``` math
\dot x (x,u) \approx f(x_0,u_0) + \left \frac{\partial f}{\partial x} \right|_{x_0,u_0} (x-x_0)  + \left \frac{\partial f}{\partial u} \right|_{x_0,u_0} (u-u_0)  
```

The jacobians are redefined to be $`A(x_0,u_0)`$ and $`B(x_0,u_0)`$ in
order of appearance. We can then group terms and define
$`(\Delta x, \Delta u) := (x-x_0, u-u_0)`$, such that:

``` math
 \Delta \dot x = A\Delta x+B\Delta u
```

Note the linearity of the derivative, such that
$`\frac{d\Delta x}{dt} = \Delta \frac{dx}{dt}`$.

## full state feedback

### Definitions

I will assume you have seen or understand the prior definitions for the
linearization of our nonlinear dynamics. Given them, We define full
state feedback as such:

``` math
u = -K^Tx \implies \Delta x = (A - BK^T)\Delta x
```

It should be noted $`u_0 = -K^Tx_0 \implies \Delta u = -K^T\Delta x`$

The matrix $`A-BK^T`$'s eigenvalues decide directly the settling time
and stability margins for our linear system. Because our system is
heavily nonlinear and MIMO (Multiple Input, Multiple Output), we cannot
simply solve for the eigenvalues and remain computationally efficient.
Instead, can use $`\texttt{place()}`$, in MATLAB or python control
packages to automate these processes.

There are conditions to this process, one such one being the linear
system be controllable, a key failure of gain scheduling.

## Iterative Eigenvalue Manipulation

Now that we can know the form of the matrix to solve for and have a
method to solve for the gain matrix $`K`$, we must build our pipeline.

``` text
function getNewControl(f,x)
    Given dynamics f, current state x,
    linearize f about x,
    solve for desired eigenvalues,
    return controller u=-K^Tx
```

text

# General Tools

## MATLAB

[syms MATLAB help
page](https://www.mathworks.com/help/symbolic/syms.html) [ode45 MATLAB
help page](https://www.mathworks.com/help/matlab/ref/ode45.html) [Pole
Placement MATLAB Tutorial (Eigenvalue
Manipulation)](https://www.youtube.com/watch?v=FXSpHy8LvmY)
