# Interface for the NLP library

This is the design document for the `nlp_interface` package.

## Problem statement

nlp_solver solves nonlinear optimization problems of the form

$$
\begin{array}{ll}
\text{minimize} & f(x) \\
\text{subject to} & g_l \leq g(x) \leq g_u \\
& x_l \leq x \leq x_u
\end{array}
$$

where $x \in \mathbf{R}^n$ is the optimization variable.
The objective function $f: \mathbf{R}^n \rightarrow \mathbf{R}$ is a scalar-valued nonlinear function.
The nonlinear constraints are defined by the function $g: \mathbf{R}^n \rightarrow \mathbf{R}^m$,
and vectors $g_l, g_u \in \mathbf{R}^m$ so that $g_{l,i} \in \mathbf{R} \cup \{-\infty\}$ and $g_{u,i} \in \mathbf{R} \cup \{+\infty\}$ for all $i \in \{1,\ldots,m\}$.
The variable bounds are defined by vectors $x_l, x_u \in \mathbf{R}^n$ so that $x_{l,i} \in \mathbf{R} \cup \{-\infty\}$ and $x_{u,i} \in \mathbf{R} \cup \{+\infty\}$ for all $i \in \{1,\ldots,n\}$.

## Purpose / Use cases

<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This packages provides a C++ interface for the [OSQP library](https://osqp.org/docs/solver/index.html).

## Design

<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

The class `OSQPInterface` takes a problem formulation as Eigen matrices and vectors, converts these objects into
C-style Compressed-Column-Sparse matrices and dynamic arrays, loads the data into the OSQP workspace dataholder, and runs the optimizer.

## Inputs / Outputs / API

<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

The interface can be used in several ways:

1. Initialize the interface WITHOUT data. Load the problem formulation at the optimization call.

   ```cpp
       osqp_interface = OSQPInterface();
       osqp_interface.optimize(P, A, q, l, u);
   ```

2. Initialize the interface WITH data.

   ```cpp
       osqp_interface = OSQPInterface(P, A, q, l, u);
       osqp_interface.optimize();
   ```

3. WARM START OPTIMIZATION by modifying the problem formulation between optimization runs.

   ```cpp
       osqp_interface = OSQPInterface(P, A, q, l, u);
       osqp_interface.optimize();
       osqp.initializeProblem(P_new, A_new, q_new, l_new, u_new);
       osqp_interface.optimize();
   ```

   The optimization results are returned as a vector by the optimization function.

   ```cpp
   std::tuple<std::vector<double>, std::vector<double>> result = osqp_interface.optimize();
   std::vector<double> param = std::get<0>(result);
   double x_0 = param[0];
   double x_1 = param[1];
   ```

## References / External links

<!-- Optional -->

- OSQP library: <https://osqp.org/>

## Related issues

<!-- Required -->
