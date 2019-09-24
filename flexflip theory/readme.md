# Modeling for ***flexflip***




### Generate minimum bending energy curves

In the file `generateBendingCurve.m`:

* Specify initial guess as a linear combination of basis functions
```
var_theta_init = 1*ones(1, length(var_s)) +...
                 1*sin(2*pi*var_s/var_s(end)) +...
                 1*cos(2*pi*var_s/var_s(end)) +...
                 1*sin(4*pi*var_s/var_s(end)) +...
                 1*cos(4*pi*var_s/var_s(end));
```
